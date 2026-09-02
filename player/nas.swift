// Bounded read-ahead for files on mounted network volumes. AVFoundation asks
// for byte ranges through a private URL scheme; SMB reads happen on a worker,
// in 2 MiB blocks, rather than on the render or resource-loader queues.
import AVFoundation
import UniformTypeIdentifiers
import Darwin

final class NASVideoAsset: NSObject, AVAssetResourceLoaderDelegate {
    let asset: AVURLAsset
    private let source: URL
    private let queue = DispatchQueue(label: "player.nas.requests", qos: .userInitiated)
    private let ioQueue = DispatchQueue(label: "player.nas.reads", qos: .userInitiated)
    private let blockSize: Int64 = 2 * 1024 * 1024
    private let maxBlocks = 128 // 256 MiB, plus one in-flight read and responses
    private let forwardBlocks: Int64 = 120 // leave room for metadata/audio/backtracking
    private var file: FileHandle? // only accessed on ioQueue
    private var length: Int64?
    private var failure: Error?
    private var stopped = false
    private var reading = false
    private var requests: [AVAssetResourceLoadingRequest] = []
    private struct Block {
        let data: Data
        var use: UInt64
    }
    private var blocks: [Int64: Block] = [:]
    private var clock: UInt64 = 0
    private var prefetchAnchor: Int64?
    private var bytesRead = 0
    private var readSeconds = 0.0
    private var maxReadSeconds = 0.0
    private var hits = 0
    private var misses = 0
    private var lastReport = ProcessInfo.processInfo.systemUptime

    static func isNetworkFile(_ url: URL) -> Bool {
        guard url.isFileURL else { return false }
        var info = statfs()
        return url.withUnsafeFileSystemRepresentation { path in
            guard let path, statfs(path, &info) == 0 else { return false }
            return info.f_flags & UInt32(MNT_LOCAL) == 0
        }
    }

    init(url: URL) {
        source = url
        // The original path remains the identity used for resume positions,
        // format detection, and UI. Only AVFoundation sees this private URL.
        let proxy = URL(string: "psvr2-nas://\(UUID().uuidString)/video")!
            .appendingPathExtension(url.pathExtension)
        asset = AVURLAsset(url: proxy)
        super.init()
        asset.resourceLoader.setDelegate(self, queue: queue)
        ioQueue.async { [self] in
            do {
                let handle = try FileHandle(forReadingFrom: source)
                file = handle
                var info = stat()
                guard fstat(handle.fileDescriptor, &info) == 0 else {
                    throw POSIXError(POSIXErrorCode(rawValue: errno) ?? .EIO)
                }
                let size = Int64(info.st_size)
                queue.async { [self] in
                    guard !stopped else { return }
                    length = size
                    print("[nas] read-ahead enabled: 256 MiB cache, 2 MiB reads, \(size) bytes")
                    pump()
                }
            } catch {
                queue.async { [self] in fail(error) }
            }
        }
    }

    // A syscall already in flight can finish, but no old speculative reads
    // are scheduled after a seek. Cached ranges remain useful for rewinding.
    func prepareForSeek() {
        queue.async { [self] in prefetchAnchor = nil }
    }

    func stop() {
        queue.async { [self] in
            guard !stopped else { return }
            stopped = true
            prefetchAnchor = nil
            let pending = requests
            requests.removeAll()
            blocks.removeAll()
            for request in pending where !request.isCancelled && !request.isFinished {
                request.finishLoading(with: URLError(.cancelled))
            }
            // Serialize close with reads so a cancelled source cannot close
            // a descriptor that a worker is still using.
            ioQueue.async { [self] in
                try? file?.close()
                file = nil
            }
        }
    }

    func resourceLoader(_ resourceLoader: AVAssetResourceLoader,
                        shouldWaitForLoadingOfRequestedResource request: AVAssetResourceLoadingRequest) -> Bool {
        guard !stopped else {
            request.finishLoading(with: URLError(.cancelled))
            return true
        }
        requests.append(request)
        pump()
        return true
    }

    func resourceLoader(_ resourceLoader: AVAssetResourceLoader,
                        didCancel request: AVAssetResourceLoadingRequest) {
        requests.removeAll { $0 === request }
        if requests.isEmpty { prefetchAnchor = nil }
        pump()
    }

    // All request/cache state is confined to queue. Never wait here for SMB.
    private func pump() {
        guard !stopped else { return }
        if let failure {
            fail(failure)
            return
        }
        guard let length else { return }
        var missing: Int64?
        // New seek requests take priority over older reads. Cached requests
        // for both tracks can still be fulfilled in the same pass.
        for request in requests.reversed() where !request.isCancelled && !request.isFinished {
            if let info = request.contentInformationRequest {
                info.contentType = UTType(filenameExtension: source.pathExtension)?.identifier
                    ?? AVFileType.mp4.rawValue
                info.contentLength = length
                info.isByteRangeAccessSupported = true
                info.isEntireLengthAvailableOnDemand = false
            }
            guard let dataRequest = request.dataRequest else {
                request.finishLoading()
                continue
            }
            let start = dataRequest.requestedOffset
            guard start >= 0, start <= length, dataRequest.requestedLength >= 0 else {
                request.finishLoading(with: URLError(.badServerResponse))
                continue
            }
            let end = dataRequest.requestsAllDataToEndOfResource ? length
                : start + min(Int64(dataRequest.requestedLength), length - start)
            var offset = max(start, dataRequest.currentOffset)
            while offset < end {
                let index = offset / blockSize
                guard var block = blocks[index] else {
                    if missing == nil {
                        missing = index
                        prefetchAnchor = index
                        misses += 1
                    }
                    break
                }
                clock &+= 1
                block.use = clock
                blocks[index] = block
                let lower = Int(offset - index * blockSize)
                let count = min(block.data.count - lower, Int(end - offset))
                guard count > 0 else {
                    request.finishLoading(with: URLError(.cannotDecodeRawData))
                    break
                }
                dataRequest.respond(with: block.data.subdata(in: lower..<(lower + count)))
                offset += Int64(count)
                hits += 1
                if missing == nil { prefetchAnchor = index }
            }
            if offset >= end && !request.isFinished { request.finishLoading() }
        }
        requests.removeAll { $0.isCancelled || $0.isFinished }
        guard !reading else { return }
        if let missing {
            readBlock(missing, length: length)
        } else if let anchor = prefetchAnchor, length > 0 {
            let last = min((length - 1) / blockSize, anchor + forwardBlocks)
            if anchor < last,
               let next = ((anchor + 1)...last).first(where: { blocks[$0] == nil }) {
                readBlock(next, length: length)
            }
        }
    }

    private func readBlock(_ index: Int64, length: Int64) {
        reading = true
        let offset = index * blockSize
        let count = Int(min(blockSize, length - offset))
        ioQueue.async { [self] in
            let started = ProcessInfo.processInfo.systemUptime
            let result: Result<Data, Error>
            do {
                guard let file else { throw URLError(.fileDoesNotExist) }
                try file.seek(toOffset: UInt64(offset))
                var data = Data()
                // FileHandle may legally return fewer bytes than requested.
                while data.count < count {
                    guard let part = try file.read(upToCount: count - data.count), !part.isEmpty else {
                        throw URLError(.cannotDecodeRawData)
                    }
                    data.append(part)
                }
                result = .success(data)
            } catch {
                result = .failure(error)
            }
            let elapsed = ProcessInfo.processInfo.systemUptime - started
            queue.async { [self] in
                reading = false
                guard !stopped else { return }
                switch result {
                case .success(let data):
                    if blocks.count >= maxBlocks,
                       let oldest = blocks.min(by: { $0.value.use < $1.value.use })?.key {
                        blocks.removeValue(forKey: oldest)
                    }
                    clock &+= 1
                    blocks[index] = Block(data: data, use: clock)
                    bytesRead += data.count
                    readSeconds += elapsed
                    maxReadSeconds = max(maxReadSeconds, elapsed)
                    report()
                    pump()
                case .failure(let error): fail(error)
                }
            }
        }
    }

    private func fail(_ error: Error) {
        guard !stopped else { return }
        if failure == nil { print("[nas] read failed: \(error.localizedDescription)") }
        failure = error
        let pending = requests
        requests.removeAll()
        prefetchAnchor = nil
        for request in pending where !request.isCancelled && !request.isFinished {
            request.finishLoading(with: error)
        }
    }

    private func report() {
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastReport >= 2 else { return }
        let cached = blocks.values.reduce(0) { $0 + $1.data.count }
        print(String(format: "[nas] cache=%.0fMiB read=%.1fMiB/s readMax=%.0fms hits=%d misses=%d pending=%d",
                     Double(cached) / 1048576, Double(bytesRead) / 1048576 / max(readSeconds, 0.001),
                     maxReadSeconds * 1000, hits, misses, requests.count))
        bytesRead = 0
        readSeconds = 0
        maxReadSeconds = 0
        hits = 0
        misses = 0
        lastReport = now
    }
}
