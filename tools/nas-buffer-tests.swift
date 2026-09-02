// Run with tools/test-nas-buffer. Uses a generated, non-personal MP4 fixture.
import AVFoundation
import CryptoKit

@main
struct NASBufferTests {
    struct Fingerprint: Equatable {
        let samples: Int
        let bytes: Int
        let digest: String
    }

    static func fingerprint(_ asset: AVAsset, type: AVMediaType,
                            range: CMTimeRange? = nil) async throws -> Fingerprint {
        let tracks = try await asset.loadTracks(withMediaType: type)
        let track = tracks[0]
        let reader = try AVAssetReader(asset: asset)
        if let range { reader.timeRange = range }
        let output = AVAssetReaderTrackOutput(track: track, outputSettings: nil)
        reader.add(output)
        guard reader.startReading() else { throw reader.error ?? URLError(.cannotDecodeRawData) }
        var hash = SHA256()
        var samples = 0
        var bytes = 0
        while let sample = output.copyNextSampleBuffer() {
            if let buffer = CMSampleBufferGetDataBuffer(sample) {
                let size = CMBlockBufferGetDataLength(buffer)
                var data = Data(count: size)
                let status = data.withUnsafeMutableBytes {
                    CMBlockBufferCopyDataBytes(buffer, atOffset: 0, dataLength: size,
                                               destination: $0.baseAddress!)
                }
                precondition(status == kCMBlockBufferNoErr)
                hash.update(data: data)
                bytes += size
            }
            samples += 1
        }
        guard reader.status == .completed else { throw reader.error ?? URLError(.cannotDecodeRawData) }
        return Fingerprint(samples: samples, bytes: bytes, digest: String(describing: hash.finalize()))
    }

    @MainActor
    static func checkSeeks(_ url: URL) async throws {
        let buffered = NASVideoAsset(url: url)
        let player = AVPlayer(playerItem: AVPlayerItem(asset: buffered.asset))
        player.isMuted = true
        var started = 0
        let seeker = VideoSeeker(player: player) {
            started += 1
            buffered.prepareForSeek()
        }
        var completed: [Int: Bool] = [:]
        await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
            for i in 1...3 {
                seeker.seek(to: CMTime(seconds: Double(i * 2), preferredTimescale: 600),
                            toleranceBefore: .zero, toleranceAfter: .zero) { success in
                    completed[i] = success
                    if i == 3 { done.resume() }
                }
            }
        }
        precondition(started == 2, "Middle target should be replaced before it starts")
        precondition(completed == [1: false, 2: false, 3: true])
        precondition(abs(player.currentTime().seconds - 6) < 0.05)
        precondition(player.rate == 0, "Seeking a paused video must not resume playback")
        print("PASS rapid seeks finish at the latest target and preserve pause")
        player.rate = 1.5
        await withCheckedContinuation { (done: CheckedContinuation<Void, Never>) in
            seeker.seek(to: CMTime(seconds: 2, preferredTimescale: 600),
                        toleranceBefore: .zero, toleranceAfter: .zero) { success in
                precondition(success && player.rate == 0)
                done.resume()
            }
            player.pause() // user pauses while the seek is in progress
        }
        print("PASS pausing during a seek remains paused")
        seeker.stop()
        player.replaceCurrentItem(with: nil)
        buffered.stop()
    }

    static func main() async throws {
        let url = URL(fileURLWithPath: CommandLine.arguments[1])
        precondition(!NASVideoAsset.isNetworkFile(url), "Fixture should be on local storage")
        let direct = AVURLAsset(url: url)
        var buffered: NASVideoAsset? = NASVideoAsset(url: url)
        weak var released: NASVideoAsset?
        released = buffered
        let duration = try await direct.load(.duration)
        let bufferedDuration = try await buffered!.asset.load(.duration)
        precondition(duration == bufferedDuration)
        // Exact compressed sample hashes catch offset mistakes, short reads,
        // metadata reads at the tail, and accidental mixing of the two tracks.
        for type: AVMediaType in [.video, .audio] {
            let expected = try await fingerprint(direct, type: type)
            let actual = try await fingerprint(buffered!.asset, type: type)
            precondition(expected.samples > 0 && expected == actual)
            print("PASS \(type.rawValue): \(actual.samples) samples, \(actual.bytes) identical bytes")
        }
        let tail = CMTimeRange(start: duration - CMTime(seconds: 1, preferredTimescale: 600),
                               duration: CMTime(seconds: 1, preferredTimescale: 600))
        let expectedTail = try await fingerprint(direct, type: .video, range: tail)
        let actualTail = try await fingerprint(buffered!.asset, type: .video, range: tail)
        precondition(expectedTail.samples > 0 && expectedTail == actualTail)
        print("PASS seek to final second / EOF")
        buffered!.prepareForSeek()
        buffered!.stop()
        buffered!.stop() // idempotent cleanup
        buffered = nil
        for _ in 0..<100 where released != nil {
            try await Task.sleep(nanoseconds: 20_000_000)
        }
        precondition(released == nil, "Stopped asset must release its file/cache")
        print("PASS stop releases loader")

        let missing = NASVideoAsset(url: url.appendingPathExtension("missing"))
        do {
            _ = try await missing.asset.load(.duration)
            preconditionFailure("Missing source should fail")
        } catch {
            print("PASS missing source reports an error")
        }
        missing.stop()
        let cancelled = NASVideoAsset(url: url)
        cancelled.stop()
        do {
            _ = try await cancelled.asset.load(.duration)
            preconditionFailure("Stopped source should fail")
        } catch {
            print("PASS cancellation during open")
        }
        try await checkSeeks(url)
    }
}
