// Run with tools/test-audio-playback. The fixture has continuous sine audio.
import AVFoundation
import MediaToolbox
import QuartzCore

final class AudioMeter {
    private let lock = NSLock()
    private var frames = 0
    private var energy = 0.0
    private var errors = 0

    @MainActor
    func install(on item: AVPlayerItem) async throws {
        let track = try await item.asset.loadTracks(withMediaType: .audio)[0]
        var callbacks = MTAudioProcessingTapCallbacks(
            version: kMTAudioProcessingTapCallbacksVersion_0,
            clientInfo: Unmanaged.passRetained(self).toOpaque(),
            init: { _, info, storage in storage.pointee = info },
            finalize: { tap in
                Unmanaged<AudioMeter>.fromOpaque(MTAudioProcessingTapGetStorage(tap)).release()
            },
            prepare: { _, _, format in
                precondition(format.pointee.mFormatFlags & kAudioFormatFlagIsFloat != 0)
                precondition(format.pointee.mBitsPerChannel == 32)
            },
            unprepare: { _ in },
            process: { tap, requested, _, buffers, frames, flags in
                let status = MTAudioProcessingTapGetSourceAudio(tap, requested, buffers, flags, nil, frames)
                let meter = Unmanaged<AudioMeter>.fromOpaque(MTAudioProcessingTapGetStorage(tap))
                    .takeUnretainedValue()
                var energy = 0.0
                for buffer in UnsafeMutableAudioBufferListPointer(buffers) {
                    guard let data = buffer.mData else { continue }
                    if status == noErr {
                        let samples = data.assumingMemoryBound(to: Float.self)
                        for i in 0..<min(Int(buffer.mDataByteSize) / 4, 128) {
                            energy += Double(abs(samples[i]))
                        }
                    }
                    // Keep decoding/output active without making test noise.
                    memset(data, 0, Int(buffer.mDataByteSize))
                }
                meter.lock.lock()
                meter.frames += status == noErr ? frames.pointee : 0
                meter.energy += energy
                meter.errors += status == noErr ? 0 : 1
                meter.lock.unlock()
            })
        var tap: MTAudioProcessingTap?
        precondition(MTAudioProcessingTapCreate(kCFAllocatorDefault, &callbacks,
            kMTAudioProcessingTapCreationFlag_PreEffects, &tap) == noErr)
        let parameters = AVMutableAudioMixInputParameters(track: track)
        parameters.audioTapProcessor = tap
        let mix = AVMutableAudioMix()
        mix.inputParameters = [parameters]
        item.audioMix = mix
    }

    func takeReading() -> (frames: Int, energy: Double, errors: Int) {
        lock.lock()
        defer { lock.unlock() }
        let result = (frames, energy, errors)
        frames = 0
        energy = 0
        errors = 0
        return result
    }
}

@main
struct AudioPlaybackTests {
    @MainActor
    static func main() async throws {
        let url = URL(fileURLWithPath: CommandLine.arguments[1])
        let buffered = NASVideoAsset(url: url)
        let item = AVPlayerItem(asset: buffered.asset)
        item.audioTimePitchAlgorithm = .timeDomain
        let video = AVPlayerItemVideoOutput(pixelBufferAttributes: [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange,
        ])
        item.add(video)
        let meter = AudioMeter()
        try await meter.install(on: item)
        let player = AVPlayer(playerItem: item)
        player.play()
        let started = CACurrentMediaTime()
        var lastCheck = started
        var videoFrames = 0
        var previousPTS = CMTime.invalid
        while player.currentTime().seconds < 65 {
            try await Task.sleep(nanoseconds: 8_333_333)
            let now = CACurrentMediaTime()
            precondition(now - started < 100, "Playback did not reach 65 seconds")
            precondition(item.status != .failed, "Playback failed: \(String(describing: item.error))")
            var pts = CMTime.invalid
            let time = video.itemTime(forHostTime: now)
            if video.copyPixelBuffer(forItemTime: time, itemTimeForDisplay: &pts) != nil,
               pts != previousPTS {
                videoFrames += 1
                previousPTS = pts
            }
            if now - lastCheck >= 2 {
                let audio = meter.takeReading()
                if now - started > 6 {
                    precondition(audio.errors == 0 && audio.frames >= 48_000 && audio.energy > 1,
                                 "Audio stopped at \(player.currentTime().seconds)s: \(audio)")
                    precondition(videoFrames >= 30, "Video stopped during audio test")
                }
                lastCheck = now
                videoFrames = 0
            }
        }
        player.pause()
        player.replaceCurrentItem(with: nil)
        buffered.stop()
        print("PASS audio and video remain active beyond 65 seconds with cache eviction")
    }
}
