// The headset audio check must answer within its deadline without blocking
// the main thread: yes for a running output, no for one that cannot start.
import AppKit
import CoreAudio

func requireAudio(_ condition: Bool, _ message: String) {
    if !condition { fatalError(message) }
}

@main
struct AudioOutputTests {
    static func check(_ device: AudioDeviceID) -> (responding: Bool, seconds: Double, longestTurn: Double) {
        var answer: Bool?
        let start = CACurrentMediaTime()
        AudioOutputCheck.run(device: device) { answer = $0 }
        var longest = 0.0
        while answer == nil, CACurrentMediaTime() - start < 3 {
            let turn = CACurrentMediaTime()
            RunLoop.main.run(until: Date(timeIntervalSinceNow: 0.01))
            longest = max(longest, CACurrentMediaTime() - turn)
        }
        requireAudio(answer != nil, "the audio check must answer within its deadline")
        return (answer!, CACurrentMediaTime() - start, longest)
    }

    static func main() {
        _ = NSApplication.shared
        var device = AudioDeviceID(0)
        var size = UInt32(MemoryLayout<AudioDeviceID>.size)
        var address = AudioObjectPropertyAddress(
            mSelector: kAudioHardwarePropertyDefaultOutputDevice,
            mScope: kAudioObjectPropertyScopeGlobal,
            mElement: kAudioObjectPropertyElementMain)
        requireAudio(AudioObjectGetPropertyData(AudioObjectID(kAudioObjectSystemObject),
            &address, 0, nil, &size, &device) == noErr && device != 0, "no default audio output")
        let running = check(device)
        requireAudio(running.responding, "the default output must be reported as running")
        let missing = check(AudioDeviceID(0x7FFF_FFF0))
        requireAudio(!missing.responding, "a device that cannot start must be reported as not running")
        requireAudio(max(running.longestTurn, missing.longestTurn) < 0.1,
            "the audio check must not block the main thread")
        print(String(format: "PASS: audio output check (running %.0f ms, missing %.0f ms, main thread free)",
            running.seconds * 1000, missing.seconds * 1000))
    }
}
