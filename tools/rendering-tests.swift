import AppKit
import MetalKit

func requireRendering(_ condition: Bool, _ message: String) {
    if !condition { fatalError(message) }
}

@main
struct RenderingTests {
    static func main() throws {
        let period = 1.0 / 120.0
        let scanout = period * 2040.0 / 2200.0
        let lead = PoseTiming.lookahead(now: 10, presentation: 10 + period,
            period: period, scanout: scanout, adjustment: 0)
        let delayed = PoseTiming.lookahead(now: 10.006, presentation: 10 + period,
            period: period, scanout: scanout, adjustment: 0)
        requireRendering(abs(Double(lead - delayed) - 0.006) < 0.000001,
            "Pose must predict to the same presentation time after a scheduling delay")
        let expired = PoseTiming.lookahead(now: 10.01, presentation: 10 + period,
            period: period, scanout: scanout, adjustment: 0)
        requireRendering(expired > Float(scanout * 0.5) && expired < Float(period + scanout * 0.5),
            "An expired deadline must roll forward to the next refresh")
        requireRendering(PoseTiming.lookahead(now: 0, presentation: 2, period: period,
            scanout: scanout, adjustment: 0) == 0.1, "Extrapolation must stay bounded")
        print("PASS: pose lookahead follows the presentation deadline")

        // Compositor model measured on a PS VR2: one refresh after the display
        // link's target, frames in order, one per refresh. A missed refresh
        // keeps the queue a frame deeper until a display tick is skipped.
        // Feedback arrives when the frame reaches the panel; a frame renders
        // about 1.3 refreshes before its display link target.
        func simulate(ticks: [Int], misses: Set<Int>, cancel: Set<Int> = [],
                      discard: Set<Int> = []) -> [(predicted: Double, presented: Double)] {
            let schedule = PresentationSchedule()
            var inFlight: [(slot: PresentationSchedule.Slot, presented: Double)] = []
            var shown: [(predicted: Double, presented: Double)] = []
            var lastShown = -Double.infinity
            for (index, tick) in ticks.enumerated() {
                let target = 100 + Double(tick) * period
                let renderTime = target - 1.3 * period
                while let first = inFlight.first, first.presented < renderTime {
                    inFlight.removeFirst()
                    schedule.complete(first.slot, presented: first.presented, period: period)
                }
                let slot = schedule.schedule(target: target, period: period)
                if cancel.contains(index) {
                    schedule.cancel(slot)
                    continue
                }
                if discard.contains(index) {
                    schedule.complete(slot, presented: 0, period: period)
                    continue
                }
                var presented = max(target + period, lastShown + period)
                if misses.contains(index) { presented += period }
                lastShown = presented
                inFlight.append((slot, presented))
                shown.append((slot.time, presented))
            }
            return shown
        }
        func mispredicted(_ frames: ArraySlice<(predicted: Double, presented: Double)>) -> Int {
            frames.filter { abs($0.predicted - $0.presented) > period * 0.5 }.count
        }
        let steady = simulate(ticks: Array(0..<120), misses: [])
        requireRendering(mispredicted(steady[5...]) == 0,
            "A constant compositor delay must be predicted exactly")
        // Miss at frame 40, then the queue stays deep until tick 81 is skipped.
        let deep = simulate(ticks: Array(0..<80) + Array(81..<160), misses: [40])
        // The missed frame itself, plus the three rendered before it is reported.
        requireRendering(mispredicted(deep[5...]) <= 4,
            "Only frames rendered before a miss is reported may be mispredicted")
        requireRendering(mispredicted(deep[44..<80]) == 0,
            "A deeper queue after a missed refresh must be predicted")
        requireRendering(mispredicted(deep[80...]) == 0,
            "A skipped display tick must return prediction to the shorter delay")
        let interrupted = simulate(ticks: Array(0..<120), misses: [], cancel: [50], discard: [70])
        requireRendering(mispredicted(interrupted[5...]) == 0,
            "Frames never shown must not take a refresh in the prediction")
        print("PASS: presentation schedule follows compositor queue depth")

        let useHeadset = CommandLine.arguments.contains("--headset")
        let targetScreen = useHeadset
            ? NSScreen.screens.first { $0.localizedName.localizedCaseInsensitiveContains("PS VR2") }
            : NSScreen.screens.first
        guard let device = MTLCreateSystemDefaultDevice(),
              let screen = targetScreen else {
            fatalError("This integration test requires Metal and a desktop session")
        }
        _ = NSApplication.shared
        let foreground = CommandLine.arguments.contains("--foreground")
        let previousApp = NSWorkspace.shared.frontmostApplication
        NSApp.setActivationPolicy(foreground ? .regular : .prohibited)
        let rect = NSRect(x: screen.visibleFrame.minX + 12, y: screen.visibleFrame.minY + 12,
                          width: 320, height: 164)
        let window = NSWindow(contentRect: rect, styleMask: [.borderless],
                              backing: .buffered, defer: false)
        window.isReleasedWhenClosed = false
        let view = MTKView(frame: NSRect(origin: .zero, size: rect.size), device: device)
        view.isPaused = true
        view.enableSetNeedsDisplay = false
        view.autoResizeDrawable = false
        view.drawableSize = useHeadset ? CGSize(width: 4000, height: 2040) : CGSize(width: 640, height: 328)
        window.contentView = view
        window.orderFrontRegardless()
        window.setFrame(rect, display: true)
        if foreground { NSApp.activate(ignoringOtherApps: true) }
        let layer = view.layer as! CAMetalLayer
        let screenNumber = screen.deviceDescription[NSDeviceDescriptionKey("NSScreenNumber")] as! NSNumber
        let displayID = CGDirectDisplayID(truncating: screenNumber)
        layer.drawableSize = view.drawableSize
        print("[test] screen=\(window.screen?.localizedName ?? "unknown") target=\(layer.drawableSize)")
        let renderer = try Renderer(device: device, config: PlaybackConfig(),
            calibration: [-0.10, 0, 0.10, 0, 1, 0, 1, 0])
        let video: VideoSource?
        if let index = CommandLine.arguments.firstIndex(of: "--video"),
           index + 1 < CommandLine.arguments.count {
            video = VideoSource(url: URL(fileURLWithPath: CommandLine.arguments[index + 1]), device: device)
            renderer.video = video
            video?.player.volume = 0
            video?.player.play()
        } else {
            video = nil
        }
        renderer.startRendering(layer: layer, displayID: displayID)
        renderer.draw(in: view)
        let timer = Timer.scheduledTimer(withTimeInterval: period, repeats: true) { _ in
            renderer.draw(in: view)
        }
        // Exercise the production Renderer, its mailbox, the real display link
        // and GPU. Only the USB source is synthetic.
        RunLoop.current.run(until: Date(timeIntervalSinceNow: 0.8))
        if let video {
            requireRendering(video.textureY != nil && !video.textureBacking.isEmpty,
                "Video decoding must publish retained Core Video surfaces")
        }
        let before = test_prediction_count()
        Thread.sleep(forTimeInterval: 0.25) // intentional main-thread stall
        let duringStall = test_prediction_count() - before
        requireRendering(duringStall >= 10,
            "Head pose stopped during the main-thread stall: only \(duringStall) samples")
        print("PASS: \(duringStall) fresh render poses during a 250 ms main-thread stall")
        RunLoop.current.run(until: Date(timeIntervalSinceNow: 3.5))
        timer.invalidate()
        video?.stop()
        renderer.video = nil
        renderer.draw(in: view)
        renderer.stopRendering()
        let afterStop = test_prediction_count()
        Thread.sleep(forTimeInterval: 0.05)
        requireRendering(afterStop == test_prediction_count(), "Rendering must stop before USB teardown")
        // Also check shutdown immediately after starting the display thread.
        for _ in 0..<5 {
            let driver = HeadsetFrameDriver(layer: layer, displayID: displayID, hz: 120) { drawable, _ in
                drawable.present()
                return true
            }!
            requireRendering(driver.start(), "The display link must start")
            driver.stop()
        }
        window.close()
        if foreground { previousApp?.activate() }
        print("PASS: render-thread shutdown, including immediate start/stop")
    }
}
