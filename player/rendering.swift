import Foundation
import QuartzCore
import Metal
import CoreVideo

// Publish complete, immutable frames. Rendering never waits for the main
// thread to finish decoding a video frame or painting the HUD.
final class LatestFrame<Value> {
    private let lock = NSLock()
    private var value: Value?

    func publish(_ value: Value) {
        lock.lock()
        let previous = self.value
        self.value = value
        lock.unlock()
        // Decoder-surface and texture destruction can be expensive. Release
        // the old scene after unlocking so it cannot delay a render read.
        withExtendedLifetime(previous) {}
    }

    func read() -> Value? {
        lock.lock()
        defer { lock.unlock() }
        return value
    }
}

enum PoseTiming {
    static func lookahead(now: Double, presentation: Double, period: Double,
                          scanout: Double, adjustment: Double) -> Float {
        // A missed deadline will present on a later refresh. Do not predict
        // backwards or keep the pose tied to the expired deadline.
        var target = presentation
        if target < now, period > 0 {
            target += ceil((now - target) / period) * period
        }
        return Float(max(0, min(0.1, target - now + scanout * 0.5 + adjustment)))
    }
}

// Predict the refresh each frame will actually reach the panel on.
// Frames are shown in submission order, at most one per refresh. When the
// compositor misses a refresh, the queue stays one frame deeper (+8 ms at
// 120 Hz) until a display tick is skipped. A smoothed delay is wrong for
// every frame around those switches, and during fast head turns each wrong
// refresh shows as a jump. Chain from the newest presented frame through the
// frames still in flight instead.
final class PresentationSchedule {
    struct Slot {
        let id: UInt64
        let time: Double
    }

    private struct Pending {
        let id: UInt64
        let target: Double
        let predicted: Double
    }

    private let lock = NSLock()
    private var nextID: UInt64 = 0
    private var pending: [Pending] = []
    private var lastPresented: Double?
    // Whole refreshes between the display link's target and presentation.
    // The smallest recent value is the compositor's fixed delay; deeper
    // queueing is already covered by the chain.
    private var leads: [(refreshes: Int, time: Double)] = []
    private var statFrames = 0
    private var statMispredicted = 0

    func schedule(target: Double, period: Double) -> Slot {
        lock.lock()
        defer { lock.unlock() }
        nextID += 1
        // Feedback can stop (display asleep, window hidden). Stale frames
        // would only lengthen the chain.
        pending.removeAll { target - $0.target > 0.5 }
        let lead = Double(leads.lazy.map(\.refreshes).min() ?? 0) * period
        var chained = lastPresented
        for frame in pending {
            chained = max(frame.target + lead, (chained ?? -.infinity) + period)
        }
        let predicted = max(target + lead, (chained ?? -.infinity) + period)
        pending.append(Pending(id: nextID, target: target, predicted: predicted))
        return Slot(id: nextID, time: predicted)
    }

    // The drawable was never presented: it does not occupy a refresh.
    func cancel(_ slot: Slot) {
        lock.lock()
        pending.removeAll { $0.id == slot.id }
        lock.unlock()
    }

    func complete(_ slot: Slot, presented: Double, period: Double) {
        lock.lock()
        defer { lock.unlock() }
        guard let index = pending.firstIndex(where: { $0.id == slot.id }) else { return }
        let frame = pending[index]
        guard presented > 0, period > 0 else {
            pending.remove(at: index)
            return
        }
        // Older frames without feedback were replaced before reaching the panel.
        pending.removeSubrange(...index)
        lastPresented = presented
        let refreshes = Int(((presented - frame.target) / period).rounded())
        leads.append((max(0, min(4, refreshes)), presented))
        leads.removeAll { presented - $0.time > 2 }
        statFrames += 1
        if abs(presented - frame.predicted) > period * 0.5 { statMispredicted += 1 }
    }

    // Presented frames, and how many reached the panel on a different refresh
    // than their head pose was predicted for. Resets the counters.
    func takeStats() -> (frames: Int, mispredicted: Int) {
        lock.lock()
        defer { lock.unlock() }
        let stats = (statFrames, statMispredicted)
        statFrames = 0
        statMispredicted = 0
        return stats
    }
}

// Core Video follows the specific headset display at its fixed 90/120 Hz.
// Drawing runs on its display thread, independently of AppKit's main loop.
final class HeadsetFrameDriver {
    private let link: CVDisplayLink
    private let layer: CAMetalLayer
    // Returns false when the drawable was not presented.
    private let render: (CAMetalDrawable, Double) -> Bool
    private let renderLock = NSLock()
    private let rateLock = NSLock()
    private var hz: Float
    private var stopped = false
    let schedule = PresentationSchedule()

    init?(layer: CAMetalLayer, displayID: CGDirectDisplayID, hz: Float,
          render: @escaping (CAMetalDrawable, Double) -> Bool) {
        var created: CVDisplayLink?
        guard CVDisplayLinkCreateWithCGDisplay(displayID, &created) == kCVReturnSuccess,
              let created else { return nil }
        link = created
        self.layer = layer
        self.hz = hz
        self.render = render
        CVDisplayLinkSetOutputHandler(link) { [weak self] _, _, output, _, _ in
            self?.draw(output.pointee)
            return kCVReturnSuccess
        }
    }

    func start() -> Bool {
        CVDisplayLinkStart(link) == kCVReturnSuccess
    }

    func setRate(hz: Float) {
        rateLock.lock()
        self.hz = hz
        rateLock.unlock()
    }

    private func draw(_ output: CVTimeStamp) {
        renderLock.lock()
        defer { renderLock.unlock() }
        guard !stopped else { return }
        autoreleasepool {
            // Wait for the drawable before sampling the head. Its availability
            // must not make the orientation stale while waiting for the GPU.
            guard let drawable = layer.nextDrawable() else { return }
            rateLock.lock()
            let period = 1 / Double(hz)
            rateLock.unlock()
            let hasHostTime = output.flags & CVTimeStampFlags.hostTimeValid.rawValue != 0
            let target = hasHostTime
                ? Double(output.hostTime) / CVGetHostClockFrequency()
                : CACurrentMediaTime() + period
            let schedule = self.schedule
            let slot = schedule.schedule(target: target, period: period)
            drawable.addPresentedHandler { frame in
                schedule.complete(slot, presented: frame.presentedTime, period: period)
            }
            if !render(drawable, slot.time) {
                schedule.cancel(slot)
            }
        }
    }

    func stop() {
        CVDisplayLinkStop(link)
        // Drain any callback already executing before USB/state teardown.
        renderLock.lock()
        stopped = true
        renderLock.unlock()
    }
}

// macOS gives a newly connected headset a scaled "looks like 3200x1632"
// desktop. WindowServer then composites a 6400x3264 framebuffer on every
// refresh, and the lens-corrected image is resampled twice on its way to the
// 4000x2040 panel (softer image, more GPU contention, more missed refreshes).
// Use a mode on the panel's own pixel grid while the player runs; macOS
// restores the previous mode when the player exits.
enum HeadsetDisplayMode {
    static func useNativePixels() {
        var count: UInt32 = 0
        var ids = [CGDirectDisplayID](repeating: 0, count: 16)
        guard CGGetOnlineDisplayList(UInt32(ids.count), &ids, &count) == .success else { return }
        let options = [kCGDisplayShowDuplicateLowResolutionModes: true] as CFDictionary
        for id in ids.prefix(Int(count))
        where CGDisplayVendorNumber(id) == 0x4DD9 && CGDisplayMirrorsDisplay(id) == 0 {
            guard let current = CGDisplayCopyDisplayMode(id),
                  current.pixelWidth != 4000 || current.pixelHeight != 2040 else { continue }
            let modes = CGDisplayCopyAllDisplayModes(id, options) as? [CGDisplayMode] ?? []
            let hiDPI = current.pixelWidth > current.width
            // Keep the refresh rate (90/120 Hz), then the HiDPI choice.
            guard let mode = modes
                .filter({ $0.pixelWidth == 4000 && $0.pixelHeight == 2040 && $0.isUsableForDesktopGUI() })
                .min(by: { a, b in
                    let da = abs(a.refreshRate - current.refreshRate)
                    let db = abs(b.refreshRate - current.refreshRate)
                    if da != db { return da < db }
                    return (a.pixelWidth > a.width) == hiDPI && (b.pixelWidth > b.width) != hiDPI
                }) else { continue }
            var config: CGDisplayConfigRef?
            guard CGBeginDisplayConfiguration(&config) == .success else { continue }
            CGConfigureDisplayWithDisplayMode(config, id, mode, nil)
            let result = CGCompleteDisplayConfiguration(config, .forAppOnly)
            if result == .success {
                print("[display] Headset desktop was \(current.width)x\(current.height) "
                    + "(\(current.pixelWidth)x\(current.pixelHeight) px, scaled); using "
                    + "\(mode.width)x\(mode.height) at \(Int(mode.refreshRate)) Hz (4000x2040 px) "
                    + "until exit. Pick 2000x1020 or 4000x2040 for PS VR2 in Displays settings to skip this switch")
            } else {
                CGCancelDisplayConfiguration(config)
                print("[display] !!! Could not switch the headset to 4000x2040 px: error \(result.rawValue)")
            }
        }
    }
}
