import AVFoundation

// Main-thread coordinator. Overlapping AVPlayer seeks can repeatedly flush
// the decoder. Finish the current seek, then chase only the latest target.
// This never changes rate, so user pause, proximity pause, and speed survive.
final class VideoSeeker {
    private struct Request {
        let time: CMTime
        let before: CMTime
        let after: CMTime
        let completion: (Bool) -> Void
    }
    private weak var player: AVPlayer?
    private let onSeek: () -> Void
    private var current: Request?
    private var pending: Request?
    private var stopped = false

    var target: CMTime? { pending?.time ?? current?.time }

    init(player: AVPlayer, onSeek: @escaping () -> Void = {}) {
        self.player = player
        self.onSeek = onSeek
    }

    func seek(to time: CMTime, toleranceBefore: CMTime, toleranceAfter: CMTime,
              completion: @escaping (Bool) -> Void = { _ in }) {
        guard !stopped else { completion(false); return }
        let replaced = pending
        pending = Request(time: time, before: toleranceBefore, after: toleranceAfter,
                          completion: completion)
        replaced?.completion(false)
        startNext()
    }

    func stop() {
        stopped = true
        let replaced = pending
        pending = nil
        replaced?.completion(false)
        player?.currentItem?.cancelPendingSeeks()
    }

    private func startNext() {
        guard !stopped, current == nil, let request = pending, let player else { return }
        pending = nil
        current = request
        onSeek()
        player.seek(to: request.time, toleranceBefore: request.before, toleranceAfter: request.after) {
            [weak self] finished in
            DispatchQueue.main.async {
                guard let self else { return }
                self.current = nil
                request.completion(finished && !self.stopped && self.pending == nil)
                self.startNext()
            }
        }
    }
}
