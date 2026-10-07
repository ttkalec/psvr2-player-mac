import Foundation

struct PickerEntry {
    let url: URL
    let isDir: Bool
    let name: String
}

struct PickerFileList {
    var entries: [PickerEntry] = []
    private var shuffledFileOrder: [String: [String]] = [:]

    func clampedScroll(_ scroll: Int, rows: Int) -> Int {
        max(0, min(scroll, max(0, entries.count - rows)))
    }

    mutating func load(_ entries: [PickerEntry], in directory: URL) {
        guard let order = shuffledFileOrder[directory.path] else {
            self.entries = entries
            return
        }
        let ranks = Dictionary(uniqueKeysWithValues: order.enumerated().map { ($0.element, $0.offset) })
        let files = entries.filter { !$0.isDir }.sorted {
            let lhs = ranks[$0.url.path], rhs = ranks[$1.url.path]
            if lhs != rhs { return (lhs ?? Int.max) < (rhs ?? Int.max) }
            return $0.name.localizedCaseInsensitiveCompare($1.name) == .orderedAscending
        }
        self.entries = entries.filter { $0.isDir } + files
    }

    mutating func shuffle(in directory: URL) {
        var files = entries.filter { !$0.isDir }
        guard files.count > 1 else { return }
        let previousOrder = files.map { $0.url.path }
        files.shuffle()
        if files.map({ $0.url.path }) == previousOrder {
            files.append(files.removeFirst())
        }
        entries = entries.filter { $0.isDir } + files
        shuffledFileOrder[directory.path] = files.map { $0.url.path }
    }

    mutating func remove(_ url: URL) {
        entries.removeAll { !$0.isDir && $0.url.path == url.path }
        // Remove from every remembered alias of the containing folder too.
        for directory in Array(shuffledFileOrder.keys) {
            shuffledFileOrder[directory]?.removeAll { $0 == url.path }
        }
    }
}

enum PickerFileOperation {
    case delete, move

    enum Failure: LocalizedError {
        case notFile, missingFolder, alreadyInFolder, nameConflict

        var errorDescription: String? {
            switch self {
            case .notFile: return "The selected file no longer exists or is a folder."
            case .missingFolder: return "The !vr folder does not exist one level up."
            case .alreadyInFolder: return "This file is already in !vr."
            case .nameConflict: return "A file with this name already exists in !vr."
            }
        }
    }

    // Called off the main thread because mounted volumes may be slow.
    // The URL is captured when the user selects the action, never an array index.
    @discardableResult
    func perform(on source: URL, fileManager: FileManager = .default) throws -> URL? {
        var isDirectory: ObjCBool = false
        guard fileManager.fileExists(atPath: source.path, isDirectory: &isDirectory),
              !isDirectory.boolValue else { throw Failure.notFile }
        switch self {
        case .delete:
            try fileManager.removeItem(at: source)
            return nil
        case .move:
            let folder = source.deletingLastPathComponent().deletingLastPathComponent()
                .appendingPathComponent("!vr", isDirectory: true)
            guard fileManager.fileExists(atPath: folder.path, isDirectory: &isDirectory),
                  isDirectory.boolValue else { throw Failure.missingFolder }
            let destination = folder.appendingPathComponent(source.lastPathComponent)
            guard destination.resolvingSymlinksInPath().standardizedFileURL
                    != source.resolvingSymlinksInPath().standardizedFileURL else {
                throw Failure.alreadyInFolder
            }
            guard !fileManager.fileExists(atPath: destination.path) else { throw Failure.nameConflict }
            // moveItem also refuses to overwrite if another file appears after the check.
            try fileManager.moveItem(at: source, to: destination)
            return destination
        }
    }
}
