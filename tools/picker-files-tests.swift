import Foundation

@main
struct PickerFileTests {
    static func check(_ condition: @autoclosure () -> Bool, _ message: String) {
        guard condition() else { fatalError(message) }
    }

    static func expectFailure(_ failure: PickerFileOperation.Failure, _ body: () throws -> Void) {
        do {
            try body()
            fatalError("Expected \(failure)")
        } catch let error as PickerFileOperation.Failure {
            check(error == failure, "Expected \(failure), got \(error)")
        } catch {
            fatalError("Unexpected error: \(error)")
        }
    }

    static func main() throws {
        setbuf(stdout, nil)
        let fm = FileManager.default
        let root = fm.temporaryDirectory.resolvingSymlinksInPath().appendingPathComponent(UUID().uuidString, isDirectory: true)
        let sourceFolder = root.appendingPathComponent("vr_new", isDirectory: true)
        let destinationFolder = root.appendingPathComponent("!vr", isDirectory: true)
        try fm.createDirectory(at: sourceFolder, withIntermediateDirectories: true)
        defer { try? fm.removeItem(at: root) }

        func makeFile(_ name: String, in folder: URL = sourceFolder) throws -> URL {
            let url = folder.appendingPathComponent(name)
            try Data(name.utf8).write(to: url)
            return url
        }
        func listing() throws -> [PickerEntry] {
            let files = try fm.contentsOfDirectory(at: sourceFolder, includingPropertiesForKeys: nil)
                .sorted { $0.path < $1.path }
            return [PickerEntry(url: root, isDir: true, name: "..")] + files.map {
                PickerEntry(url: $0, isDir: false, name: $0.lastPathComponent)
            }
        }

        for i in 0..<12 { _ = try makeFile("video \(i).mp4") }
        let original = try listing()
        var list = PickerFileList()
        list.load(original, in: sourceFolder)
        list.shuffle(in: sourceFolder)
        check(list.entries[0].isDir, "Shuffle must keep folders at the top")
        check(list.entries.map(\.url) != original.map(\.url), "Shuffle must change the order")

        // Pick a row whose shuffled position differs from its original position.
        let row = list.entries.indices.first { list.entries[$0].url != original[$0].url }!
        let selected = list.entries[row].url
        let originalRowFile = original[row].url
        let survivors = list.entries.map(\.url).filter { $0 != selected }
        try PickerFileOperation.delete.perform(on: selected)
        list.remove(selected)
        check(!fm.fileExists(atPath: selected.path), "Delete must remove the selected file from disk")
        check(fm.fileExists(atPath: originalRowFile.path), "Delete must not target the unshuffled row")
        check(list.entries.map(\.url) == survivors, "Delete must preserve the order of the remaining rows")
        list.load(try listing(), in: sourceFolder)
        check(list.entries.map(\.url) == survivors, "Returning to the folder must preserve the shuffle")
        print("PASS: delete after shuffle targets the selected file and preserves remaining order")

        let moveTarget = list.entries.last!.url
        let beforeFailure = list.entries.map(\.url)
        expectFailure(.missingFolder) { try PickerFileOperation.move.perform(on: moveTarget) }
        check(fm.fileExists(atPath: moveTarget.path), "Missing !vr must leave the source intact")
        check(list.entries.map(\.url) == beforeFailure, "A failed move must leave the list intact")
        _ = try makeFile("!vr", in: root)
        expectFailure(.missingFolder) { try PickerFileOperation.move.perform(on: moveTarget) }
        try fm.removeItem(at: destinationFolder)
        try fm.createDirectory(at: destinationFolder, withIntermediateDirectories: false)
        print("PASS: missing !vr or a regular file named !vr cannot move a source")

        let conflict = try makeFile(moveTarget.lastPathComponent, in: destinationFolder)
        let conflictData = try Data(contentsOf: conflict)
        expectFailure(.nameConflict) { try PickerFileOperation.move.perform(on: moveTarget) }
        let afterConflictData = try Data(contentsOf: conflict)
        check(afterConflictData == conflictData, "Move must not overwrite existing content")
        check(fm.fileExists(atPath: moveTarget.path), "A name conflict must keep the source")
        try fm.removeItem(at: conflict)
        print("PASS: destination conflicts never overwrite or remove files")

        let sourceData = try Data(contentsOf: moveTarget)
        let moved = try PickerFileOperation.move.perform(on: moveTarget)!
        list.remove(moveTarget)
        check(moved.standardizedFileURL.path == destinationFolder.appendingPathComponent(moveTarget.lastPathComponent).standardizedFileURL.path,
              "Move must use the sibling !vr folder")
        let movedData = try Data(contentsOf: moved)
        check(movedData == sourceData, "Move must preserve the file contents")
        check(!fm.fileExists(atPath: moveTarget.path), "Move must remove the original path")
        let afterMove = list.entries.map(\.url)
        check(afterMove == beforeFailure.filter { $0 != moveTarget }, "Move must preserve shuffled order")
        list.load(try listing(), in: sourceFolder)
        check(list.entries.map(\.url) == afterMove, "Reload must not restore the moved file")
        expectFailure(.alreadyInFolder) { try PickerFileOperation.move.perform(on: moved) }
        print("PASS: move after shuffle transfers the exact file to ../!vr and survives reload")

        expectFailure(.notFile) { try PickerFileOperation.delete.perform(on: sourceFolder) }
        expectFailure(.notFile) { try PickerFileOperation.move.perform(on: sourceFolder) }
        expectFailure(.notFile) { try PickerFileOperation.delete.perform(on: selected) }
        print("PASS: folder actions and missing sources are rejected")

        var scroll = list.clampedScroll(Int.max, rows: 6)
        for entry in list.entries.filter({ !$0.isDir }).reversed() {
            try PickerFileOperation.delete.perform(on: entry.url)
            list.remove(entry.url)
            scroll = list.clampedScroll(scroll, rows: 6)
            check(scroll >= 0 && scroll <= max(0, list.entries.count - 6), "Scroll must remain on a valid page")
        }
        check(scroll == 0 && list.entries.count == 1, "Deleting the last file must leave the parent row visible")
        list.remove(root)
        check(list.entries.count == 1, "Removing a file must never remove a folder row")
        print("PASS: repeated deletions clamp the last page and preserve folder rows")

        // A captured action remains tied to its URL even if another shuffle happens.
        _ = try makeFile("a.mp4")
        _ = try makeFile("b.mp4")
        list.load(try listing(), in: sourceFolder)
        list.shuffle(in: sourceFolder)
        let captured = list.entries[1].url
        let other = list.entries[2].url
        list.shuffle(in: sourceFolder)
        try PickerFileOperation.delete.perform(on: captured)
        list.remove(captured)
        check(list.entries.filter { !$0.isDir }.map { $0.url.path } == [other.path],
              "A pending action must retain its selected URL through another shuffle")
        print("PASS: captured file actions remain valid through a later shuffle")
    }
}
