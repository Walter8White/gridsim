import Combine
import CryptoKit
import Foundation

@MainActor
final class CaptureNetworkService: ObservableObject {
    enum State: Equatable {
        case disconnected, connecting, connected, failed
    }

    @Published private(set) var state: State = .disconnected
    @Published private(set) var message = "Disconnected"
    var onCaptureCommand: ((CaptureCommand) -> Void)?

    private var socket: URLSessionWebSocketTask?
    private var receiveTask: Task<Void, Never>?

    func connect(to address: String) {
        disconnect()
        guard let url = Self.webSocketURL(from: address) else {
            state = .failed
            message = "Invalid Linux address"
            return
        }

        state = .connecting
        message = "Connecting to \(url.host ?? address)…"
        let socket = URLSession.shared.webSocketTask(with: url)
        self.socket = socket
        socket.resume()
        receiveTask = Task { [weak self] in
            await self?.receiveLoop(socket: socket)
        }
    }

    func disconnect() {
        receiveTask?.cancel()
        receiveTask = nil
        socket?.cancel(with: .normalClosure, reason: nil)
        socket = nil
        state = .disconnected
        message = "Disconnected"
    }

    func send(result: Result<CaptureRecord, Error>, captureID: String) {
        guard let socket else { return }
        Task {
            do {
                switch result {
                case .success(let record):
                    try await upload(record: record, using: socket)
                    try await sendResponse(
                        CaptureResultMessage(
                            type: "capture_result",
                            captureID: captureID,
                            success: true,
                            photoFilename: record.photoFilename,
                            captureCompletedAt: record.captureCompletedAt,
                            error: nil
                        ),
                        using: socket
                    )
                    message = "Uploaded \(record.photoFilename)"
                case .failure(let error):
                    try await sendResponse(
                        CaptureResultMessage(
                            type: "capture_result",
                            captureID: captureID,
                            success: false,
                            photoFilename: nil,
                            captureCompletedAt: nil,
                            error: error.localizedDescription
                        ),
                        using: socket
                    )
                }
            } catch {
                try? await sendResponse(
                    CaptureResultMessage(
                        type: "capture_result",
                        captureID: captureID,
                        success: false,
                        photoFilename: nil,
                        captureCompletedAt: nil,
                        error: "Upload failed: \(error.localizedDescription)"
                    ),
                    using: socket
                )
                state = .failed
                message = "Upload failed: \(error.localizedDescription)"
            }
        }
    }

    private func upload(record: CaptureRecord, using socket: URLSessionWebSocketTask) async throws {
        let documents = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let directory = documents.appendingPathComponent("Captures", isDirectory: true)
        let photo = try Data(contentsOf: directory.appendingPathComponent(record.photoFilename))
        let metadata = try Data(contentsOf: directory.appendingPathComponent(record.metadataFilename))
        let header = CaptureUploadStartMessage(
            type: "capture_upload_start",
            captureID: record.captureID,
            photoFilename: record.photoFilename,
            photoSize: photo.count,
            photoSHA256: SHA256.hash(data: photo).hexString,
            metadataFilename: record.metadataFilename,
            metadataSize: metadata.count,
            metadataSHA256: SHA256.hash(data: metadata).hexString
        )
        let headerData = try JSONEncoder().encode(header)
        guard let headerText = String(data: headerData, encoding: .utf8) else {
            throw UploadError.invalidHeader
        }
        try await socket.send(.string(headerText))
        try await socket.send(.data(photo))
        try await socket.send(.data(metadata))
    }

    private func sendResponse(
        _ response: CaptureResultMessage,
        using socket: URLSessionWebSocketTask
    ) async throws {
        let encoder = JSONEncoder()
        encoder.dateEncodingStrategy = .iso8601
        let data = try encoder.encode(response)
        guard let text = String(data: data, encoding: .utf8) else { throw UploadError.invalidHeader }
        try await socket.send(.string(text))
    }

    private func receiveLoop(socket: URLSessionWebSocketTask) async {
        do {
            while !Task.isCancelled {
                let message = try await socket.receive()
                let data: Data
                switch message {
                case .string(let text): data = Data(text.utf8)
                case .data(let value): data = value
                @unknown default: continue
                }
                try handle(data)
            }
        } catch where Task.isCancelled {
            return
        } catch {
            guard self.socket === socket else { return }
            state = .failed
            message = "Connection lost: \(error.localizedDescription)"
        }
    }

    private func handle(_ data: Data) throws {
        let wire = try JSONDecoder().decode(IncomingMessage.self, from: data)
        switch wire.type {
        case "hello":
            state = .connected
            message = "Connected to Linux"
        case "capture":
            guard let captureID = wire.captureID else { throw NetworkError.missingCaptureID }
            state = .connected
            message = String(
                format: "Remote capture at %.3f mm",
                wire.measuredPositionMM ?? 0
            )
            onCaptureCommand?(CaptureCommand(
                captureID: captureID,
                targetPositionMM: wire.targetPositionMM,
                measuredPositionMM: wire.measuredPositionMM,
                receivedAt: Date()
            ))
        default:
            break
        }
    }

    private static func webSocketURL(from input: String) -> URL? {
        var value = input.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !value.isEmpty else { return nil }
        if !value.hasPrefix("ws://") && !value.hasPrefix("wss://") { value = "ws://" + value }
        guard var components = URLComponents(string: value) else { return nil }
        if components.port == nil { components.port = 8765 }
        if components.path.isEmpty { components.path = "/" }
        return components.url
    }
}

private struct IncomingMessage: Decodable {
    let type: String
    let captureID: String?
    let targetPositionMM: Double?
    let measuredPositionMM: Double?

    enum CodingKeys: String, CodingKey {
        case type
        case captureID = "capture_id"
        case targetPositionMM = "target_position_mm"
        case measuredPositionMM = "measured_position_mm"
    }
}

private enum NetworkError: LocalizedError {
    case missingCaptureID
    var errorDescription: String? { "Capture command has no capture_id." }
}

private enum UploadError: LocalizedError {
    case invalidHeader
    var errorDescription: String? { "Could not encode the upload header." }
}

private extension SHA256.Digest {
    var hexString: String { map { String(format: "%02x", $0) }.joined() }
}
