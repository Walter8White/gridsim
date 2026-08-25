@preconcurrency import AVFoundation
import Combine
import Foundation

@MainActor
final class CameraService: NSObject, ObservableObject {
    enum State: Equatable {
        case idle, configuring, ready, capturing, denied, failed
    }

    let session = AVCaptureSession()
    @Published private(set) var state: State = .idle
    @Published private(set) var message: String?
    @Published private(set) var lastCapture: CaptureRecord?

    private let output = AVCapturePhotoOutput()
    private let sessionQueue = DispatchQueue(label: "com.deploya.gridcapture.camera")
    private var delegates: [Int64: PhotoDelegate] = [:]
    private var configured = false

    func start() async {
        state = .configuring
        let authorized: Bool
        switch AVCaptureDevice.authorizationStatus(for: .video) {
        case .authorized:
            authorized = true
        case .notDetermined:
            authorized = await AVCaptureDevice.requestAccess(for: .video)
        default:
            authorized = false
        }

        guard authorized else {
            state = .denied
            message = "Camera access is required. Enable it in Settings."
            return
        }

        do {
            try configureIfNeeded()
            sessionQueue.async { [session] in
                if !session.isRunning { session.startRunning() }
            }
            state = .ready
            message = "Ready"
        } catch {
            state = .failed
            message = error.localizedDescription
        }
    }

    func stop() {
        sessionQueue.async { [session] in
            if session.isRunning { session.stopRunning() }
        }
    }

    func capture(
        command: CaptureCommand = CaptureCommand(
            captureID: UUID().uuidString,
            targetPositionMM: nil,
            measuredPositionMM: nil
        ),
        motion: MotionSample,
        completion: ((Result<CaptureRecord, Error>) -> Void)? = nil
    ) {
        guard state == .ready else {
            completion?(.failure(CameraError.cameraNotReady))
            return
        }
        state = .capturing
        message = "Capturing…"

        let settings = AVCapturePhotoSettings(format: [AVVideoCodecKey: AVVideoCodecType.jpeg])
        let settingsID = settings.uniqueID
        let delegate = PhotoDelegate { [weak self] result in
            Task { @MainActor in
                guard let self else { return }
                self.delegates.removeValue(forKey: settingsID)
                switch result {
                case .success(let data):
                    do {
                        self.lastCapture = try self.save(data: data, command: command, motion: motion)
                        self.state = .ready
                        self.message = "Saved \(self.lastCapture?.photoFilename ?? "photo")"
                        if let record = self.lastCapture { completion?(.success(record)) }
                    } catch {
                        self.state = .failed
                        self.message = error.localizedDescription
                        completion?(.failure(error))
                    }
                case .failure(let error):
                    self.state = .failed
                    self.message = error.localizedDescription
                    completion?(.failure(error))
                }
            }
        }
        delegates[settingsID] = delegate
        output.capturePhoto(with: settings, delegate: delegate)
    }

    private func configureIfNeeded() throws {
        guard !configured else { return }
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .photo

        guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
            throw CameraError.noRearCamera
        }
        let input = try AVCaptureDeviceInput(device: device)
        guard session.canAddInput(input), session.canAddOutput(output) else {
            throw CameraError.configurationFailed
        }
        session.addInput(input)
        session.addOutput(output)
        configured = true
    }

    private func save(data: Data, command: CaptureCommand, motion: MotionSample) throws -> CaptureRecord {
        let directory = try captureDirectory()
        let safeID = command.captureID.replacingOccurrences(of: "/", with: "-")
        let photoFilename = "\(safeID).jpg"
        let metadataFilename = "\(safeID).json"
        try data.write(to: directory.appendingPathComponent(photoFilename), options: .atomic)

        let record = CaptureRecord(
            captureID: command.captureID,
            targetPositionMM: command.targetPositionMM,
            measuredPositionMM: command.measuredPositionMM,
            photoFilename: photoFilename,
            metadataFilename: metadataFilename,
            commandReceivedAt: command.receivedAt,
            captureCompletedAt: Date(),
            motion: motion
        )
        let encoder = JSONEncoder()
        encoder.outputFormatting = [.prettyPrinted, .sortedKeys]
        encoder.dateEncodingStrategy = .iso8601
        try encoder.encode(record).write(
            to: directory.appendingPathComponent(metadataFilename),
            options: .atomic
        )
        return record
    }

    private func captureDirectory() throws -> URL {
        let documents = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let directory = documents.appendingPathComponent("Captures", isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        return directory
    }
}

private final class PhotoDelegate: NSObject, AVCapturePhotoCaptureDelegate {
    private let completion: (Result<Data, Error>) -> Void

    init(completion: @escaping (Result<Data, Error>) -> Void) {
        self.completion = completion
    }

    func photoOutput(
        _ output: AVCapturePhotoOutput,
        didFinishProcessingPhoto photo: AVCapturePhoto,
        error: Error?
    ) {
        if let error {
            completion(.failure(error))
        } else if let data = photo.fileDataRepresentation() {
            completion(.success(data))
        } else {
            completion(.failure(CameraError.noPhotoData))
        }
    }
}

private enum CameraError: LocalizedError {
    case noRearCamera, configurationFailed, noPhotoData, cameraNotReady

    var errorDescription: String? {
        switch self {
        case .noRearCamera: "No rear camera is available."
        case .configurationFailed: "The camera session could not be configured."
        case .noPhotoData: "The camera returned no photo data."
        case .cameraNotReady: "The camera is not ready to capture."
        }
    }
}
