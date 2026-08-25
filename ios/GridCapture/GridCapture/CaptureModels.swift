import Foundation

struct MotionSample: Codable, Sendable {
    var timestamp: TimeInterval = 0
    var rollRadians: Double = 0
    var pitchRadians: Double = 0
    var yawRadians: Double = 0
    var rotationRateX: Double = 0
    var rotationRateY: Double = 0
    var rotationRateZ: Double = 0
    var accelerationX: Double = 0
    var accelerationY: Double = 0
    var accelerationZ: Double = 0

    var rollDegrees: Double { rollRadians * 180 / .pi }
    var pitchDegrees: Double { pitchRadians * 180 / .pi }
    var yawDegrees: Double { yawRadians * 180 / .pi }
}

struct CaptureCommand: Codable, Sendable {
    let captureID: String
    let targetPositionMM: Double?
    let measuredPositionMM: Double?
    var receivedAt: Date? = nil
}

struct CaptureResultMessage: Codable, Sendable {
    let type: String
    let captureID: String
    let success: Bool
    let photoFilename: String?
    let captureCompletedAt: Date?
    let error: String?

    enum CodingKeys: String, CodingKey {
        case type, success, error
        case captureID = "capture_id"
        case photoFilename = "photo_filename"
        case captureCompletedAt = "capture_completed_at"
    }
}

struct CaptureUploadStartMessage: Codable, Sendable {
    let type: String
    let captureID: String
    let photoFilename: String
    let photoSize: Int
    let photoSHA256: String
    let metadataFilename: String
    let metadataSize: Int
    let metadataSHA256: String

    enum CodingKeys: String, CodingKey {
        case type
        case captureID = "capture_id"
        case photoFilename = "photo_filename"
        case photoSize = "photo_size"
        case photoSHA256 = "photo_sha256"
        case metadataFilename = "metadata_filename"
        case metadataSize = "metadata_size"
        case metadataSHA256 = "metadata_sha256"
    }
}

struct CaptureRecord: Codable, Sendable {
    let captureID: String
    let targetPositionMM: Double?
    let measuredPositionMM: Double?
    let photoFilename: String
    let metadataFilename: String
    let commandReceivedAt: Date?
    let captureCompletedAt: Date
    let motion: MotionSample
}

protocol CaptureCommandReceiving: AnyObject {
    func start() async throws
    func stop()
}
