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
    var passIndex: Int? = nil
    var passCaptureIndex: Int? = nil
    var xOffsetCM: Double? = nil
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
    var camera: CameraMetadata? = nil
    var pass_index: Int? = nil
    var x_offset_cm: Double? = nil
}

struct PhotoDimensions: Codable, Hashable, Identifiable, Sendable {
    let width: Int32
    let height: Int32
    var id: String { "\(width)x\(height)" }
    var requiresDeferredDelivery: Bool { width == 5712 && height == 4284 }
    var pixels: Int64 { Int64(width) * Int64(height) }
    var label: String { "\(Int((Double(pixels) / 1_000_000).rounded())) MP · \(id)" }
}

struct CameraMetadata: Codable, Sendable {
    var selected_camera: String
    var camera_device_type: String
    var selected_resolution: PhotoDimensions
    var maxPhotoDimensions: PhotoDimensions
    var codec: String = "jpeg"
    var lens_position: Float
    var focus_mode: String
    var exposure_mode: String
    var exposure_duration: Double
    var ISO: Float
    var aperture: Double?
    var focal_length: Double?
    var white_balance_mode: String
    var white_balance_gains: [Float]
    var white_balance_temperature: Float
    var white_balance_tint: Float
    var image_width: Int = 0
    var image_height: Int = 0
    var orientation: Int = 1
    var timestamp_unix: Double = 0
    var photo_timestamp_seconds: Double = 0
    var capture_time_seconds: Double = 0
}


protocol CaptureCommandReceiving: AnyObject {
    func start() async throws
    func stop()
}

struct IncomingMessage: Decodable {
    let type: String
    let captureID: String?
    let targetPositionMM: Double?
    let measuredPositionMM: Double?

    let passIndex: Int?
    let passCaptureIndex: Int?
    let xOffsetCM: Double?
    let xOffsetM: Double?

    enum CodingKeys: String, CodingKey {
        case type
        case passIndex = "pass_index"
        case passCaptureIndex = "pass_capture_index"
        case xOffsetCM = "x_offset_cm"
        case xOffsetM = "x_offset_m"
        case captureID = "capture_id"
        case targetPositionMM = "target_position_mm"
        case measuredPositionMM = "measured_position_mm"
    }
}
