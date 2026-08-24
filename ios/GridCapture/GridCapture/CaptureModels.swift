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
}

struct CaptureRecord: Codable, Sendable {
    let captureID: String
    let targetPositionMM: Double?
    let measuredPositionMM: Double?
    let photoFilename: String
    let commandReceivedAt: Date?
    let captureCompletedAt: Date
    let motion: MotionSample
}

protocol CaptureCommandReceiving: AnyObject {
    func start() async throws
    func stop()
}
