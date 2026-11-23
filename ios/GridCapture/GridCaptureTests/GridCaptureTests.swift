import Foundation
import Testing
@testable import GridCapture

struct GridCaptureTests {
    @Test func legacyCaptureCommandsRemainCompatible() throws {
        let message = try JSONDecoder().decode(IncomingMessage.self, from: Data(#"{"type":"capture","capture_id":"legacy","target_position_mm":20,"measured_position_mm":19.99}"#.utf8))
        #expect(message.captureID == "legacy")
        #expect(message.measuredPositionMM == 19.99)
        #expect(message.passIndex == nil)
        #expect(message.xOffsetCM == nil)
    }

    @Test func multiPassWireMetadataSurvivesDecoding() throws {
        let message = try JSONDecoder().decode(IncomingMessage.self, from: Data(#"{"type":"capture","capture_id":"pass-2","pass_index":2,"pass_capture_index":3,"x_offset_cm":-15.5,"x_offset_m":-0.155}"#.utf8))
        #expect(message.passIndex == 2)
        #expect(message.passCaptureIndex == 3)
        #expect(message.xOffsetCM == -15.5)
        #expect(message.xOffsetM == -0.155)
    }

    @Test func sidecarPreservesImageAndCameraDetails() throws {
        let resolution = PhotoDimensions(width: 8064, height: 6048)
        var camera = CameraMetadata(selected_camera: "Wide", camera_device_type: "builtInWideAngleCamera", selected_resolution: resolution, maxPhotoDimensions: resolution, lens_position: 0.4, focus_mode: "locked", exposure_mode: "locked", exposure_duration: 0.01, ISO: 100, aperture: 1.8, focal_length: 6.8, white_balance_mode: "locked", white_balance_gains: [2, 1, 1.5], white_balance_temperature: 5000, white_balance_tint: 0)
        camera.image_width = 8064; camera.image_height = 6048
        camera.timestamp_unix = 1791440000.123456
        let record = CaptureRecord(captureID: "test", targetPositionMM: 20, measuredPositionMM: 19.99, photoFilename: "test.jpg", metadataFilename: "test.json", commandReceivedAt: nil, captureCompletedAt: Date(), motion: MotionSample(), camera: camera, pass_index: 2, x_offset_cm: -15.5)
        let encoded = try JSONEncoder().encode(record)
        let json = try #require(JSONSerialization.jsonObject(with: encoded) as? [String: Any])
        let details = try #require(json["camera"] as? [String: Any])
        #expect(details["image_width"] as? Int == 8064)
        #expect(details["codec"] as? String == "jpeg")
        #expect(details["timestamp_unix"] as? Double == camera.timestamp_unix)
        #expect(json["pass_index"] as? Int == 2)
        #expect(json["x_offset_cm"] as? Double == -15.5)
        let decoded = try JSONDecoder().decode(CaptureRecord.self, from: encoded)
        #expect(decoded.camera?.selected_resolution == resolution)
        #expect(decoded.measuredPositionMM == 19.99)
    }

    @Test func onlyApple24MPSizeRequiresDeferredDelivery() {
        #expect(PhotoDimensions(width: 5712, height: 4284).requiresDeferredDelivery)
        #expect(!PhotoDimensions(width: 4032, height: 3024).requiresDeferredDelivery)
        #expect(!PhotoDimensions(width: 8064, height: 6048).requiresDeferredDelivery)
    }
}
