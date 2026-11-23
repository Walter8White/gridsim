import AVFoundation
import Combine

/// Hardware discovery only. No depth output, AR session, or metadata writer is started.
@MainActor
final class LiDARService: ObservableObject {
    @Published private(set) var hasChecked = false
    @Published private(set) var deviceName: String?

    var isDetected: Bool { deviceName != nil }
    var statusLabel: String {
        guard hasChecked else { return "LiDAR…" }
        return isDetected ? "LiDAR detected" : "No LiDAR"
    }

    func refreshAvailability() {
        let discovery = AVCaptureDevice.DiscoverySession(
            deviceTypes: [.builtInLiDARDepthCamera], mediaType: .video, position: .back
        )
        deviceName = discovery.devices.first?.localizedName
        hasChecked = true
    }
}
