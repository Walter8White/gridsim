import Combine
import CoreMotion
import Foundation

@MainActor
final class MotionService: ObservableObject {
    @Published private(set) var sample = MotionSample()
    @Published private(set) var isActive = false
    private let manager = CMMotionManager()

    func start() {
        guard manager.isDeviceMotionAvailable else { return }
        manager.deviceMotionUpdateInterval = 1.0 / 50.0
        manager.startDeviceMotionUpdates(using: .xArbitraryZVertical, to: .main) { [weak self] data, _ in
            guard let self, let data else { return }
            self.sample = MotionSample(
                timestamp: data.timestamp,
                rollRadians: data.attitude.roll,
                pitchRadians: data.attitude.pitch,
                yawRadians: data.attitude.yaw,
                rotationRateX: data.rotationRate.x,
                rotationRateY: data.rotationRate.y,
                rotationRateZ: data.rotationRate.z,
                accelerationX: data.userAcceleration.x,
                accelerationY: data.userAcceleration.y,
                accelerationZ: data.userAcceleration.z
            )
            self.isActive = true
        }
    }

    func stop() {
        manager.stopDeviceMotionUpdates()
        isActive = false
    }
}
