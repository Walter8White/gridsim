import SwiftUI
import AVFoundation

private enum DeployaStyle {
    static let red = Color(red: 219/255, green: 0, blue: 13/255)
    static let ink = Color(red: 0.13, green: 0.13, blue: 0.14)
    static let paper = Color.white
}

/// GridCapture is a passive camera and motion sensor. All capture controls live on Linux.
struct ContentView: View {
    @StateObject private var camera = CameraService()
    @StateObject private var motion = MotionService()
    @StateObject private var lidar = LiDARService()
    @StateObject private var network = CaptureNetworkService()

    var body: some View {
        GeometryReader { geometry in
            VStack(spacing: 0) {
                header
                cameraPanel
                    .frame(maxWidth: .infinity, maxHeight: .infinity)
                    .padding(.horizontal, geometry.size.width > geometry.size.height ? 24 : 14)
                    .padding(.vertical, 16)
                footer
            }
            .frame(maxWidth: .infinity, maxHeight: .infinity)
            .background(DeployaStyle.paper.ignoresSafeArea())
        }
        .task {
            camera.onPreviewFrame = { data in network.sendPreview(data) }
            network.onConnected = { network.sendCameraState(camera.remoteSettingsSnapshot) }
            network.onCameraSettings = { settings in
                Task {
                    await camera.applyRemoteSettings(settings)
                    network.sendCameraState(camera.remoteSettingsSnapshot)
                }
            }
            network.onCaptureCommand = { command in
                camera.capture(command: command, motion: motion.sample) { result in
                    network.send(result: result, captureID: command.captureID)
                }
            }
            network.startDiscovery()
            await camera.start()
            network.sendCameraState(camera.remoteSettingsSnapshot)
            lidar.refreshAvailability()
            motion.start()
        }
        .onDisappear {
            camera.stop()
            motion.stop()
            network.disconnect()
        }
        .preferredColorScheme(.light)
    }

    private var header: some View {
        HStack(alignment: .center, spacing: 14) {
            Image("DeployaWordmark")
                .resizable()
                .scaledToFit()
                .frame(width: 220, height: 34)
                .accessibilityLabel("déploya:robotics")
            Spacer(minLength: 8)
            HStack(spacing: 7) {
                Circle().fill(connectionColor).frame(width: 9, height: 9)
                Text(connectionLabel.uppercased())
                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                    .tracking(0.3)
                    .lineLimit(1)
            }
            .foregroundStyle(DeployaStyle.ink)
            .padding(.horizontal, 12).padding(.vertical, 9)
            .background(Color(red: 0.96, green: 0.96, blue: 0.96), in: Capsule())
            .accessibilityLabel("Linux connection: \(connectionLabel)")
        }
        .padding(.horizontal, 22)
        .padding(.vertical, 15)
        .overlay(alignment: .bottom) { Rectangle().fill(DeployaStyle.red.opacity(0.18)).frame(height: 1) }
    }

    private var cameraPanel: some View {
        ZStack(alignment: .bottomLeading) {
            CameraPreview(session: camera.session, focusEnabled: false, onFocus: { _ in })
                .background(Color(red: 0.97, green: 0.97, blue: 0.96))
                .clipShape(RoundedRectangle(cornerRadius: 18, style: .continuous))
            if camera.state != .ready {
                VStack(spacing: 12) {
                    Image(systemName: camera.state == .denied ? "camera.slash" : "camera")
                        .font(.system(size: 32, weight: .medium))
                        .foregroundStyle(DeployaStyle.red)
                    Text((camera.message ?? "INITIALIZING CAMERA").uppercased())
                        .font(.system(size: 12, weight: .bold, design: .monospaced))
                        .foregroundStyle(DeployaStyle.ink)
                        .multilineTextAlignment(.center)
                }
                .padding(22)
                .frame(maxWidth: .infinity, maxHeight: .infinity)
                .background(.white.opacity(0.9))
            }
            if let resolution = camera.live?.selected_resolution.label {
                Text("PHOTO · \(resolution.uppercased())")
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
                    .foregroundStyle(DeployaStyle.ink)
                    .padding(.horizontal, 11).padding(.vertical, 8)
                    .background(.white.opacity(0.92), in: Capsule())
                    .frame(maxWidth: .infinity, maxHeight: .infinity, alignment: .topTrailing)
                    .padding(12)
            }
            HStack(spacing: 8) {
                Circle().fill(camera.state == .ready ? DeployaStyle.red : .gray).frame(width: 7, height: 7)
                Text(camera.state == .ready ? "LIVE SENSOR FEED" : "CAMERA \(camera.state == .denied ? "PERMISSION REQUIRED" : "STARTING")")
                Spacer()
                Text(lidar.isDetected ? "LIDAR · ACTIVE" : "MOTION · \(motion.isActive ? "ACTIVE" : "STARTING")")
            }
            .font(.system(size: 10, weight: .bold, design: .monospaced))
            .tracking(0.2)
            .foregroundStyle(.white)
            .padding(.horizontal, 14).padding(.vertical, 11)
            .background(DeployaStyle.ink.opacity(0.78), in: RoundedRectangle(cornerRadius: 10))
            .padding(12)
        }
        .overlay(RoundedRectangle(cornerRadius: 18).stroke(DeployaStyle.ink.opacity(0.12), lineWidth: 1))
        .accessibilityLabel("Passive iPhone camera feed")
    }

    private var footer: some View {
        HStack {
            Text("CAMERA + MOTION SENSOR")
            Spacer()
            Text("CONTROLLED FROM LINUX STATION")
        }
        .font(.system(size: 10, weight: .bold, design: .monospaced))
        .tracking(0.35)
        .foregroundStyle(DeployaStyle.ink.opacity(0.62))
        .padding(.horizontal, 22).padding(.vertical, 13)
        .overlay(alignment: .top) { Rectangle().fill(DeployaStyle.ink.opacity(0.1)).frame(height: 1) }
    }

    private var connectionLabel: String {
        switch network.state {
        case .connected: "CONNECTED TO CONTROL STATION"
        case .connecting: "CONNECTING"
        case .failed: "SEARCHING FOR STATION"
        case .disconnected: "SEARCHING FOR STATION"
        }
    }

    private var connectionColor: Color {
        network.state == .connected ? DeployaStyle.red : .orange
    }
}
