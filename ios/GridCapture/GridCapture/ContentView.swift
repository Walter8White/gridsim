import SwiftUI

struct ContentView: View {
    @StateObject private var camera = CameraService()
    @StateObject private var motion = MotionService()
    @StateObject private var network = CaptureNetworkService()
    @AppStorage("linuxAddress") private var linuxAddress = ""

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            CameraPreview(session: camera.session).ignoresSafeArea()
            LinearGradient(colors: [.black.opacity(0.65), .clear, .black.opacity(0.8)], startPoint: .top, endPoint: .bottom)
                .ignoresSafeArea().allowsHitTesting(false)
            VStack(spacing: 16) {
                header
                connectionPanel
                Spacer()
                telemetry
                captureButton
            }
            .padding()
        }
        .task {
            network.onCaptureCommand = { command in
                camera.capture(command: command, motion: motion.sample) { result in
                    network.send(result: result, captureID: command.captureID)
                }
            }
            await camera.start()
            motion.start()
        }
        .onDisappear {
            camera.stop()
            motion.stop()
            network.disconnect()
        }
    }

    private var header: some View {
        HStack {
            Label("GridCapture", systemImage: "camera.viewfinder").font(.headline)
            Spacer()
            StatusPill(label: "Camera", active: camera.state == .ready)
            StatusPill(label: "IMU", active: motion.isActive)
        }
        .foregroundStyle(.white)
    }

    private var connectionPanel: some View {
        HStack(spacing: 10) {
            TextField("Linux IP (e.g. 192.168.1.42)", text: $linuxAddress)
                .textInputAutocapitalization(.never)
                .autocorrectionDisabled()
                .keyboardType(.URL)
                .padding(.horizontal, 12)
                .frame(height: 42)
                .background(.thinMaterial, in: RoundedRectangle(cornerRadius: 12))

            Button(network.state == .connected ? "Stop" : "Connect") {
                if network.state == .connected || network.state == .connecting {
                    network.disconnect()
                } else {
                    network.connect(to: linuxAddress)
                }
            }
            .buttonStyle(.borderedProminent)
            .tint(network.state == .connected ? .red : .blue)
        }
        .foregroundStyle(.white)
    }

    private var telemetry: some View {
        VStack(spacing: 10) {
            HStack {
                AngleValue(label: "Roll", value: motion.sample.rollDegrees)
                AngleValue(label: "Pitch", value: motion.sample.pitchDegrees)
                AngleValue(label: "Yaw", value: motion.sample.yawDegrees)
            }
            if let message = camera.message {
                Text(message).font(.footnote)
                    .foregroundStyle(camera.state == .failed ? .red : .white)
                    .multilineTextAlignment(.center)
            }
            Text(network.message)
                .font(.footnote)
                .foregroundStyle(network.state == .failed ? .red : .secondary)
        }
        .padding()
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 18))
    }

    private var captureButton: some View {
        Button {
            camera.capture(motion: motion.sample)
        } label: {
            ZStack {
                Circle().fill(.white).frame(width: 76, height: 76)
                Circle().stroke(.black.opacity(0.8), lineWidth: 3).frame(width: 64, height: 64)
                if camera.state == .capturing { ProgressView().tint(.black) }
            }
        }
        .disabled(camera.state != .ready)
        .accessibilityLabel("Take Photo")
    }
}

private struct StatusPill: View {
    let label: String
    let active: Bool

    var body: some View {
        HStack(spacing: 5) {
            Circle().fill(active ? .green : .orange).frame(width: 8, height: 8)
            Text(label).font(.caption.weight(.medium))
        }
        .padding(.horizontal, 9).padding(.vertical, 6)
        .background(.black.opacity(0.45), in: Capsule())
    }
}

private struct AngleValue: View {
    let label: String
    let value: Double

    var body: some View {
        VStack(spacing: 3) {
            Text(label).font(.caption).foregroundStyle(.secondary)
            Text(value.formatted(.number.precision(.fractionLength(1))) + "°")
                .font(.system(.body, design: .monospaced, weight: .semibold))
        }
        .frame(maxWidth: .infinity)
    }
}
