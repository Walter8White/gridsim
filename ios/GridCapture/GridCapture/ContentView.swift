import SwiftUI
import AVFoundation

struct ContentView: View {
    @StateObject private var camera = CameraService()
    @StateObject private var motion = MotionService()
    @StateObject private var lidar = LiDARService()
    @StateObject private var network = CaptureNetworkService()
    @State private var showSettings = false
    @AppStorage("linuxAddress") private var linuxAddress = ""

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            if !showSettings {
                CameraPreview(session: camera.session, focusEnabled: camera.canTapToFocus, onFocus: camera.tapFocus).ignoresSafeArea()
            }
            LinearGradient(colors: [.black.opacity(0.65), .clear, .black.opacity(0.8)], startPoint: .top, endPoint: .bottom)
                .ignoresSafeArea().allowsHitTesting(false)
            VStack(spacing: 16) {
                header
                cameraBadges
                connectionPanel
                Spacer()
                captureStatus
                captureButton
            }
            .padding()
        }
        .sheet(isPresented: $showSettings) { cameraSettings }
        .task {
            network.onCaptureCommand = { command in
                camera.capture(command: command, motion: motion.sample) { result in
                    network.send(result: result, captureID: command.captureID)
                }
            }
            await camera.start()
            lidar.refreshAvailability()
            motion.start()
        }
        .onDisappear {
            camera.stop()
            motion.stop()
            network.disconnect()
        }
    }

    private var focusSwitch: some View {
        Toggle(isOn: Binding(get: { camera.isFocusLocked }, set: camera.setFocusLocked)) {
            Label(camera.isFocusLocked ? "Lock focus" : "Autofocus",
                  systemImage: camera.isFocusLocked ? "lock.fill" : "viewfinder")
                .font(.caption.weight(.semibold))
                .foregroundStyle(camera.isFocusLocked ? .red : .green)
        }
        .toggleStyle(.switch)
        .tint(.red)
        .fixedSize()
        .disabled(!camera.canToggleFocus)
        .accessibilityIdentifier("focusLockSwitch")
        .accessibilityLabel("Focus lock")
        .accessibilityValue(camera.isFocusLocked ? "Locked" : "Autofocus")
        .accessibilityHint(camera.passLocked ? "End the pass to change focus" : "Locked focus ignores taps on either preview")
    }

    private var exposureSwitch: some View {
        Toggle(isOn: Binding(get: { camera.isExposureLocked }, set: camera.setExposureLocked)) {
            Label(camera.isExposureLocked ? (camera.live?.exposure_mode == "manual" ? "Exp manual" : "Exp locked") : "Exp auto",
                  systemImage: camera.isExposureLocked ? "lock.fill" : "sun.max")
                .font(.caption.weight(.semibold))
                .foregroundStyle(camera.isExposureLocked ? .red : .green)
        }
        .toggleStyle(.switch)
        .tint(.red)
        .fixedSize()
        .disabled(!camera.canToggleExposure)
        .accessibilityIdentifier("exposureLockSwitch")
        .accessibilityLabel("Exposure lock")
        .accessibilityValue(camera.isExposureLocked ? "Locked" : "Auto exposure")
        .accessibilityHint(camera.passLocked ? "End the pass to change exposure" : "Unlock to resume automatic exposure")
    }

    private var whiteBalanceSwitch: some View {
        Toggle(isOn: Binding(get: { camera.isWhiteBalanceLocked }, set: camera.setWhiteBalanceLocked)) {
            Label(camera.isWhiteBalanceLocked ? "WB locked" : "WB auto",
                  systemImage: camera.isWhiteBalanceLocked ? "lock.fill" : "camera.filters")
                .font(.caption.weight(.semibold))
                .foregroundStyle(camera.isWhiteBalanceLocked ? .red : .green)
        }
        .toggleStyle(.switch)
        .tint(.red)
        .fixedSize()
        .disabled(!camera.canToggleWhiteBalance)
        .accessibilityIdentifier("whiteBalanceLockSwitch")
        .accessibilityLabel("White balance lock")
        .accessibilityValue(camera.isWhiteBalanceLocked ? "Locked" : "Auto white balance")
        .accessibilityHint(camera.passLocked ? "End the pass to change white balance" : "Unlock to resume automatic white balance")
    }

    private var cameraBadges: some View {
        LazyVGrid(columns: [GridItem(.flexible()), GridItem(.flexible()), GridItem(.flexible())], spacing: 6) {
            if let live = camera.live {
                StatusPill(label: "\(Int((Double(live.selected_resolution.pixels) / 1_000_000).rounded())) MP", expanded: true)
                StatusPill(label: String(format: "ISO %.0f", live.ISO), expanded: true)
                StatusPill(label: shutterLabel(live.exposure_duration), expanded: true)
                StatusPill(label: String(format: "Lens %.2f", live.lens_position), expanded: true)
                StatusPill(label: lidar.statusLabel, color: lidar.isDetected ? .green : (lidar.hasChecked ? .red : .orange), expanded: true)
                StatusPill(label: camera.lastCapture?.camera.map { String(format: "%.2f s", $0.capture_time_seconds) } ?? "Time —", expanded: true)
            }
        }
        .foregroundStyle(.white)
    }

    private var showsCapturedDimensions: Bool {
        guard let last = camera.lastCapture?.camera, let live = camera.live else { return false }
        return last.selected_resolution == live.selected_resolution
            && last.camera_device_type == live.camera_device_type
    }

    private var pixelDimensions: String {
        if showsCapturedDimensions, let last = camera.lastCapture?.camera {
            return "\(last.image_width)×\(last.image_height) px"
        }
        if let live = camera.live {
            return "\(live.selected_resolution.width)×\(live.selected_resolution.height) px"
        }
        return "— px"
    }

    private func shutterLabel(_ seconds: Double) -> String {
        guard seconds.isFinite, seconds > 0 else { return "Shutter —" }
        return seconds < 1 ? String(format: "1/%.0f s", 1 / seconds) : String(format: "%.2f s", seconds)
    }

    private var cameraSettings: some View {
        NavigationStack {
            Form {
                Section("Photogrammetry") {
                    Button("Photogrammetry Mode") { Task { await camera.preparePhotogrammetry() } }
                        .disabled(camera.state != .ready || camera.passLocked)
                    Text("Aim at the wall first. Uses wide, maximum resolution, JPEG quality 100%, flash off and no requested stabilization. Waits for stability, then locks focus, exposure and white balance for the entire pass.")
                        .font(.caption)
                    Button("Lock current settings for pass") { Task { await camera.lockCurrentPass() } }
                        .disabled(camera.state != .ready || camera.passLocked)
                    if camera.passLocked { Button("End pass / unlock controls") { camera.unlockPass() } }
                    if let message = camera.message { Text(message).font(.caption) }
                }
                Group {
                    Section("Camera and resolution") {
                        Picker("Camera", selection: Binding(get: { camera.cameraID }, set: camera.selectCamera)) {
                            ForEach(camera.cameras, id: \.uniqueID) { device in
                                Text(device.deviceType == .builtInWideAngleCamera ? "Wide / main" : "Ultra-wide").tag(device.uniqueID)
                            }
                        }
                        Picker("Resolution", selection: Binding(get: { camera.resolutionID }, set: { id in Task { await camera.selectResolution(id) } })) {
                            ForEach(camera.resolutions) { Text($0.label).tag($0.id) }
                        }
                        .pickerStyle(.wheel)
                        .frame(height: 132)
                        .clipped()
                        .accessibilityHint("Swipe up or down to select a supported resolution")
                        Text("Only supported dimensions are offered. 24 MP requires Photos access and saves a library copy for deferred processing. Actual saved size appears after capture.").font(.caption)
                    }
                    Section("Focus") {
                        focusSwitch
                        Text(camera.isFocusLocked ? "Focus locked. Preview taps do nothing." : "Tap the preview to focus on the wall.").font(.caption)
                        if camera.manualFocusSupported {
                            Text("Lens position")
                            Slider(value: Binding(get: { Double(camera.live?.lens_position ?? 0) }, set: camera.lens), in: 0...1)
                        }
                    }
                    Section("Exposure") {
                        exposureSwitch
                        if camera.manualExposureSupported {
                            Text("Manual ISO: \(Int(camera.live?.ISO ?? 0))")
                            Slider(value: Binding(get: { min(max(Double(camera.live?.ISO ?? 1), camera.isoRange.lowerBound), camera.isoRange.upperBound) }, set: { camera.manualExposure(iso: $0) }), in: camera.isoRange)
                            Text(String(format: "Shutter: %.6f s", camera.live?.exposure_duration ?? 0))
                            Slider(value: Binding(get: { min(max(log10(max(camera.live?.exposure_duration ?? 0.01, 0.000001)), camera.shutterRange.lowerBound), camera.shutterRange.upperBound) }, set: { camera.manualExposure(seconds: pow(10, $0)) }), in: camera.shutterRange)
                        }
                        Text(String(format: "Exposure compensation: %.1f EV", camera.exposureBias))
                        Slider(value: Binding(get: { camera.exposureBias }, set: camera.bias), in: camera.biasRange)
                            .disabled(camera.isExposureLocked)
                    }
                    Section("White balance") {
                        whiteBalanceSwitch
                    }
                }.disabled(camera.state != .ready || camera.passLocked)
                Section("LiDAR") {
                    LabeledContent("Hardware", value: lidar.statusLabel)
                    if let name = lidar.deviceName { LabeledContent("Sensor", value: name) }
                    LabeledContent("Depth capture", value: "Not enabled")
                    Text(lidar.isDetected
                         ? "LiDAR detected. Depth capture settings will be available when capture is implemented."
                         : "LiDAR settings will be available on a device with a LiDAR sensor.")
                        .font(.caption).foregroundStyle(.secondary)
                    Button("Check LiDAR") { lidar.refreshAvailability() }
                }
            }
            .safeAreaInset(edge: .top, spacing: 0) {
                settingsPreview
            }
            .navigationTitle("Camera settings")
            .navigationBarTitleDisplayMode(.inline)
            .toolbar { Button("Done") { showSettings = false } }
        }
        .presentationDetents([.large])
    }

    private var settingsPreview: some View {
        HStack(alignment: .top, spacing: 12) {
            VStack(alignment: .leading, spacing: 5) {
                Text("Live preview").font(.headline)
                if let live = camera.live {
                    Text(live.selected_camera)
                    Text(live.selected_resolution.label)
                }
                Text(camera.isFocusLocked ? "Focus locked" : "Tap the preview to focus")
                    .foregroundStyle(.secondary)
            }
            .font(.caption)
            .frame(maxWidth: .infinity, alignment: .leading)

            CameraPreview(session: camera.session, focusEnabled: camera.canTapToFocus, onFocus: camera.tapFocus)
                .frame(width: 144, height: 108)
                .background(.black)
                .clipShape(RoundedRectangle(cornerRadius: 12))
                .overlay {
                    RoundedRectangle(cornerRadius: 12)
                        .strokeBorder(.primary.opacity(0.15), lineWidth: 1)
                        .allowsHitTesting(false)
                }
                .accessibilityLabel("Live preview of the active capture camera")
                .accessibilityHint(camera.isFocusLocked ? "Focus locked; taps are ignored" : "Tap to focus")
        }
        .padding(.horizontal, 20)
        .padding(.vertical, 10)
        .background(.regularMaterial)
    }

    private var header: some View {
        VStack(spacing: 10) {
            HStack(spacing: 8) {
                StatusPill(label: camera.live == nil ? "Camera" : camera.cameraLabel,
                           color: camera.state == .ready ? .green : .orange)
                Text(pixelDimensions)
                    .font(.caption.monospacedDigit())
                    .lineLimit(1).minimumScaleFactor(0.8)
                    .frame(maxWidth: .infinity)
                    .accessibilityLabel(!showsCapturedDimensions ? "Requested dimensions: \(pixelDimensions)" : "Last image dimensions: \(pixelDimensions)")
                Button { showSettings = true } label: {
                    Image(systemName: "gearshape.fill")
                        .font(.title3)
                        .frame(width: 44, height: 44)
                        .background(.black.opacity(0.45), in: Circle())
                }
                .buttonStyle(.plain)
                .accessibilityLabel("Camera settings")
            }
            HStack(spacing: 8) {
                lockControl(title: "Focus", locked: camera.isFocusLocked, enabled: camera.canToggleFocus,
                            identifier: "focusLockSwitch", action: camera.setFocusLocked)
                lockControl(title: "Exposure", locked: camera.isExposureLocked, enabled: camera.canToggleExposure,
                            identifier: "exposureLockSwitch", action: camera.setExposureLocked)
                lockControl(title: "WB", locked: camera.isWhiteBalanceLocked, enabled: camera.canToggleWhiteBalance,
                            identifier: "whiteBalanceLockSwitch", action: camera.setWhiteBalanceLocked)
            }
            if camera.passLocked {
                Label("Pass locked", systemImage: "lock.fill").font(.caption).foregroundStyle(.red)
            }
        }
        .foregroundStyle(.white)
    }

    private func lockControl(title: String, locked: Bool, enabled: Bool, identifier: String,
                             action: @escaping (Bool) -> Void) -> some View {
        VStack(spacing: 4) {
            Text(title).font(.caption.weight(.semibold))
            Toggle(title, isOn: Binding(get: { locked }, set: action))
                .labelsHidden().toggleStyle(.switch).tint(.red)
                .accessibilityIdentifier(identifier)
                .accessibilityLabel(title == "WB" ? "White balance lock" : "\(title) lock")
                .accessibilityValue(locked ? "Locked" : "Auto")
            Text(locked ? "Locked" : "Auto")
                .font(.caption2.weight(.medium))
                .foregroundStyle(locked ? .red : .green)
        }
        .frame(maxWidth: .infinity)
        .padding(.vertical, 8)
        .background(.black.opacity(0.35), in: RoundedRectangle(cornerRadius: 14))
        .disabled(!enabled)
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

    private var captureStatus: some View {
        VStack(spacing: 10) {
            if let message = camera.message {
                Text(message).font(.footnote)
                    .foregroundStyle(camera.state == .failed ? .red : .white)
                    .multilineTextAlignment(.center)
            }
            Text(network.message)
                .font(.footnote)
                .foregroundStyle(network.state == .failed ? .red : .secondary)
        }
        .foregroundStyle(.white)
        .lineLimit(2)
        .shadow(color: .black, radius: 3)
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
    var color: Color? = nil
    var expanded = false

    var body: some View {
        HStack(spacing: 5) {
            if let color { Circle().fill(color).frame(width: 7, height: 7) }
            Text(label).font(.caption.weight(.medium)).monospacedDigit().lineLimit(1)
        }
        .frame(maxWidth: expanded ? .infinity : nil)
        .padding(.horizontal, 9).padding(.vertical, 6)
        .background(.black.opacity(0.45), in: Capsule())
        .accessibilityElement(children: .combine)
    }
}
