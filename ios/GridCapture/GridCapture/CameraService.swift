@preconcurrency import AVFoundation
import Combine
import Foundation
import ImageIO
import Photos
import UIKit
import UniformTypeIdentifiers

@MainActor
final class CameraService: NSObject, ObservableObject {
    enum State: Equatable { case idle, configuring, ready, capturing, denied, failed }
    let session = AVCaptureSession()
    var onPreviewFrame: ((Data) -> Void)?
    private let videoOutput = AVCaptureVideoDataOutput()
    private var previewDelegate: PreviewDelegate?
    @Published private(set) var state: State = .idle
    @Published private(set) var message: String?
    @Published private(set) var lastCapture: CaptureRecord?
    @Published private(set) var cameras: [AVCaptureDevice] = []
    @Published private(set) var cameraID = ""
    @Published private(set) var resolutions: [PhotoDimensions] = []
    @Published private(set) var resolutionID = ""
    @Published private(set) var formats: [String] = []
    @Published private(set) var live: CameraMetadata?
    @Published private(set) var photogrammetry = false
    @Published private(set) var passLocked = false
    private let output = AVCapturePhotoOutput()
    private let sessionQueue = DispatchQueue(label: "com.deploya.gridcapture.camera")
    private var device: AVCaptureDevice?
    private var captureRotation: AVCaptureDevice.RotationCoordinator?
    private var delegates: [Int64: PhotoDelegate] = [:]
    private var monitor: Task<Void, Never>?
    var isFocusLocked: Bool { device?.focusMode == .locked }
    var canToggleFocus: Bool {
        state == .ready && !passLocked && device?.isFocusModeSupported(.locked) == true
            && device?.isFocusModeSupported(.continuousAutoFocus) == true
    }
    // Custom exposure also keeps ISO/shutter fixed, so it must never appear as auto.
    var isExposureLocked: Bool { device?.exposureMode == .locked || device?.exposureMode == .custom }
    var canToggleExposure: Bool {
        state == .ready && !passLocked && device?.isExposureModeSupported(.locked) == true
            && device?.isExposureModeSupported(.continuousAutoExposure) == true
    }
    var isWhiteBalanceLocked: Bool { device?.whiteBalanceMode == .locked }
    var canToggleWhiteBalance: Bool {
        state == .ready && !passLocked && device?.isWhiteBalanceModeSupported(.locked) == true
            && device?.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) == true
    }
    var canTapToFocus: Bool {
        state == .ready && !passLocked && !isFocusLocked
            && device?.isFocusPointOfInterestSupported == true
            && device?.isFocusModeSupported(.continuousAutoFocus) == true
    }
    var cameraLabel: String { device?.deviceType == .builtInUltraWideCamera ? "Ultra-wide" : "Wide" }
    var manualFocusSupported: Bool { device?.isLockingFocusWithCustomLensPositionSupported == true }
    var manualExposureSupported: Bool { device?.isExposureModeSupported(.custom) == true }
    var isoRange: ClosedRange<Double> { guard let d = device else { return 1...2 }; return Double(d.activeFormat.minISO)...Double(d.activeFormat.maxISO) }
    var shutterRange: ClosedRange<Double> {
        guard let d = device else { return -4...0 }
        return log10(max(0.000001, d.activeFormat.minExposureDuration.seconds))...log10(d.activeFormat.maxExposureDuration.seconds)
    }
    var biasRange: ClosedRange<Double> { guard let d = device else { return -1...1 }; return Double(d.minExposureTargetBias)...Double(d.maxExposureTargetBias) }
    var exposureBias: Double { Double(device?.exposureTargetBias ?? 0) }

    func start() async {
        guard state == .idle else { return }
        state = .configuring
        let authorized = AVCaptureDevice.authorizationStatus(for: .video) == .authorized
        if !authorized {
            guard await AVCaptureDevice.requestAccess(for: .video) else {
                state = .denied; message = "Enable camera access in Settings."; return
            }
        }
        guard !Task.isCancelled, state == .configuring else { return }
        // Drain any previous stop before mutating session configuration.
        await withCheckedContinuation { continuation in sessionQueue.async { continuation.resume() } }
        guard !Task.isCancelled, state == .configuring else { return }
        do {
            cameras = AVCaptureDevice.DiscoverySession(deviceTypes: [.builtInWideAngleCamera, .builtInUltraWideCamera], mediaType: .video, position: .back).devices
            guard let wide = cameras.first(where: { $0.deviceType == .builtInWideAngleCamera }) else { throw CameraError.problem("No rear wide camera.") }
            try configure(wide)
            await withCheckedContinuation { continuation in
                sessionQueue.async { [session] in session.startRunning(); continuation.resume() }
            }
            guard !Task.isCancelled, state == .configuring else { return }
            state = .ready
            monitor = Task { [weak self] in
                while !Task.isCancelled {
                    self?.refresh()
                    try? await Task.sleep(for: .milliseconds(200))
                }
            }
        } catch { state = .failed; message = error.localizedDescription }
    }
    func stop() {
        monitor?.cancel(); monitor = nil
        sessionQueue.async { [session] in session.stopRunning() }
        state = .idle
    }

    private func configure(_ next: AVCaptureDevice) throws {
        var configured = false
        defer { if !configured { state = .failed } }
        let input = try AVCaptureDeviceInput(device: next)
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .photo
        let oldInputs = session.inputs
        oldInputs.forEach { session.removeInput($0) }
        guard session.canAddInput(input) else {
            oldInputs.forEach { session.addInput($0) }
            throw CameraError.problem("Camera input unavailable.")
        }
        session.addInput(input)
        if !session.outputs.contains(output) {
            guard session.canAddOutput(output) else { throw CameraError.problem("Photo output unavailable.") }
            session.addOutput(output)
        }
        if !session.outputs.contains(videoOutput) {
            guard session.canAddOutput(videoOutput) else { throw CameraError.problem("Preview output unavailable.") }
            videoOutput.alwaysDiscardsLateVideoFrames = true
            videoOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA]
            let delegate = PreviewDelegate { [weak self] data in
                Task { @MainActor [weak self] in self?.onPreviewFrame?(data) }
            }
            previewDelegate = delegate
            videoOutput.setSampleBufferDelegate(delegate, queue: DispatchQueue(label: "com.deploya.gridcapture.preview"))
            session.addOutput(videoOutput)
        }
        try next.lockForConfiguration()
        defer { next.unlockForConfiguration() }
        // Select a real format advertising the largest still dimensions; never invent 24/48 MP support.
        guard let best = next.formats.max(by: { largest($0) < largest($1) }) else { throw CameraError.problem("No photo formats.") }
        next.activeFormat = best
        next.videoZoomFactor = 1
        device = next; cameraID = next.uniqueID
        captureRotation = AVCaptureDevice.RotationCoordinator(device: next, previewLayer: nil)
        formats = next.formats.enumerated().map { index, format in
            let video = CMVideoFormatDescriptionGetDimensions(format.formatDescription)
            return "Format \(index): video \(video.width)×\(video.height), photos " + format.supportedMaxPhotoDimensions.map { "\($0.width)×\($0.height)" }.joined(separator: ", ")
        }
        resolutions = Array(Set(next.formats.flatMap { $0.supportedMaxPhotoDimensions }.map { PhotoDimensions(width: $0.width, height: $0.height) })).sorted { $0.pixels < $1.pixels }
        guard let maximum = resolutions.last else { throw CameraError.problem("No supported photo dimensions.") }
        output.maxPhotoDimensions = CMVideoDimensions(width: maximum.width, height: maximum.height)
        output.maxPhotoQualityPrioritization = .quality
        output.isAutoDeferredPhotoDeliveryEnabled = false
        resolutionID = maximum.id
        if next.isFocusModeSupported(.continuousAutoFocus) { next.focusMode = .continuousAutoFocus }
        if next.isExposureModeSupported(.continuousAutoExposure) { next.exposureMode = .continuousAutoExposure }
        if next.isWhiteBalanceModeSupported(.continuousAutoWhiteBalance) { next.whiteBalanceMode = .continuousAutoWhiteBalance }
        if let connection = output.connection(with: .video), connection.isVideoStabilizationSupported { connection.preferredVideoStabilizationMode = .off }
        passLocked = false
        configured = true
        refresh()
    }
    private func largest(_ format: AVCaptureDevice.Format) -> Int64 {
        format.supportedMaxPhotoDimensions.map { Int64($0.width) * Int64($0.height) }.max() ?? 0
    }
    func selectCamera(_ id: String) {
        guard state == .ready, !passLocked, let next = cameras.first(where: { $0.uniqueID == id }) else { return }
        do { try configure(next); photogrammetry = false } catch { message = error.localizedDescription }
    }
    func selectResolution(_ id: String) async {
        guard state == .ready, !passLocked, let resolution = resolutions.first(where: { $0.id == id }), let d = device else { return }
        state = .configuring
        defer { if state != .failed && state != .idle { state = .ready } }
        do {
            if resolution.requiresDeferredDelivery { try await authorizePhotos() }
            guard state == .configuring else { return }
            guard let format = d.formats.filter({ $0.supportedMaxPhotoDimensions.contains { $0.width == resolution.width && $0.height == resolution.height } }).max(by: { largest($0) < largest($1) }) else { return }
            let oldFormat = d.activeFormat
            let oldMaximum = output.maxPhotoDimensions
            let oldDeferred = output.isAutoDeferredPhotoDeliveryEnabled
            var applied = false
            session.beginConfiguration()
            defer {
                if !applied {
                    do {
                        try d.lockForConfiguration()
                        d.activeFormat = oldFormat
                        d.unlockForConfiguration()
                        output.maxPhotoDimensions = oldMaximum
                        output.isAutoDeferredPhotoDeliveryEnabled = oldDeferred
                    } catch { state = .failed }
                }
                session.commitConfiguration()
            }
            try d.lockForConfiguration()
            d.activeFormat = format
            d.unlockForConfiguration()
            let maximum = format.supportedMaxPhotoDimensions.max { Int64($0.width) * Int64($0.height) < Int64($1.width) * Int64($1.height) }!
            output.maxPhotoDimensions = maximum
            if resolution.requiresDeferredDelivery && !output.isAutoDeferredPhotoDeliverySupported { throw CameraError.problem("24 MP deferred processing is unavailable for this format.") }
            output.isAutoDeferredPhotoDeliveryEnabled = resolution.requiresDeferredDelivery
            resolutionID = id; applied = true; refresh()
        } catch { message = error.localizedDescription }
    }
    private func authorizePhotos() async throws {
        let status = await PHPhotoLibrary.requestAuthorization(for: .readWrite)
        guard status == .authorized || status == .limited else { throw CameraError.problem("24 MP requires Photos access to finish processing and retrieve the image. Choose 12 MP or maximum direct capture instead.") }
    }
    func lockCurrentPass() async {
        guard state == .ready else { return }
        state = .configuring
        do { try await settleAndLock(); message = "Current settings locked for the pass." }
        catch { message = error.localizedDescription }
        if state != .idle { state = .ready }
    }
    private func change(_ body: (AVCaptureDevice) -> Void) {
        guard state == .ready, !passLocked, let device else { return }
        do { try device.lockForConfiguration(); defer { device.unlockForConfiguration() }; body(device); refresh() }
        catch { message = error.localizedDescription }
    }
    func setFocusLocked(_ locked: Bool) {
        guard canToggleFocus else { return }
        change { $0.focusMode = locked ? .locked : .continuousAutoFocus }
    }
    func lens(_ position: Double) { change { if $0.isLockingFocusWithCustomLensPositionSupported { $0.setFocusModeLocked(lensPosition: Float(position), completionHandler: nil) } } }
    func tapFocus(_ point: CGPoint) {
        // Enforce the lock here too: neither preview can override it with a tap.
        guard canTapToFocus else { return }
        change { d in
            d.focusPointOfInterest = point
            d.focusMode = .continuousAutoFocus
        }
    }
    func setExposureLocked(_ locked: Bool) {
        guard canToggleExposure else { return }
        change { $0.exposureMode = locked ? .locked : .continuousAutoExposure }
    }
    func manualExposure(iso: Double? = nil, seconds: Double? = nil) {
        change { d in
            guard d.isExposureModeSupported(.custom) else { return }
            let value = min(max(Float(iso ?? Double(d.iso)), d.activeFormat.minISO), d.activeFormat.maxISO)
            let duration = min(max(seconds ?? d.exposureDuration.seconds, d.activeFormat.minExposureDuration.seconds), d.activeFormat.maxExposureDuration.seconds)
            d.setExposureModeCustom(duration: CMTime(seconds: duration, preferredTimescale: 1_000_000_000), iso: value, completionHandler: nil)
        }
    }
    func bias(_ value: Double) {
        guard !isExposureLocked else { return }
        change { $0.setExposureTargetBias(Float(value), completionHandler: nil) }
    }
    func setWhiteBalanceLocked(_ locked: Bool) {
        guard canToggleWhiteBalance else { return }
        change { $0.whiteBalanceMode = locked ? .locked : .continuousAutoWhiteBalance }
    }
    func unlockPass() { guard state == .ready else { return }; passLocked = false; photogrammetry = false; message = "Pass unlocked; settings can be changed." }

    func preparePhotogrammetry() async {
        guard state == .ready, !passLocked else { return }
        state = .configuring; passLocked = false
        do {
            guard let wide = cameras.first(where: { $0.deviceType == .builtInWideAngleCamera }) else { throw CameraError.problem("No wide camera.") }
            try configure(wide)
            if resolutions.last?.requiresDeferredDelivery == true {
                try await authorizePhotos()
                guard output.isAutoDeferredPhotoDeliverySupported else { throw CameraError.problem("24 MP deferred processing unavailable.") }
                output.isAutoDeferredPhotoDeliveryEnabled = true
            }
            try await settleAndLock()
            photogrammetry = true; state = .ready
            message = "Photogrammetry ready: focus, exposure and white balance locked."
        } catch { if state != .failed && state != .idle { state = .ready }; photogrammetry = false; message = error.localizedDescription }
    }
    private func settleAndLock() async throws {
        guard let d = device else { throw CameraError.problem("No camera.") }
        message = "Aim at the wall; waiting for focus/exposure/white balance…"
        // Require a continuous stable window, allowing time for automatic adjustment to begin.
        try await Task.sleep(for: .milliseconds(700))
        var stable = 0
        for _ in 0..<80 {
            try Task.checkCancellation()
            guard state == .configuring else { throw CancellationError() }
            stable = (d.isAdjustingFocus || d.isAdjustingExposure || d.isAdjustingWhiteBalance) ? 0 : stable + 1
            if stable >= 5 { break }
            try await Task.sleep(for: .milliseconds(100))
        }
        guard stable >= 5 else { throw CameraError.problem("Camera did not stabilize. Re-aim and retry Photogrammetry Mode.") }
        guard d.isFocusModeSupported(.locked), d.isExposureModeSupported(.locked), d.isWhiteBalanceModeSupported(.locked) else { throw CameraError.problem("This camera cannot lock all pass settings.") }
        try d.lockForConfiguration()
        d.focusMode = .locked; d.exposureMode = .locked; d.whiteBalanceMode = .locked
        d.unlockForConfiguration()
        passLocked = true; refresh()
    }
    private func refresh() { live = snapshot() }
    private func snapshot() -> CameraMetadata? {
        guard let d = device, let resolution = resolutions.first(where: { $0.id == resolutionID }) else { return nil }
        let gains = d.deviceWhiteBalanceGains
        let temperature = d.temperatureAndTintValues(for: gains)
        return CameraMetadata(selected_camera: d.localizedName, camera_device_type: d.deviceType.rawValue,
            selected_resolution: resolution, maxPhotoDimensions: PhotoDimensions(width: output.maxPhotoDimensions.width, height: output.maxPhotoDimensions.height),
            lens_position: d.lensPosition, focus_mode: d.focusMode == .locked ? "locked" : (d.focusMode == .autoFocus ? "auto" : "continuous auto"),
            exposure_mode: d.exposureMode == .custom ? "manual" : (d.exposureMode == .locked ? "locked" : "auto"),
            exposure_duration: d.exposureDuration.seconds, ISO: d.iso, aperture: Double(d.lensAperture),
            white_balance_mode: d.whiteBalanceMode == .locked ? "locked" : "auto",
            white_balance_gains: [gains.redGain, gains.greenGain, gains.blueGain], white_balance_temperature: temperature.temperature, white_balance_tint: temperature.tint)
    }

    func capture(command: CaptureCommand = CaptureCommand(captureID: UUID().uuidString, targetPositionMM: nil, measuredPositionMM: nil), motion: MotionSample, completion: ((Result<CaptureRecord, Error>) -> Void)? = nil) {
        guard state == .ready, var metadata = snapshot() else { completion?(.failure(CameraError.problem("Camera is not ready."))); return }
        if metadata.selected_resolution.requiresDeferredDelivery && !output.isAutoDeferredPhotoDeliveryEnabled {
            completion?(.failure(CameraError.problem("Select 24 MP in Camera settings to enable Photos processing.")))
            message = "Select 24 MP in Camera settings to enable Photos processing."
            return
        }
        // Snapshot capture orientation once per request, independently of either preview.
        // AVCapturePhotoOutput writes the corresponding EXIF orientation without resampling.
        guard let connection = output.connection(with: .video), let captureRotation else {
            completion?(.failure(CameraError.problem("Photo orientation is not ready.")))
            return
        }
        let angle = captureRotation.videoRotationAngleForHorizonLevelCapture
        guard connection.isVideoRotationAngleSupported(angle) else {
            message = "The camera cannot capture at the current orientation."
            completion?(.failure(CameraError.problem(message!)))
            return
        }
        connection.videoRotationAngle = angle
        state = .capturing
        let started = Date()
        let settings = AVCapturePhotoSettings(format: [AVVideoCodecKey: AVVideoCodecType.jpeg, AVVideoCompressionPropertiesKey: [AVVideoQualityKey: 1.0]])
        settings.maxPhotoDimensions = CMVideoDimensions(width: metadata.selected_resolution.width, height: metadata.selected_resolution.height)
        settings.photoQualityPrioritization = .quality
        settings.flashMode = .off
        // The legacy still-image stabilization switch is deprecated and ignored on modern devices.
        // Video stabilization is disabled on the connection; do not request a digital zoom/crop.
        let id = settings.uniqueID
        let delegate = PhotoDelegate(onExposure: { [weak self] in
            let exposureDate = Date()
            // The delegate callback timestamp is recorded separately from the sensor timestamp.
            Task { @MainActor in
                if let current = self?.snapshot() { metadata = current }
                metadata.timestamp_unix = exposureDate.timeIntervalSince1970
            }
        }) { [weak self] result in
            Task { @MainActor in
                guard let self else { return }
                self.delegates.removeValue(forKey: id)
                do {
                    let photo = try result.get()
                    let data = try await self.finalPhotoData(photo)
                    guard let source = CGImageSourceCreateWithData(data as CFData, nil), let props = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [String: Any] else { throw CameraError.problem("Cannot read image dimensions.") }
                    metadata.image_width = (props[kCGImagePropertyPixelWidth as String] as? NSNumber)?.intValue ?? 0
                    metadata.image_height = (props[kCGImagePropertyPixelHeight as String] as? NSNumber)?.intValue ?? 0
                    metadata.orientation = (props[kCGImagePropertyOrientation as String] as? NSNumber)?.intValue ?? 1
                    let exif = props[kCGImagePropertyExifDictionary as String] as? [String: Any] ?? [:]
                    metadata.focal_length = (exif[kCGImagePropertyExifFocalLength as String] as? NSNumber)?.doubleValue
                    metadata.aperture = (exif[kCGImagePropertyExifFNumber as String] as? NSNumber)?.doubleValue ?? metadata.aperture
                    metadata.exposure_duration = (exif[kCGImagePropertyExifExposureTime as String] as? NSNumber)?.doubleValue ?? metadata.exposure_duration
                    if let iso = exif[kCGImagePropertyExifISOSpeedRatings as String] as? [NSNumber], let first = iso.first { metadata.ISO = first.floatValue }
                    metadata.photo_timestamp_seconds = photo.timestamp.seconds.isFinite ? photo.timestamp.seconds : 0
                    // Map the sensor presentation time (host clock) to Unix time, preserving fractions.
                    if photo.timestamp.isValid && photo.timestamp.seconds.isFinite {
                        metadata.timestamp_unix = Date().timeIntervalSince1970 - CMClockGetTime(CMClockGetHostTimeClock()).seconds + photo.timestamp.seconds
                    }
                    metadata.capture_time_seconds = Date().timeIntervalSince(started)
                    let record = try self.save(data: data, command: command, motion: motion, metadata: metadata)
                    self.lastCapture = record; self.state = .ready
                    let dimensionsMatch = Set([metadata.image_width, metadata.image_height]) == Set([Int(metadata.selected_resolution.width), Int(metadata.selected_resolution.height)])
                    self.message = dimensionsMatch ? "Saved \(record.photoFilename)" : "Saved \(metadata.image_width)×\(metadata.image_height); requested \(metadata.selected_resolution.id)."
                    completion?(.success(record))
                } catch { self.state = .ready; self.message = error.localizedDescription; completion?(.failure(error)) }
            }
        }
        delegates[id] = delegate
        output.capturePhoto(with: settings, delegate: delegate)
    }
    private func finalPhotoData(_ photo: AVCapturePhoto) async throws -> Data {
        guard let data = photo.fileDataRepresentation() else { throw CameraError.problem("No photo data.") }
        guard photo is AVCaptureDeferredPhotoProxy else { return data }
        // A 24 MP proxy is not a final image. Photos owns the deferred processing pipeline.
        var identifier: String?
        try await PHPhotoLibrary.shared().performChanges {
            let request = PHAssetCreationRequest.forAsset()
            request.addResource(with: .photoProxy, data: data, options: nil)
            identifier = request.placeholderForCreatedAsset?.localIdentifier
        }
        guard let identifier, let asset = PHAsset.fetchAssets(withLocalIdentifiers: [identifier], options: nil).firstObject else { throw CameraError.problem("Cannot retrieve processed Photos asset.") }
        let finalData: Data = try await withCheckedThrowingContinuation { continuation in
            let options = PHImageRequestOptions()
            options.version = .current; options.deliveryMode = .highQualityFormat; options.isNetworkAccessAllowed = true
            PHImageManager.default().requestImageDataAndOrientation(for: asset, options: options) { data, _, _, info in
                if let error = info?[PHImageErrorKey] as? Error { continuation.resume(throwing: error) }
                else if let data { continuation.resume(returning: data) }
                else { continuation.resume(throwing: CameraError.problem("Photos did not return the final image.")) }
            }
        }
        guard let source = CGImageSourceCreateWithData(finalData as CFData, nil) else { throw CameraError.problem("Invalid processed photo." ) }
        if CGImageSourceGetType(source) as String? == UTType.jpeg.identifier { return finalData }
        // Keep the JPEG upload contract and preserve EXIF when Photos returns another container.
        let jpeg = NSMutableData()
        guard let destination = CGImageDestinationCreateWithData(jpeg, UTType.jpeg.identifier as CFString, 1, nil) else { throw CameraError.problem("Cannot encode final JPEG.") }
        var properties = CGImageSourceCopyPropertiesAtIndex(source, 0, nil) as? [String: Any] ?? [:]
        properties[kCGImageDestinationLossyCompressionQuality as String] = 1.0
        CGImageDestinationAddImageFromSource(destination, source, 0, properties as CFDictionary)
        guard CGImageDestinationFinalize(destination) else { throw CameraError.problem("JPEG encoding failed.") }
        return jpeg as Data
    }
    private func save(data: Data, command: CaptureCommand, motion: MotionSample, metadata: CameraMetadata) throws -> CaptureRecord {
        let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("Captures", isDirectory: true)
        try FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        let safeID = command.captureID.replacingOccurrences(of: "/", with: "-")
        let record = CaptureRecord(captureID: command.captureID, targetPositionMM: command.targetPositionMM, measuredPositionMM: command.measuredPositionMM,
            photoFilename: "\(safeID).jpg", metadataFilename: "\(safeID).json", commandReceivedAt: command.receivedAt, captureCompletedAt: Date(), motion: motion,
            camera: metadata, pass_index: command.passIndex, x_offset_cm: command.xOffsetCM)
        let encoder = JSONEncoder(); encoder.outputFormatting = [.prettyPrinted, .sortedKeys]; encoder.dateEncodingStrategy = .iso8601
        try data.write(to: directory.appendingPathComponent(record.photoFilename), options: .atomic)
        try encoder.encode(record).write(to: directory.appendingPathComponent(record.metadataFilename), options: .atomic)
        return record
    }
}

private final class PhotoDelegate: NSObject, AVCapturePhotoCaptureDelegate {
    let onExposure: () -> Void
    let completion: (Result<AVCapturePhoto, Error>) -> Void
    private var result: Result<AVCapturePhoto, Error>?
    init(onExposure: @escaping () -> Void, completion: @escaping (Result<AVCapturePhoto, Error>) -> Void) { self.onExposure = onExposure; self.completion = completion }
    func photoOutput(_ output: AVCapturePhotoOutput, willCapturePhotoFor resolvedSettings: AVCaptureResolvedPhotoSettings) { onExposure() }
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishProcessingPhoto photo: AVCapturePhoto, error: Error?) {
        result = error.map { .failure($0) } ?? .success(photo)
    }
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishCapturingDeferredPhotoProxy proxy: AVCaptureDeferredPhotoProxy?, error: Error?) {
        if let error { result = .failure(error) }
        else if let proxy { result = .success(proxy) }
        else { result = .failure(CameraError.problem("No deferred photo proxy.")) }
    }
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishCaptureFor resolvedSettings: AVCaptureResolvedPhotoSettings, error: Error?) {
        completion(error.map { .failure($0) } ?? result ?? .failure(CameraError.problem("No photo returned.")))
    }
}
private enum CameraError: LocalizedError {
    case problem(String)
    var errorDescription: String? { if case .problem(let text) = self { return text }; return nil }
}

// A small JPEG at 5 fps keeps preview traffic separate from full-resolution stills.
nonisolated private final class PreviewDelegate: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, @unchecked Sendable {
    private let context = CIContext()
    private var lastFrame = CMTime.zero
    private let deliver: @Sendable (Data) -> Void

    init(deliver: @escaping @Sendable (Data) -> Void) { self.deliver = deliver }

    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        let timestamp = CMSampleBufferGetPresentationTimeStamp(sampleBuffer)
        guard CMTimeGetSeconds(timestamp - lastFrame) >= 0.2,
              let buffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        lastFrame = timestamp
        let image = CIImage(cvPixelBuffer: buffer)
        let scale = min(1, 640 / max(image.extent.width, image.extent.height))
        let scaled = image.transformed(by: CGAffineTransform(scaleX: scale, y: scale))
        guard let cgImage = context.createCGImage(scaled, from: scaled.extent),
              let data = UIImage(cgImage: cgImage).jpegData(compressionQuality: 0.55) else { return }
        deliver(data)
    }
}
