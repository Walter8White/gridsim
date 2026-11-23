import AVFoundation
import SwiftUI
import UIKit

struct CameraPreview: UIViewRepresentable {
    let session: AVCaptureSession
    var focusEnabled = true
    var onFocus: (CGPoint) -> Void = { _ in }

    func makeUIView(context: Context) -> PreviewView {
        let view = PreviewView()
        view.isUserInteractionEnabled = focusEnabled
        view.onFocus = onFocus
        view.addGestureRecognizer(UITapGestureRecognizer(target: view, action: #selector(PreviewView.focusTapped(_:))))
        view.connect(to: session)
        view.previewLayer.videoGravity = .resizeAspectFill
        return view
    }

    static func dismantleUIView(_ uiView: PreviewView, coordinator: ()) {
        uiView.disconnect()
    }

    func updateUIView(_ uiView: PreviewView, context: Context) {
        uiView.isUserInteractionEnabled = focusEnabled
        uiView.onFocus = onFocus
        uiView.connect(to: session)
    }
}

final class PreviewView: UIView {
    private var rotationCoordinator: AVCaptureDevice.RotationCoordinator?
    private var rotationObservation: NSKeyValueObservation?

    func connect(to session: AVCaptureSession) {
        if previewLayer.session !== session { previewLayer.session = session }
        let device = session.inputs.compactMap { $0 as? AVCaptureDeviceInput }
            .first(where: { $0.device.hasMediaType(.video) })?.device
        guard let device else { return }
        if rotationCoordinator?.device?.uniqueID != device.uniqueID {
            rotationObservation?.invalidate()
            let coordinator = AVCaptureDevice.RotationCoordinator(device: device, previewLayer: previewLayer)
            rotationCoordinator = coordinator
            rotationObservation = coordinator.observe(\.videoRotationAngleForHorizonLevelPreview, options: [.new]) { [weak self] coordinator, _ in
                Task { @MainActor [weak self] in
                    guard let self, self.rotationCoordinator === coordinator else { return }
                    self.updateRotation()
                }
            }
        }
        updateRotation()
    }

    func disconnect() {
        rotationObservation?.invalidate()
        rotationObservation = nil
        rotationCoordinator = nil
        previewLayer.session = nil
    }

    override func didMoveToWindow() {
        super.didMoveToWindow()
        if let session = previewLayer.session { connect(to: session) }
    }

    override func layoutSubviews() {
        super.layoutSubviews()
        updateRotation()
    }

    private func updateRotation() {
        guard window != nil, let coordinator = rotationCoordinator,
              let connection = previewLayer.connection else { return }
        let angle = coordinator.videoRotationAngleForHorizonLevelPreview
        guard connection.isVideoRotationAngleSupported(angle), connection.videoRotationAngle != angle else { return }
        CATransaction.begin()
        CATransaction.setDisableActions(true)
        connection.videoRotationAngle = angle
        CATransaction.commit()
    }

    var onFocus: (CGPoint) -> Void = { _ in }
    @objc func focusTapped(_ recognizer: UITapGestureRecognizer) {
        onFocus(previewLayer.captureDevicePointConverted(fromLayerPoint: recognizer.location(in: self)))
    }

    override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
    var previewLayer: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
}
