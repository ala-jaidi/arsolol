import Flutter
import UIKit
import ARKit

@main
@objc class AppDelegate: FlutterAppDelegate {
  private var methodChannel: FlutterMethodChannel?
  private var eventChannel: FlutterEventChannel?
  private var eventSink: FlutterEventSink?
  private let channelName = "com.podo.lidar/scanner"
  private let eventName = "com.podo.lidar/points"
  private let session = ARSession()
  private var streaming = false

  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    GeneratedPluginRegistrant.register(with: self)
    if let controller = window?.rootViewController as? FlutterViewController {
      methodChannel = FlutterMethodChannel(name: channelName, binaryMessenger: controller.binaryMessenger)
      eventChannel = FlutterEventChannel(name: eventName, binaryMessenger: controller.binaryMessenger)

      methodChannel?.setMethodCallHandler { [weak self] call, result in
        guard let self = self else { return }
        switch call.method {
        case "startScan":
          self.startScan()
          result(nil)
        case "stopScan":
          self.stopScan()
          result(nil)
        default:
          result(FlutterMethodNotImplemented)
        }
      }

      eventChannel?.setStreamHandler(self)
    }
    session.delegate = self
    return super.application(application, didFinishLaunchingWithOptions: launchOptions)
  }
}

extension AppDelegate: FlutterStreamHandler {
  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    eventSink = events
    return nil
  }

  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    eventSink = nil
    return nil
  }
}

extension AppDelegate: ARSessionDelegate {
  private func startScan() {
    guard ARWorldTrackingConfiguration.isSupported else { return }
    let config = ARWorldTrackingConfiguration()
    config.frameSemantics = [.sceneDepth]
    config.worldAlignment = .gravity
    if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
      config.sceneReconstruction = .mesh
    }
    session.run(config, options: [.resetTracking, .removeExistingAnchors])
    streaming = true
  }

  private func stopScan() {
    streaming = false
    session.pause()
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    guard streaming, let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth else { return }
    let depthMap = depthData.depthMap
    let width = CVPixelBufferGetWidth(depthMap)
    let height = CVPixelBufferGetHeight(depthMap)

    CVPixelBufferLockBaseAddress(depthMap, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

    let baseAddress = CVPixelBufferGetBaseAddress(depthMap)!
    let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)

    let camera = frame.camera
    let intrinsics = camera.intrinsics
    let resolution = camera.imageResolution
    let transform = camera.transform

    let step = 4
    var positions: [Float32] = []
    positions.reserveCapacity((width/step)*(height/step)*3)

    for y in stride(from: 0, to: height, by: step) {
      for x in stride(from: 0, to: width, by: step) {
        let idx = y * width + x
        let z = floatBuffer[idx]
        if !z.isFinite || z <= 0 { continue }
        let px = Float32(x)
        let py = Float32(y)
        let fx = Float32(intrinsics.columns.0.x)
        let fy = Float32(intrinsics.columns.1.y)
        let cx = Float32(intrinsics.columns.2.x)
        let cy = Float32(intrinsics.columns.2.y)
        let Xc = (px - cx) * z / fx
        let Yc = (py - cy) * z / fy
        let Zc = z
        let pointCam = SIMD4<Float32>(Xc, Yc, Zc, 1)
        let pointWorld = transform * pointCam
        positions.append(pointWorld.x)
        positions.append(pointWorld.y)
        positions.append(pointWorld.z)
      }
    }

    if positions.isEmpty { return }
    var data = Data(capacity: 4 + positions.count * 4)
    var count = Int32(positions.count / 3)
    withUnsafeBytes(of: &count) { data.append(contentsOf: $0) }
    positions.withUnsafeBytes { raw in
      data.append(raw.bindMemory(to: UInt8.self))
    }
    eventSink?(FlutterStandardTypedData(bytes: data))
  }
}
