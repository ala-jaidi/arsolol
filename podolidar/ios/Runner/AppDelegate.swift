import Flutter
import UIKit
import ARKit

@main
@objc class AppDelegate: FlutterAppDelegate {
  private var methodChannel: FlutterMethodChannel?
  private var eventChannel: FlutterEventChannel?
  private var videoEventChannel: FlutterEventChannel?
  private var eventSink: FlutterEventSink?
  private var videoSink: FlutterEventSink?
  private let channelName = "com.podo.lidar/scanner"
  private let eventName = "com.podo.lidar/points"
  private let session = ARSession()
  private var streaming = false
  private var lastVideoTime: CFTimeInterval = 0
  private var targetFps: Double = 24.0
  private var maxPoints: Int = 50000
  private var pixelStep: Int = 4
  private var lastFrameTs: CFTimeInterval = 0
  private let processingQueue = DispatchQueue(label: "com.podo.lidar.proc", qos: .userInitiated)
  private var processing = false
  private var minDepthM: Float32 = 0.02
  private var maxDepthM: Float32 = 0.60
  private var removeGround: Bool = true
  private var clusterFoot: Bool = true
  private var trackingOnlyNormal: Bool = true
  private var groundOrigin: SIMD3<Float32>? = nil
  private var groundNormal: SIMD3<Float32>? = nil
  private var clusterCell: Float32 = 0.01

  override func application(
    _ application: UIApplication,
    didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?
  ) -> Bool {
    GeneratedPluginRegistrant.register(with: self)
    if let controller = window?.rootViewController as? FlutterViewController {
      methodChannel = FlutterMethodChannel(name: channelName, binaryMessenger: controller.binaryMessenger)
      eventChannel = FlutterEventChannel(name: eventName, binaryMessenger: controller.binaryMessenger)
      videoEventChannel = FlutterEventChannel(name: "com.podo.lidar/video", binaryMessenger: controller.binaryMessenger)

      methodChannel?.setMethodCallHandler { [weak self] call, result in
        guard let self = self else { return }
        switch call.method {
        case "startScan":
          self.startScan()
          result(nil)
        case "stopScan":
          self.stopScan()
          result(nil)
        case "getCapabilities":
          let supported = ARWorldTrackingConfiguration.isSupported && ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth)
          result(["platform": "ios", "depth": supported])
        case "setQuality":
          if let args = call.arguments as? [String: Any] {
            if let tf = args["targetFps"] as? Double { self.targetFps = tf }
            if let mp = args["maxPoints"] as? Int { self.maxPoints = mp }
          }
          result(nil)
        case "setSegmentation":
          if let args = call.arguments as? [String: Any] {
            if let v = args["minDepthM"] as? Double { self.minDepthM = Float32(v) }
            if let v = args["maxDepthM"] as? Double { self.maxDepthM = Float32(v) }
            if let v = args["removeGround"] as? Bool { self.removeGround = v }
            if let v = args["clusterFoot"] as? Bool { self.clusterFoot = v }
            if let v = args["trackingOnlyNormal"] as? Bool { self.trackingOnlyNormal = v }
            if let v = args["clusterCellM"] as? Double { self.clusterCell = Float32(v) }
          }
          result(nil)
        default:
          result(FlutterMethodNotImplemented)
        }
      }

      eventChannel?.setStreamHandler(self)
      videoEventChannel?.setStreamHandler(VideoStreamHandler { [weak self] sink in self?.videoSink = sink }, onCancel: { [weak self] in self?.videoSink = nil })
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

class VideoStreamHandler: NSObject, FlutterStreamHandler {
  private let onListenHandler: (FlutterEventSink?) -> Void
  private let onCancelHandler: () -> Void
  init(_ onListen: @escaping (FlutterEventSink?) -> Void, onCancel: @escaping () -> Void) {
    self.onListenHandler = onListen
    self.onCancelHandler = onCancel
  }
  func onListen(withArguments arguments: Any?, eventSink events: @escaping FlutterEventSink) -> FlutterError? {
    onListenHandler(events)
    return nil
  }
  func onCancel(withArguments arguments: Any?) -> FlutterError? {
    onCancelHandler()
    return nil
  }
}

extension AppDelegate: ARSessionDelegate {
  private func startScan() {
    if streaming { return }
    guard ARWorldTrackingConfiguration.isSupported else { return }
    let config = ARWorldTrackingConfiguration()
    config.frameSemantics = [.sceneDepth]
    config.worldAlignment = .gravity
    config.planeDetection = [.horizontal]
    if ARWorldTrackingConfiguration.supportsSceneReconstruction(.mesh) {
      config.sceneReconstruction = .mesh
    }
    session.run(config, options: [.resetTracking, .removeExistingAnchors])
    streaming = true
    pixelStep = 4
    lastFrameTs = CACurrentMediaTime()
  }

  private func stopScan() {
    streaming = false
    session.pause()
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    if trackingOnlyNormal {
      switch frame.camera.trackingState {
      case .normal: break
      default: return
      }
    }
    guard streaming, !processing, let depthData = frame.smoothedSceneDepth ?? frame.sceneDepth else { return }
    processing = true
    let depthMap = depthData.depthMap
    let camera = frame.camera
    let intrinsics = camera.intrinsics
    let transform = camera.transform
    let capturedImage = frame.capturedImage
    processingQueue.async { [weak self] in
      guard let self = self else { return }
      let width = CVPixelBufferGetWidth(depthMap)
      let height = CVPixelBufferGetHeight(depthMap)
      CVPixelBufferLockBaseAddress(depthMap, .readOnly)
      let baseAddress = CVPixelBufferGetBaseAddress(depthMap)!
      let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
      var step = self.pixelStep
      let maxPoints = self.maxPoints
      let total = width * height
      while (total / (step * step)) > maxPoints { step += 1 }
      var positions: [Float32] = []
      positions.reserveCapacity((width/step)*(height/step)*3)
      let fx = Float32(intrinsics.columns.0.x)
      let fy = Float32(intrinsics.columns.1.y)
      let cx = Float32(intrinsics.columns.2.x)
      let cy = Float32(intrinsics.columns.2.y)
      var pointCells: [String] = []
      pointCells.reserveCapacity((width/step)*(height/step))
      let gO = self.groundOrigin
      let gN = self.groundNormal
      let groundEps: Float32 = 0.006
      for y in stride(from: 0, to: height, by: step) {
        for x in stride(from: 0, to: width, by: step) {
          let idx = y * width + x
          let z = floatBuffer[idx]
          if !z.isFinite || z <= 0 { continue }
          if z < self.minDepthM || z > self.maxDepthM { continue }
          let px = Float32(x)
          let py = Float32(y)
          let Xc = (px - cx) * z / fx
          let Yc = (py - cy) * z / fy
          let Zc = z
          let pointCam = SIMD4<Float32>(Xc, Yc, Zc, 1)
          let pointWorld = transform * pointCam
          if self.removeGround, let o = gO, let n = gN {
            let pw = SIMD3<Float32>(pointWorld.x, pointWorld.y, pointWorld.z)
            let d = simd_dot(n, pw - o)
            if d < groundEps { continue }
          }
          positions.append(pointWorld.x)
          positions.append(pointWorld.y)
          positions.append(pointWorld.z)
          let ix = Int(floorf(pointWorld.x / self.clusterCell))
          let iy = Int(floorf(pointWorld.y / self.clusterCell))
          let iz = Int(floorf(pointWorld.z / self.clusterCell))
          pointCells.append("\(ix):\(iy):\(iz)")
        }
      }
      CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
      if !positions.isEmpty {
        var usePositions = positions
        if self.clusterFoot {
          var cellSet = Set<String>()
          cellSet.reserveCapacity(pointCells.count)
          for c in pointCells { cellSet.insert(c) }
          var visited = Set<String>()
          var bestCluster: Set<String> = []
          let nbrs = [
            (-1,-1,-1),(-1,-1,0),(-1,-1,1),(-1,0,-1),(-1,0,0),(-1,0,1),(-1,1,-1),(-1,1,0),(-1,1,1),
            (0,-1,-1),(0,-1,0),(0,-1,1),(0,0,-1),(0,0,1),(0,1,-1),(0,1,0),(0,1,1),
            (1,-1,-1),(1,-1,0),(1,-1,1),(1,0,-1),(1,0,0),(1,0,1),(1,1,-1),(1,1,0),(1,1,1)
          ]
          for c in cellSet {
            if visited.contains(c) { continue }
            var queue: [String] = [c]
            var cluster: Set<String> = []
            visited.insert(c)
            while !queue.isEmpty {
              let cur = queue.removeLast()
              cluster.insert(cur)
              let parts = cur.split(separator: ":")
              if parts.count != 3 { continue }
              let ix = Int(parts[0]) ?? 0
              let iy = Int(parts[1]) ?? 0
              let iz = Int(parts[2]) ?? 0
              for (dx,dy,dz) in nbrs {
                let nx = ix + dx
                let ny = iy + dy
                let nz = iz + dz
                let key = "\(nx):\(ny):\(nz)"
                if cellSet.contains(key) && !visited.contains(key) {
                  visited.insert(key)
                  queue.append(key)
                }
              }
            }
            if cluster.count > bestCluster.count { bestCluster = cluster }
          }
          if !bestCluster.isEmpty {
            var filtered: [Float32] = []
            filtered.reserveCapacity(usePositions.count)
            var i = 0
            for cell in pointCells {
              if bestCluster.contains(cell) {
                filtered.append(usePositions[i])
                filtered.append(usePositions[i+1])
                filtered.append(usePositions[i+2])
              }
              i += 3
            }
            usePositions = filtered
          }
        }
        if !usePositions.isEmpty {
          var data = Data(capacity: 4 + usePositions.count * 4)
          var count = Int32(usePositions.count / 3)
          withUnsafeBytes(of: &count) { data.append(contentsOf: $0) }
          usePositions.withUnsafeBytes { raw in
            data.append(raw.bindMemory(to: UInt8.self))
          }
          self.eventSink?(FlutterStandardTypedData(bytes: data))
        }
      }
      let t = CACurrentMediaTime()
      let dt = t - self.lastFrameTs
      self.lastFrameTs = t
      let target = 1.0 / self.targetFps
      if dt > target { self.pixelStep = min(step + 1, 16) } else { self.pixelStep = max(step - 1, 1) }
      if let sink = self.videoSink {
        let now = CACurrentMediaTime()
        let videoInterval = max(0.05, 1.0 / max(10.0, self.targetFps / 2.0))
        if self.lastVideoTime == 0 || (now - self.lastVideoTime) > videoInterval {
          self.lastVideoTime = now
          let ci = CIImage(cvPixelBuffer: capturedImage)
          let context = CIContext(options: nil)
          let rect = ci.extent
          let scale: CGFloat = 640.0 / rect.width
          let targetW = Int(rect.width * scale)
          let targetH = Int(rect.height * scale)
          if let cg = context.createCGImage(ci, from: rect) {
            let uiImage = UIImage(cgImage: cg)
            UIGraphicsBeginImageContext(CGSize(width: targetW, height: targetH))
            uiImage.draw(in: CGRect(x: 0, y: 0, width: targetW, height: targetH))
            let scaled = UIGraphicsGetImageFromCurrentImageContext()
            UIGraphicsEndImageContext()
            if let img = scaled, let bytes = img.jpegData(compressionQuality: 0.6) {
              sink(FlutterStandardTypedData(bytes: bytes))
            }
          }
        }
      }
      self.processing = false
    }
  }

  func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
    for a in anchors {
      if let pa = a as? ARPlaneAnchor, pa.alignment == .horizontal {
        let t = pa.transform
        let o = SIMD3<Float32>(t.columns.3.x, t.columns.3.y, t.columns.3.z)
        let n = SIMD3<Float32>(t.columns.1.x, t.columns.1.y, t.columns.1.z)
        groundOrigin = o
        groundNormal = simd_normalize(n)
      }
    }
  }

  func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
    for a in anchors {
      if let pa = a as? ARPlaneAnchor, pa.alignment == .horizontal {
        let t = pa.transform
        let o = SIMD3<Float32>(t.columns.3.x, t.columns.3.y, t.columns.3.z)
        let n = SIMD3<Float32>(t.columns.1.x, t.columns.1.y, t.columns.1.z)
        groundOrigin = o
        groundNormal = simd_normalize(n)
      }
    }
  }
}
