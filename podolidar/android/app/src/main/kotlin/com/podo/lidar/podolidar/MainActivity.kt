package com.podo.lidar.podolidar

import android.graphics.SurfaceTexture
import android.Manifest
import android.os.Bundle
import android.util.Log
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import android.content.pm.PackageManager
import io.flutter.embedding.android.FlutterActivity
import io.flutter.embedding.engine.FlutterEngine
import io.flutter.plugin.common.EventChannel
import io.flutter.plugin.common.MethodCall
import io.flutter.plugin.common.MethodChannel
import io.flutter.view.TextureRegistry
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.Session
import android.media.Image
import android.graphics.ImageFormat
import android.graphics.YuvImage
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.util.concurrent.Executors

class MainActivity : FlutterActivity() {
    private val methodChannelName = "com.podo.lidar/scanner"
    private val eventChannelName = "com.podo.lidar/points"
    private val videoChannelName = "com.podo.lidar/video"
    private var eventSink: EventChannel.EventSink? = null
    private var videoSink: EventChannel.EventSink? = null

    private var session: Session? = null
    private var textureId: Int = -1
    private var surfaceTexture: SurfaceTexture? = null
    private val executor = Executors.newSingleThreadExecutor()
    @Volatile private var streaming = false
    private var installRequested = false
    @Volatile private var sampleStride: Int = 1
    private var targetFps: Double = 24.0
    private var maxPoints: Int = 50000
    private var lastTs: Long = 0L
    private var minDepthM: Float = 0.02f
    private var maxDepthM: Float = 0.60f
    private var removeGround: Boolean = true
    private var clusterFoot: Boolean = true
    private var trackingOnlyNormal: Boolean = true
    private var clusterCellM: Float = 0.01f
    private var groundEpsM: Float = 0.006f
    private var groundOrigin: FloatArray? = null
    private var groundNormal: FloatArray? = null
    private var autoTuneSegmentation: Boolean = true
    private var segLastTuneNs: Long = 0L

    override fun configureFlutterEngine(flutterEngine: FlutterEngine) {
        super.configureFlutterEngine(flutterEngine)
        MethodChannel(flutterEngine.dartExecutor.binaryMessenger, methodChannelName)
            .setMethodCallHandler { call: MethodCall, result: MethodChannel.Result ->
                when (call.method) {
                    "startScan" -> {
                        startScan()
                        result.success(null)
                    }
                    "stopScan" -> {
                        stopScan()
                        result.success(null)
                    }
                    "getCapabilities" -> {
                        val ok = hasDepthSupport()
                        result.success(mapOf("platform" to "android", "depth" to ok))
                    }
                    "setQuality" -> {
                        try {
                            val args = call.arguments as Map<*, *>
                            val tf = (args["targetFps"] as? Number)?.toDouble()
                            val mp = (args["maxPoints"] as? Number)?.toInt()
                            if (tf != null) targetFps = tf
                            if (mp != null) maxPoints = mp
                            result.success(null)
                        } catch (e: Exception) {
                            result.error("quality_error", e.message, null)
                        }
                    }
                    "setSegmentation" -> {
                        try {
                            val args = call.arguments as Map<*, *>
                            val minD = (args["minDepthM"] as? Number)?.toDouble()?.toFloat()
                            val maxD = (args["maxDepthM"] as? Number)?.toDouble()?.toFloat()
                            val remG = args["removeGround"] as? Boolean
                            val clF = args["clusterFoot"] as? Boolean
                            val trN = args["trackingOnlyNormal"] as? Boolean
                            val cCell = (args["clusterCellM"] as? Number)?.toDouble()?.toFloat()
                            val autoT = args["autoTune"] as? Boolean
                            if (minD != null) minDepthM = minD
                            if (maxD != null) maxDepthM = maxD
                            if (remG != null) removeGround = remG
                            if (clF != null) clusterFoot = clF
                            if (trN != null) trackingOnlyNormal = trN
                            if (cCell != null) clusterCellM = cCell
                            if (autoT != null) autoTuneSegmentation = autoT
                            result.success(null)
                        } catch (e: Exception) {
                            result.error("segmentation_error", e.message, null)
                        }
                    }
                    else -> result.notImplemented()
                }
            }

        EventChannel(flutterEngine.dartExecutor.binaryMessenger, eventChannelName)
            .setStreamHandler(object : EventChannel.StreamHandler {
                override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
                    eventSink = events
                }

                override fun onCancel(arguments: Any?) {
                    eventSink = null
                }
            })
        EventChannel(flutterEngine.dartExecutor.binaryMessenger, videoChannelName)
            .setStreamHandler(object : EventChannel.StreamHandler {
                override fun onListen(arguments: Any?, events: EventChannel.EventSink?) {
                    videoSink = events
                }

                override fun onCancel(arguments: Any?) {
                    videoSink = null
                }
            })
    }

    private fun hasDepthSupport(): Boolean {
        return try {
            val s = Session(this)
            val cfg = Config(s)
            cfg.depthMode = Config.DepthMode.AUTOMATIC
            s.configure(cfg)
            s.close()
            true
        } catch (_: Exception) {
            false
        }
    }

    private fun startScan() {
        if (streaming) return
        try {
            if (!ensureSession()) return
            if (textureId == -1) {
                textureId = createExternalTextureId()
                surfaceTexture = SurfaceTexture(textureId)
                surfaceTexture!!.setDefaultBufferSize(1280, 720)
                session!!.setCameraTextureNames(intArrayOf(textureId))
            }
            session!!.resume()
            streaming = true
            executor.execute { streamLoop() }
        } catch (e: Exception) {
            Log.e("Lidar", "Failed to start ARCore session", e)
        }
    }

    private fun stopScan() {
        streaming = false
        try {
            surfaceTexture?.release()
            surfaceTexture = null
            session?.close()
            session = null
        } catch (_: Exception) {}
    }

    private fun ensureSession(): Boolean {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1001)
            return false
        }
        val status = ArCoreApk.getInstance().requestInstall(this, !installRequested)
        installRequested = true
        if (status == ArCoreApk.InstallStatus.INSTALL_REQUESTED) return false
        if (session == null) {
            session = Session(this)
            val config = Config(session)
            config.depthMode = Config.DepthMode.AUTOMATIC
            config.planeFindingMode = Config.PlaneFindingMode.HORIZONTAL
            session!!.configure(config)
        }
        return true
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, grantResults: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == 1001 && grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startScan()
        }
    }

    override fun onResume() {
        super.onResume()
        try { if (session != null) session!!.resume() } catch (_: Exception) {}
    }

    override fun onPause() {
        try { session?.pause() } catch (_: Exception) {}
        super.onPause()
    }

    private fun streamLoop() {
        while (streaming) {
            try {
                val frame: Frame = session!!.update()
                val camera = frame.camera
                if (trackingOnlyNormal && camera.trackingState != com.google.ar.core.TrackingState.TRACKING) {
                    continue
                }
                val planes = frame.getUpdatedTrackables(com.google.ar.core.Plane::class.java)
                for (p in planes) {
                    if (p.trackingState == com.google.ar.core.TrackingState.TRACKING && p.type == com.google.ar.core.Plane.Type.HORIZONTAL_UPWARD_FACING) {
                        val pose = p.centerPose
                        val t = pose.translation
                        val q = pose.rotationQuaternion
                        val n = floatArrayOf(0f, 1f, 0f)
                        val R = quatToMat3(q)
                        val nw = floatArrayOf(
                            R[0]*n[0] + R[1]*n[1] + R[2]*n[2],
                            R[3]*n[0] + R[4]*n[1] + R[5]*n[2],
                            R[6]*n[0] + R[7]*n[1] + R[8]*n[2]
                        )
                        groundOrigin = floatArrayOf(t[0], t[1], t[2])
                        groundNormal = norm3(nw)
                        break
                    }
                }

                var sentPoints = false
                try {
                    val depthImage: Image = frame.acquireDepthImage()
                    val width = depthImage.width
                    val height = depthImage.height
                    val plane = depthImage.planes[0]
                    val buf = plane.buffer.order(ByteOrder.LITTLE_ENDIAN)
                    val intr = camera.imageIntrinsics
                    val fx = intr.focalLength[0].toFloat()
                    val fy = intr.focalLength[1].toFloat()
                    val cx = intr.principalPoint[0].toFloat()
                    val cy = intr.principalPoint[1].toFloat()
                    val q = camera.pose.rotationQuaternion
                    val t = camera.pose.translation
                    val R = quatToMat3(q)
                    var step = 1
                    val total = width * height
                    while ((total / (step * step)) > maxPoints) { step += 1 }
                    val out = ByteBuffer.allocate(4 + (width/step)*(height/step)*12).order(ByteOrder.LITTLE_ENDIAN)
                    var count = 0
                    val cells = ArrayList<String>((width/step)*(height/step))
                    val origin = groundOrigin
                    val normal = groundNormal
                    val zVals = ArrayList<Float>((width/step)*(height/step))
                    for (y in 0 until height step step) {
                        val rowOffset = y * plane.rowStride
                        for (x in 0 until width step step) {
                            val off = rowOffset + x * 2
                            if (off + 2 > buf.capacity()) continue
                            val zmm = buf.getShort(off).toInt() and 0xFFFF
                            val z = zmm.toFloat() / 1000.0f
                            if (!z.isFinite() || z <= 0f) continue
                            if (z < minDepthM || z > maxDepthM) continue
                            val Xc = (x.toFloat() - cx) * z / fx
                            val Yc = (y.toFloat() - cy) * z / fy
                            val Zc = z
                            val Xw = R[0]*Xc + R[1]*Yc + R[2]*Zc + t[0]
                            val Yw = R[3]*Xc + R[4]*Yc + R[5]*Zc + t[1]
                            val Zw = R[6]*Xc + R[7]*Yc + R[8]*Zc + t[2]
                            if (removeGround && origin != null && normal != null) {
                                val dx = Xw - origin[0]
                                val dy = Yw - origin[1]
                                val dz = Zw - origin[2]
                                val d = normal[0]*dx + normal[1]*dy + normal[2]*dz
                                if (d < groundEpsM) continue
                            }
                            out.putFloat(Xw)
                            out.putFloat(Yw)
                            out.putFloat(Zw)
                            count++
                            zVals.add(Zc)
                            val ix = kotlin.math.floor(Xw / clusterCellM).toInt()
                            val iy = kotlin.math.floor(Yw / clusterCellM).toInt()
                            val iz = kotlin.math.floor(Zw / clusterCellM).toInt()
                            cells.add("$ix:$iy:$iz")
                        }
                    }
                    depthImage.close()
                    if (autoTuneSegmentation && zVals.isNotEmpty()) {
                        val now = System.nanoTime()
                        if (segLastTuneNs == 0L || (now - segLastTuneNs) > 500_000_000L) {
                            zVals.sort()
                            val distM = zVals[zVals.size/2]
                            if (distM <= 0.25f) {
                                minDepthM = 0.03f
                                maxDepthM = 0.50f
                                clusterCellM = 0.007f
                                groundEpsM = 0.008f
                            } else if (distM <= 0.35f) {
                                minDepthM = 0.025f
                                maxDepthM = 0.55f
                                clusterCellM = 0.009f
                                groundEpsM = 0.007f
                            } else {
                                minDepthM = 0.02f
                                maxDepthM = 0.60f
                                clusterCellM = 0.012f
                                groundEpsM = 0.006f
                            }
                            segLastTuneNs = now
                        }
                    }
                    if (count > 0) {
                        var useOut = out
                        if (clusterFoot) {
                            val filtered = filterLargestCluster(useOut.array(), count, cells)
                            useOut = ByteBuffer.wrap(filtered).order(ByteOrder.LITTLE_ENDIAN)
                        }
                        val trimmed = ByteBuffer.allocate(4 + count*12).order(ByteOrder.LITTLE_ENDIAN)
                        trimmed.putInt(count)
                        useOut.rewind()
                        var skipHeader = false
                        if (useOut.capacity() >= 4) { skipHeader = true }
                        if (skipHeader) { useOut.position(0) }
                        var i = 0
                        while (i < count*12) {
                            val b0 = useOut.get()
                            val b1 = useOut.get()
                            val b2 = useOut.get()
                            val b3 = useOut.get()
                            trimmed.put(b0); trimmed.put(b1); trimmed.put(b2); trimmed.put(b3)
                            i += 4
                        }
                        eventSink?.success(trimmed.array())
                        sentPoints = true
                    }
                } catch (_: Exception) {}

                if (!sentPoints) {
                    val pointCloud = frame.acquirePointCloud()
                    val pointsBuffer = pointCloud.points
                    pointsBuffer.rewind()
                    val count = pointsBuffer.remaining() / 4
                    val strideFromCount = if (count > maxPoints) (count / maxPoints) else 1
                    sampleStride = kotlin.math.max(sampleStride, strideFromCount)
                    val stride = sampleStride
                    val sampled = (count + stride - 1) / stride
                    val out = ByteBuffer.allocate(4 + sampled * 12).order(ByteOrder.LITTLE_ENDIAN)
                    out.putInt(sampled)
                    var i = 0
                    var idx = 0
                    while (i < count) {
                        val x = pointsBuffer.get()
                        val y = pointsBuffer.get()
                        val z = pointsBuffer.get()
                        pointsBuffer.get()
                        if ((idx % stride) == 0) {
                            out.putFloat(x)
                            out.putFloat(y)
                            out.putFloat(z)
                        }
                        idx++
                        i++
                    }
                    eventSink?.success(out.array())
                    pointCloud.release()
                }

                // Adaptive stride targeting FPS
                val t = System.nanoTime()
                if (lastTs != 0L) {
                    val dt = (t - lastTs).toDouble() / 1_000_000_000.0
                    val target = 1.0 / targetFps
                    sampleStride = if (dt > target) (sampleStride + 1).coerceAtMost(16) else (sampleStride - 1).coerceAtLeast(1)
                }
                lastTs = t

                // Video preview at ~10 FPS
                if (videoSink != null) {
                    try {
                        val img: Image = frame.acquireCameraImage()
                        val nv21 = yuv420ToNv21(img)
                        val yuv = YuvImage(nv21, ImageFormat.NV21, img.width, img.height, null)
                        val baos = ByteArrayOutputStream()
                        yuv.compressToJpeg(android.graphics.Rect(0, 0, img.width, img.height), 60, baos)
                        videoSink?.success(baos.toByteArray())
                        baos.close()
                        img.close()
                    } catch (_: Exception) {}
                }
            } catch (e: Exception) {
                Log.e("Lidar", "Streaming error", e)
                streaming = false
            }
        }
    }

    private fun quatToMat3(q: FloatArray): FloatArray {
        val x = q[0]; val y = q[1]; val z = q[2]; val w = q[3]
        val xx = x*x; val yy = y*y; val zz = z*z
        val xy = x*y; val xz = x*z; val yz = y*z
        val wx = w*x; val wy = w*y; val wz = w*z
        return floatArrayOf(
            1f - 2f*(yy + zz), 2f*(xy - wz),       2f*(xz + wy),
            2f*(xy + wz),       1f - 2f*(xx + zz), 2f*(yz - wx),
            2f*(xz - wy),       2f*(yz + wx),      1f - 2f*(xx + yy)
        )
    }

    private fun norm3(v: FloatArray): FloatArray {
        val n = kotlin.math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        if (n < 1e-6f) return floatArrayOf(0f,1f,0f)
        return floatArrayOf(v[0]/n, v[1]/n, v[2]/n)
    }

    private fun filterLargestCluster(raw: ByteArray, count: Int, cells: List<String>): ByteArray {
        val set = java.util.HashSet<String>(cells.size)
        for (c in cells) set.add(c)
        val visited = java.util.HashSet<String>()
        var best: java.util.HashSet<String> = java.util.HashSet()
        val nbrs = arrayOf(
            intArrayOf(-1,-1,-1),intArrayOf(-1,-1,0),intArrayOf(-1,-1,1),intArrayOf(-1,0,-1),intArrayOf(-1,0,0),intArrayOf(-1,0,1),intArrayOf(-1,1,-1),intArrayOf(-1,1,0),intArrayOf(-1,1,1),
            intArrayOf(0,-1,-1),intArrayOf(0,-1,0),intArrayOf(0,-1,1),intArrayOf(0,0,-1),intArrayOf(0,0,1),intArrayOf(0,1,-1),intArrayOf(0,1,0),intArrayOf(0,1,1),
            intArrayOf(1,-1,-1),intArrayOf(1,-1,0),intArrayOf(1,-1,1),intArrayOf(1,0,-1),intArrayOf(1,0,0),intArrayOf(1,0,1),intArrayOf(1,1,-1),intArrayOf(1,1,0),intArrayOf(1,1,1)
        )
        for (c in set) {
            if (visited.contains(c)) continue
            val queue = java.util.ArrayDeque<String>()
            val cluster = java.util.HashSet<String>()
            visited.add(c)
            queue.add(c)
            while (!queue.isEmpty()) {
                val cur = queue.removeLast()
                cluster.add(cur)
                val parts = cur.split(":")
                if (parts.size != 3) continue
                val ix = parts[0].toIntOrNull() ?: 0
                val iy = parts[1].toIntOrNull() ?: 0
                val iz = parts[2].toIntOrNull() ?: 0
                for (d in nbrs) {
                    val nx = ix + d[0]
                    val ny = iy + d[1]
                    val nz = iz + d[2]
                    val key = "$nx:$ny:$nz"
                    if (set.contains(key) && !visited.contains(key)) {
                        visited.add(key)
                        queue.add(key)
                    }
                }
            }
            if (cluster.size > best.size) best = cluster
        }
        val bb = ByteBuffer.wrap(raw).order(ByteOrder.LITTLE_ENDIAN)
        bb.position(4)
        val filtered = ByteBuffer.allocate(4 + count*12).order(ByteOrder.LITTLE_ENDIAN)
        var kept = 0
        var i = 0
        for (cell in cells) {
            val x = bb.getFloat()
            val y = bb.getFloat()
            val z = bb.getFloat()
            if (best.contains(cell)) {
                filtered.putFloat(x)
                filtered.putFloat(y)
                filtered.putFloat(z)
                kept++
            }
            i++
        }
        val header = ByteBuffer.allocate(4).order(ByteOrder.LITTLE_ENDIAN)
        header.putInt(kept)
        val res = ByteArray(4 + kept*12)
        System.arraycopy(header.array(), 0, res, 0, 4)
        System.arraycopy(filtered.array(), 0, res, 4, kept*12)
        return res
    }

    private fun yuv420ToNv21(image: Image): ByteArray {
        val w = image.width
        val h = image.height
        val ySize = w * h
        val uvSize = w * h / 2
        val nv21 = ByteArray(ySize + uvSize)
        val yBuffer = image.planes[0].buffer
        val uBuffer = image.planes[1].buffer
        val vBuffer = image.planes[2].buffer
        yBuffer.get(nv21, 0, ySize)
        val uRowStride = image.planes[1].rowStride
        val vRowStride = image.planes[2].rowStride
        val uPixelStride = image.planes[1].pixelStride
        val vPixelStride = image.planes[2].pixelStride
        var pos = ySize
        for (row in 0 until h/2) {
            var col = 0
            while (col < w/2) {
                val vu = vBuffer.get(row * vRowStride + col * vPixelStride)
                val uu = uBuffer.get(row * uRowStride + col * uPixelStride)
                nv21[pos++] = vu
                nv21[pos++] = uu
                col++
            }
        }
        return nv21
    }

    private fun createExternalTextureId(): Int {
        val textures = IntArray(1)
        android.opengl.GLES20.glGenTextures(1, textures, 0)
        return textures[0]
    }
}
