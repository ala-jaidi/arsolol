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
                val pointCloud = frame.acquirePointCloud()
                val pointsBuffer = pointCloud.points
                pointsBuffer.rewind()
                val count = pointsBuffer.remaining() / 4
                val maxPoints = 50000
                val stride = if (count > maxPoints) (count / maxPoints) else 1
                val sampled = (count + stride - 1) / stride
                val out = ByteBuffer.allocate(4 + sampled * 12).order(ByteOrder.LITTLE_ENDIAN)
                out.putInt(sampled)
                var i = 0
                var written = 0
                while (i < count) {
                    val x = pointsBuffer.get()
                    val y = pointsBuffer.get()
                    val z = pointsBuffer.get()
                    pointsBuffer.get()
                    if ((written % stride) == 0) {
                        out.putFloat(x)
                        out.putFloat(y)
                        out.putFloat(z)
                    }
                    written++
                    i++
                }
                eventSink?.success(out.array())
                pointCloud.release()

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
