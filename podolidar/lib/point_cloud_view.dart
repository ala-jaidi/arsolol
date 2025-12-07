import 'package:flutter/material.dart';
import 'dart:ui' as ui;
import 'package:vector_math/vector_math_64.dart' as vmath;

class PointCloudView extends StatefulWidget {
  final List<vmath.Vector3> points;
  final bool autoFit;
  const PointCloudView({super.key, required this.points, this.autoFit = true});

  @override
  State<PointCloudView> createState() => _PointCloudViewState();
}

class _PointCloudViewState extends State<PointCloudView> {
  double _yaw = 0;
  double _pitch = 0;
  double _zoom = 1.0;
  vmath.Vector3 _center = vmath.Vector3.zero();
  double _fitZoom = 1.0;
  double _minZ = 0.0;
  double _maxZ = 0.0;

  @override
  void didUpdateWidget(covariant PointCloudView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.points.isNotEmpty) {
      _center = _computeCenter(widget.points);
      final bb = _bbox(widget.points);
      _minZ = bb[4]; _maxZ = bb[5];
    }
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onScaleUpdate: (d) {
        setState(() {
          _zoom = d.scale.clamp(0.3, 3.0);
          _yaw += d.focalPointDelta.dx * 0.005;
          _pitch += d.focalPointDelta.dy * 0.005;
        });
      },
      child: CustomPaint(
        painter: _PointPainter(
          points: widget.points,
          yaw: _yaw,
          pitch: _pitch,
          zoom: widget.autoFit ? _fitZoom : _zoom,
          center: _center,
          minZ: _minZ,
          maxZ: _maxZ,
        ),
        child: const SizedBox.expand(),
      ),
    );
  }

  vmath.Vector3 _computeCenter(List<vmath.Vector3> pts) {
    final c = vmath.Vector3.zero();
    for (final p in pts) {
      c.x += p.x;
      c.y += p.y;
      c.z += p.z;
    }
    c.scale(1.0 / pts.length);
    return c;
  }

  List<double> _bbox(List<vmath.Vector3> pts) {
    double minX = double.infinity, minY = double.infinity, minZ = double.infinity;
    double maxX = -double.infinity, maxY = -double.infinity, maxZ = -double.infinity;
    for (final p in pts) {
      if (p.x < minX) minX = p.x; if (p.x > maxX) maxX = p.x;
      if (p.y < minY) minY = p.y; if (p.y > maxY) maxY = p.y;
      if (p.z < minZ) minZ = p.z; if (p.z > maxZ) maxZ = p.z;
    }
    // Compute fit zoom assuming unit scale factor maps 1m ≈ 120px (as in painter)
    final extentX = (maxX - minX).abs();
    final extentY = (maxY - minY).abs();
    // Fit to viewport width/height when built
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final box = context.findRenderObject() as RenderBox?;
      if (box != null && box.size.width > 0 && box.size.height > 0) {
        final pxPerMeter = 120.0;
        final margin = 0.85;
        final zx = (box.size.width * margin) / (extentX * pxPerMeter + 1e-6);
        final zy = (box.size.height * margin) / (extentY * pxPerMeter + 1e-6);
        final fit = zx.isFinite && zy.isFinite ? zx.clamp(0.2, 5.0) : 1.0;
        setState(() { _fitZoom = fit.toDouble(); });
      }
    });
    return [minX, minY, minZ, maxX, maxY, maxZ];
  }
}

class _PointPainter extends CustomPainter {
  final List<vmath.Vector3> points;
  final double yaw;
  final double pitch;
  final double zoom;
  final vmath.Vector3 center;
  final double minZ;
  final double maxZ;
  _PointPainter({
    required this.points,
    required this.yaw,
    required this.pitch,
    required this.zoom,
    required this.center,
    required this.minZ,
    required this.maxZ,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final base = Paint()
      ..strokeWidth = 3
      ..style = PaintingStyle.stroke;

    final view = vmath.Matrix4.identity()
      ..translate(size.width / 2, size.height / 2)
      ..scale(zoom * 120.0, -zoom * 120.0, 1)
      ..multiply(_rotation());

    for (final p in points) {
      final v = vmath.Vector3.copy(p);
      v.sub(center);
      final projected = view.transform3(v);
      // Color by height
      final t = ((p.z - minZ) / ((maxZ - minZ).abs() + 1e-9)).clamp(0.0, 1.0);
      final col = Color.lerp(Colors.blueAccent, Colors.orangeAccent, t)!;
      final paint = base..color = col;
      canvas.drawPoints(ui.PointMode.points, [ui.Offset(projected.x, projected.y)], paint);
    }

    _drawAxes(canvas, size, view);
    _drawGround(canvas, size, view);
  }

  void _drawAxes(Canvas canvas, Size size, vmath.Matrix4 view) {
    final origin = view.transform3(vmath.Vector3.zero());
    final x = view.transform3(vmath.Vector3(0.1, 0, 0));
    final y = view.transform3(vmath.Vector3(0, 0.1, 0));
    final z = view.transform3(vmath.Vector3(0, 0, 0.1));
    final axisPaint = Paint()..strokeWidth = 2;
    axisPaint.color = Colors.red; canvas.drawLine(ui.Offset(origin.x, origin.y), ui.Offset(x.x, x.y), axisPaint);
    axisPaint.color = Colors.green; canvas.drawLine(ui.Offset(origin.x, origin.y), ui.Offset(y.x, y.y), axisPaint);
    axisPaint.color = Colors.blue; canvas.drawLine(ui.Offset(origin.x, origin.y), ui.Offset(z.x, z.y), axisPaint);
  }

  vmath.Matrix4 _rotation() {
    final m = vmath.Matrix4.identity();
    m.rotateY(yaw);
    m.rotateX(pitch);
    return m;
  }

  void _drawGround(Canvas canvas, Size size, vmath.Matrix4 view) {
    final gridPaint = Paint()
      ..color = const Color.fromRGBO(128, 128, 128, 0.3)
      ..strokeWidth = 1;
    for (double x = -0.5; x <= 0.5; x += 0.1) {
      final a = view.transform3(vmath.Vector3(x, -0.5, 0));
      final b = view.transform3(vmath.Vector3(x, 0.5, 0));
      canvas.drawLine(ui.Offset(a.x, a.y), ui.Offset(b.x, b.y), gridPaint);
    }
    for (double y = -0.5; y <= 0.5; y += 0.1) {
      final a = view.transform3(vmath.Vector3(-0.5, y, 0));
      final b = view.transform3(vmath.Vector3(0.5, y, 0));
      canvas.drawLine(ui.Offset(a.x, a.y), ui.Offset(b.x, b.y), gridPaint);
    }
  }

  @override
  bool shouldRepaint(covariant _PointPainter oldDelegate) {
    return oldDelegate.points != points ||
        oldDelegate.yaw != yaw ||
        oldDelegate.pitch != pitch ||
        oldDelegate.zoom != zoom ||
        oldDelegate.center != center;
  }
}
