import 'package:flutter/material.dart';
import 'dart:ui' as ui;
import 'package:vector_math/vector_math_64.dart' as vmath;

class PointCloudView extends StatefulWidget {
  final List<vmath.Vector3> points;
  const PointCloudView({super.key, required this.points});

  @override
  State<PointCloudView> createState() => _PointCloudViewState();
}

class _PointCloudViewState extends State<PointCloudView> {
  double _yaw = 0;
  double _pitch = 0;
  double _zoom = 1.0;
  vmath.Vector3 _center = vmath.Vector3.zero();

  @override
  void didUpdateWidget(covariant PointCloudView oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.points.isNotEmpty) {
      _center = _computeCenter(widget.points);
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
          zoom: _zoom,
          center: _center,
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
}

class _PointPainter extends CustomPainter {
  final List<vmath.Vector3> points;
  final double yaw;
  final double pitch;
  final double zoom;
  final vmath.Vector3 center;
  _PointPainter({
    required this.points,
    required this.yaw,
    required this.pitch,
    required this.zoom,
    required this.center,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint()
      ..color = Colors.teal
      ..strokeWidth = 2
      ..style = PaintingStyle.stroke;

    final view = vmath.Matrix4.identity()
      ..translate(size.width / 2, size.height / 2)
      ..scale(zoom * 120.0, -zoom * 120.0, 1)
      ..multiply(_rotation());

    for (final p in points) {
      final v = vmath.Vector3.copy(p);
      v.sub(center);
      final projected = view.transform3(v);
      canvas.drawPoints(ui.PointMode.points, [ui.Offset(projected.x, projected.y)], paint);
    }

    _drawAxes(canvas, size, view);
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

  @override
  bool shouldRepaint(covariant _PointPainter oldDelegate) {
    return oldDelegate.points != points ||
        oldDelegate.yaw != yaw ||
        oldDelegate.pitch != pitch ||
        oldDelegate.zoom != zoom ||
        oldDelegate.center != center;
  }
}
