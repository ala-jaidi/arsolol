import 'package:flutter/material.dart';
import 'dart:ui' as ui;
import 'package:vector_math/vector_math_64.dart' as vmath;
import 'utils/metrics.dart';

class PointCloudView extends StatefulWidget {
  final List<vmath.Vector3> points;
  final bool autoFit;
  final SevenDims? dims;
  final Map<String, bool>? dimsShow;
  const PointCloudView({super.key, required this.points, this.autoFit = true, this.dims, this.dimsShow});

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
          dims: widget.dims,
          dimsShow: widget.dimsShow,
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
  final SevenDims? dims;
  final Map<String, bool>? dimsShow;
  _PointPainter({
    required this.points,
    required this.yaw,
    required this.pitch,
    required this.zoom,
    required this.center,
    required this.minZ,
    required this.maxZ,
    this.dims,
    this.dimsShow,
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
    _drawBox(canvas, size, view);
    _drawDims(canvas, size, view);
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

  void _drawBox(Canvas canvas, Size size, vmath.Matrix4 view) {
    final d = dims;
    if (d == null) return;
    final wantStrict = dimsShow != null && dimsShow!['BOX_STRICT'] == true;
    final wantBox = dimsShow == null || dimsShow!['BOX'] != false || wantStrict;
    if (!wantBox) return;

    vmath.Matrix3 R;
    vmath.Vector3 meanW;
    double minX, maxX, minY, maxY, minZ, maxZ;
    bool finite(vmath.Vector3 v) => v.x.isFinite && v.y.isFinite && v.z.isFinite;
    if (wantStrict) {
      final fb = computeFootBox(points);
      if (fb == null) return;
      R = fb.R; meanW = fb.mean;
      minX = fb.minX; maxX = fb.maxX;
      minY = fb.minY; maxY = fb.maxY;
      minZ = fb.minZ; maxZ = fb.maxZ;
    } else {
      var c0 = (d.flB - d.flA).normalized();
      var c1 = (d.fbhB - d.fbhA).normalized();
      if (!finite(c0) || !finite(c1)) return;
      final c2 = vmath.Vector3(
        c0.y * c1.z - c0.z * c1.y,
        c0.z * c1.x - c0.x * c1.z,
        c0.x * c1.y - c0.y * c1.x,
      ).normalized();
      if (c2.length2 < 1e-9) return;
      c1 = vmath.Vector3(
        c2.y * c0.z - c2.z * c0.y,
        c2.z * c0.x - c2.x * c0.z,
        c2.x * c0.y - c2.y * c0.x,
      ).normalized();
      R = vmath.Matrix3.columns(c0, c1, c2);
      minX = double.infinity; maxX = -double.infinity;
      minY = double.infinity; maxY = -double.infinity;
      minZ = double.infinity; maxZ = -double.infinity;
      for (final p in points) {
        final r = R.transposed().transform(p - center);
        if (r.x < minX) minX = r.x; if (r.x > maxX) maxX = r.x;
        if (r.y < minY) minY = r.y; if (r.y > maxY) maxY = r.y;
        if (r.z < minZ) minZ = r.z; if (r.z > maxZ) maxZ = r.z;
      }
      meanW = center;
    }

    final loc = [
      vmath.Vector3(minX, minY, minZ),
      vmath.Vector3(maxX, minY, minZ),
      vmath.Vector3(maxX, maxY, minZ),
      vmath.Vector3(minX, maxY, minZ),
      vmath.Vector3(minX, minY, maxZ),
      vmath.Vector3(maxX, minY, maxZ),
      vmath.Vector3(maxX, maxY, maxZ),
      vmath.Vector3(minX, maxY, maxZ),
    ];
    final world = loc.map((v) => meanW + R.transform(v)).toList(growable: false);
    final screen = world.map((w) { final t = view.transform3(w - center); return ui.Offset(t.x, t.y); }).toList(growable: false);
    final p = Paint()
      ..color = const Color(0xFFFFC107)
      ..strokeWidth = 2;
    void edge(int a, int b) { canvas.drawLine(screen[a], screen[b], p); }
    edge(0,1); edge(1,2); edge(2,3); edge(3,0);
    edge(4,5); edge(5,6); edge(6,7); edge(7,4);
    edge(0,4); edge(1,5); edge(2,6); edge(3,7);
    final axisX = vmath.Vector3(R.entry(0,0), R.entry(1,0), R.entry(2,0));
    final axisY = vmath.Vector3(R.entry(0,1), R.entry(1,1), R.entry(2,1));
    final axisZ = vmath.Vector3(R.entry(0,2), R.entry(1,2), R.entry(2,2));
    final x = view.transform3(axisX * 0.12);
    final y = view.transform3(axisY * 0.12);
    final z = view.transform3(axisZ * 0.12);
    final tp = TextPainter(textDirection: TextDirection.ltr);
    tp.text = const TextSpan(style: TextStyle(color: Colors.amber, fontSize: 12), text: 'X'); tp.layout(); tp.paint(canvas, ui.Offset(x.x, x.y));
    tp.text = const TextSpan(style: TextStyle(color: Colors.amber, fontSize: 12), text: 'Y'); tp.layout(); tp.paint(canvas, ui.Offset(y.x, y.y));
    tp.text = const TextSpan(style: TextStyle(color: Colors.amber, fontSize: 12), text: 'Z'); tp.layout(); tp.paint(canvas, ui.Offset(z.x, z.y));
  }

  void _drawDims(Canvas canvas, Size size, vmath.Matrix4 view) {
    final d = dims;
    if (d == null) return;
    final show = dimsShow ?? const {
      'FL': true, 'BFL': true, 'OBFL': true, 'FBH': true, 'FBD': true, 'HB': true, 'IH': true,
    };
    void line(vmath.Vector3 a, vmath.Vector3 b, Color c, String label) {
      final pa = view.transform3(a - center);
      final pb = view.transform3(b - center);
      final p1 = ui.Offset(pa.x, pa.y);
      final p2 = ui.Offset(pb.x, pb.y);
      final p = Paint()..color = c..strokeWidth = 3;
      canvas.drawLine(p1, p2, p);
      final dot = Paint()..color = c..style = PaintingStyle.fill;
      canvas.drawCircle(p1, 4, dot);
      canvas.drawCircle(p2, 4, dot);
      final tp = TextPainter(textDirection: TextDirection.ltr);
      tp.text = TextSpan(style: const TextStyle(color: Colors.black, backgroundColor: Color.fromRGBO(255,255,255,0.7), fontSize: 12), text: label);
      tp.layout();
      final mid = ui.Offset((p1.dx + p2.dx) * 0.5, (p1.dy + p2.dy) * 0.5);
      tp.paint(canvas, mid + const Offset(6, -6));
    }
    if (show['FL'] == true) line(d.flA, d.flB, Colors.deepPurple, 'FL ${d.flCm.toStringAsFixed(1)} cm');
    if (show['BFL'] == true) line(d.bflA, d.bflB, Colors.orange, 'BFL ${d.bflCm.toStringAsFixed(1)} cm');
    if (show['OBFL'] == true) line(d.obflA, d.obflB, Colors.redAccent, 'OBFL ${d.obflCm.toStringAsFixed(1)} cm');
    if (show['FBH'] == true) line(d.fbhA, d.fbhB, Colors.green, 'FBH ${d.fbhCm.toStringAsFixed(1)} cm');
    if (show['FBD'] == true) line(d.fbdA, d.fbdB, Colors.blueGrey, 'FBD ${d.fbdCm.toStringAsFixed(1)} cm');
    if (show['HB'] == true) line(d.hbA, d.hbB, Colors.brown, 'HB ${d.hbCm.toStringAsFixed(1)} cm');
    if (show['IH'] == true) line(d.ihA, d.ihB, Colors.teal, 'IH ${d.ihCm.toStringAsFixed(1)} cm');
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
