import 'dart:async';
import 'dart:ui' as ui;
import 'package:camera/camera.dart';
import 'package:flutter/material.dart';

enum ReferenceType { card, a4Width }

class CameraFallbackPage extends StatefulWidget {
  const CameraFallbackPage({super.key});

  @override
  State<CameraFallbackPage> createState() => _CameraFallbackPageState();
}

class _CameraFallbackPageState extends State<CameraFallbackPage> {
  CameraController? _controller;
  bool _initializing = true;
  XFile? _capture;
  ui.Image? _image;
  ReferenceType _refType = ReferenceType.card;
  final List<Offset> _marks = [];
  Size? _paintSize;

  @override
  void initState() {
    super.initState();
    _initCamera();
  }

  Future<void> _initCamera() async {
    final cams = await availableCameras();
    CameraDescription? rear;
    for (final c in cams) {
      if (c.lensDirection == CameraLensDirection.back) { rear = c; break; }
    }
    rear ??= cams.isNotEmpty ? cams.first : null;
    if (rear == null) {
      setState(() { _initializing = false; });
      return;
    }
    _controller = CameraController(rear, ResolutionPreset.medium, enableAudio: false);
    await _controller!.initialize();
    setState(() { _initializing = false; });
  }

  Future<void> _takePicture() async {
    if (_controller == null || !_controller!.value.isInitialized) return;
    final pic = await _controller!.takePicture();
    final bytes = await pic.readAsBytes();
    final codec = await ui.instantiateImageCodec(bytes);
    final frame = await codec.getNextFrame();
    setState(() {
      _capture = pic;
      _image = frame.image;
      _marks.clear();
    });
  }

  double? _refLengthMm() {
    switch (_refType) {
      case ReferenceType.card: return 85.6;
      case ReferenceType.a4Width: return 210.0;
    }
  }

  double? _scaleMmPerPx() {
    if (_marks.length < 2 || _image == null || _paintSize == null) return null;
    final a = _marks[0];
    final b = _marks[1];
    final dDisplay = (a - b).distance;
    final sx = _image!.width / _paintSize!.width;
    final sy = _image!.height / _paintSize!.height;
    final dPx = dDisplay * ((sx + sy) * 0.5);
    final refMm = _refLengthMm();
    if (refMm == null) return null;
    return refMm / dPx;
  }

  double? _footLengthCm() {
    if (_marks.length < 4 || _paintSize == null) return null;
    final s = _scaleMmPerPx();
    if (s == null) return null;
    final heel = _marks[2];
    final toe = _marks[3];
    final dDisplay = (heel - toe).distance;
    final sx = _image!.width / _paintSize!.width;
    final sy = _image!.height / _paintSize!.height;
    final dPx = dDisplay * ((sx + sy) * 0.5);
    return dPx * s / 10.0;
  }

  double? _footWidthCm() {
    if (_marks.length < 6 || _paintSize == null) return null;
    final s = _scaleMmPerPx();
    if (s == null) return null;
    final a = _marks[4];
    final b = _marks[5];
    final dDisplay = (a - b).distance;
    final sx = _image!.width / _paintSize!.width;
    final sy = _image!.height / _paintSize!.height;
    final dPx = dDisplay * ((sx + sy) * 0.5);
    return dPx * s / 10.0;
  }

  void _onTapDown(TapDownDetails d) {
    if (_image == null) return;
    final rb = context.findRenderObject() as RenderBox?;
    if (rb == null) return;
    final local = rb.globalToLocal(d.globalPosition);
    setState(() { _marks.add(local); });
  }

  void _resetMarks() { setState(() { _marks.clear(); }); }

  @override
  void dispose() {
    _controller?.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Mode caméra (étalonnage)'),
        actions: [
          PopupMenuButton<ReferenceType>(
            initialValue: _refType,
            onSelected: (v) => setState(() { _refType = v; }),
            itemBuilder: (_) => const [
              PopupMenuItem(value: ReferenceType.card, child: Text('Carte 85.6 mm')),
              PopupMenuItem(value: ReferenceType.a4Width, child: Text('Feuille A4 210 mm')),
            ],
          )
        ],
      ),
      body: _initializing
          ? const Center(child: CircularProgressIndicator())
          : _capture == null
              ? _cameraView()
              : Column(
                  children: [
                    Expanded(child: _imageView()),
                    Container(
                      padding: const EdgeInsets.all(12),
                      color: Colors.black12,
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                        children: [
                          _metricTile('Longueur', _footLengthCm()),
                          _metricTile('Largeur', _footWidthCm()),
                        ],
                      ),
                    ),
                  ],
                ),
      bottomNavigationBar: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Row(children: [
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _takePicture,
                icon: const Icon(Icons.camera),
                label: const Text('Capturer'),
              ),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: ElevatedButton.icon(
                onPressed: _resetMarks,
                icon: const Icon(Icons.refresh),
                label: const Text('Réinitialiser points'),
              ),
            ),
          ]),
        ),
      ),
    );
  }

  Widget _cameraView() {
    if (_controller == null || !_controller!.value.isInitialized) {
      return const Center(child: Text('Caméra indisponible'));
    }
    return CameraPreview(_controller!);
  }

  Widget _imageView() {
    final img = _image;
    if (img == null) return const SizedBox.shrink();
    return LayoutBuilder(builder: (context, constraints) {
      final aspect = img.width / img.height;
      final w = constraints.maxWidth;
      final h = constraints.maxHeight;
      double drawW, drawH;
      if (w / h > aspect) { drawH = h; drawW = h * aspect; } else { drawW = w; drawH = w / aspect; }
      _paintSize = Size(drawW, drawH);
      return Center(
        child: SizedBox(
          width: drawW,
          height: drawH,
          child: GestureDetector(
            behavior: HitTestBehavior.opaque,
            onTapDown: _onTapDown,
            child: CustomPaint(
              painter: _ImageMarksPainter(img, _marks),
            ),
          ),
        ),
      );
    });
  }
}

Widget _metricTile(String label, double? valueCm) {
  final text = valueCm == null ? '--' : '${valueCm.toStringAsFixed(1)} cm';
  return Column(
    mainAxisSize: MainAxisSize.min,
    children: [
      Text(label, style: const TextStyle(fontWeight: FontWeight.bold)),
      Text(text),
    ],
  );
}

class _ImageMarksPainter extends CustomPainter {
  final ui.Image image;
  final List<Offset> marks;
  _ImageMarksPainter(this.image, this.marks);

  @override
  void paint(Canvas canvas, Size size) {
    final paint = Paint();
    canvas.drawImageRect(
      image,
      Rect.fromLTWH(0, 0, image.width.toDouble(), image.height.toDouble()),
      Rect.fromLTWH(0, 0, size.width, size.height),
      paint,
    );
    final p = Paint()..color = Colors.red..style = PaintingStyle.fill;
    final labels = [
      'Réf A', 'Réf B', 'Talon', 'Orteils', 'Largeur A', 'Largeur B'
    ];
    final tp = TextPainter(textDirection: TextDirection.ltr);
    for (int i = 0; i < marks.length; i++) {
      final m = marks[i];
      canvas.drawCircle(m, 6, p);
      final text = TextSpan(style: const TextStyle(color: Colors.yellow, fontSize: 12), text: labels[i % labels.length]);
      tp.text = text; tp.layout(); tp.paint(canvas, m + const Offset(8, -8));
    }
  }

  @override
  bool shouldRepaint(covariant _ImageMarksPainter oldDelegate) {
    return oldDelegate.image != image || oldDelegate.marks != marks;
  }
}
