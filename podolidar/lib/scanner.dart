import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math_64.dart' as vmath;
import 'point_cloud_view.dart';
import 'utils/metrics.dart';
import 'camera_fallback.dart';

class ScannerPage extends StatefulWidget {
  const ScannerPage({super.key});

  @override
  State<ScannerPage> createState() => _ScannerPageState();
}

class _ScannerPageState extends State<ScannerPage> {
  static const _method = MethodChannel('com.podo.lidar/scanner');
  static const _events = EventChannel('com.podo.lidar/points');

  StreamSubscription? _sub;
  List<vmath.Vector3> _points = const [];
  Metrics? _metrics;
  bool _scanning = false;

  Future<void> _start() async {
    await _method.invokeMethod('startScan');
    _sub?.cancel();
    _sub = _events.receiveBroadcastStream().listen(_onEvent);
    setState(() => _scanning = true);
  }

  Future<void> _stop() async {
    await _method.invokeMethod('stopScan');
    await _sub?.cancel();
    _sub = null;
    setState(() => _scanning = false);
  }

  void _onEvent(dynamic data) {
    if (data is Uint8List) {
      final bd = ByteData.sublistView(data);
      final count = bd.getInt32(0, Endian.little);
      final pts = <vmath.Vector3>[];
      for (int i = 0; i < count; i++) {
        final base = 4 + i * 12;
        final x = bd.getFloat32(base, Endian.little);
        final y = bd.getFloat32(base + 4, Endian.little);
        final z = bd.getFloat32(base + 8, Endian.little);
        pts.add(vmath.Vector3(x, y, z));
      }
      setState(() {
        _points = pts;
        _metrics = computeMetrics(pts);
      });
    }
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Scanner Lidar - Podologie'), actions: [
        IconButton(
          icon: const Icon(Icons.camera_alt),
          onPressed: () {
            Navigator.of(context).push(MaterialPageRoute(builder: (_) => const CameraFallbackPage()));
          },
        )
      ]),
      body: Column(
        children: [
          Expanded(child: PointCloudView(points: _points)),
          Container(
            padding: const EdgeInsets.all(12),
            color: Colors.black12,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                _metricTile('Longueur', _metrics?.lengthCm),
                _metricTile('Largeur', _metrics?.widthCm),
                _metricTile('Hauteur voûte', _metrics?.archHeightCm),
                _metricTile('Angle talon', _metrics?.heelAngleDeg),
                _metricTile('Indice voûte', _metrics == null ? null : (_metrics!.archIndex * 100.0)),
                _metricTile('Largeur avant-pied', _metrics?.forefootWidthCm),
                _metricTile('Courbure voûte', _metrics?.archCurvature),
                _metricTile('Ratio voûte/avant-pied', _metrics?.archForefootRatio),
              ],
            ),
          ),
        ],
      ),
      bottomNavigationBar: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(12),
          child: Row(
            children: [
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: _scanning ? null : _start,
                  icon: const Icon(Icons.play_arrow),
                  label: const Text('Démarrer'),
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: _scanning ? _stop : null,
                  icon: const Icon(Icons.stop),
                  label: const Text('Arrêter'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
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
}
