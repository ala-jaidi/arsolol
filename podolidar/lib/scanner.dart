import 'dart:async';
import 'dart:typed_data';
import 'dart:io';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math_64.dart' as vmath;
import 'package:path_provider/path_provider.dart';
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
  bool _accumulate = false;
  final Map<String, vmath.Vector3> _accum = {};
  final Map<String, int> _accumC = {};
  final double _cell = 0.005;

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
      if (_accumulate) {
        for (final p in pts) {
          final ix = (p.x / _cell).floor();
          final iy = (p.y / _cell).floor();
          final iz = (p.z / _cell).floor();
          final key = '$ix:$iy:$iz';
          final prev = _accum[key];
          if (prev == null) {
            _accum[key] = p;
            _accumC[key] = 1;
          } else {
            final c = (_accumC[key] ?? 1) + 1;
            _accumC[key] = c;
            _accum[key] = vmath.Vector3(
              (prev.x * (c - 1) + p.x) / c,
              (prev.y * (c - 1) + p.y) / c,
              (prev.z * (c - 1) + p.z) / c,
            );
          }
        }
        final agg = _accum.values.toList(growable: false);
        setState(() {
          _points = agg;
          _metrics = computeMetrics(agg);
        });
      } else {
        setState(() {
          _points = pts;
          _metrics = computeMetrics(pts);
        });
      }
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
        ),
        IconButton(
          icon: Icon(_accumulate ? Icons.layers : Icons.layers_clear),
          onPressed: () { setState(() { _accumulate = !_accumulate; }); },
        ),
        IconButton(
          icon: const Icon(Icons.delete_outline),
          onPressed: () { setState(() { _accum.clear(); _accumC.clear(); _points = const []; _metrics = null; }); },
        ),
        IconButton(
          icon: const Icon(Icons.save_alt),
          onPressed: _exportJson,
        ),
        IconButton(
          icon: const Icon(Icons.picture_as_pdf),
          onPressed: _exportPdf,
        ),
        IconButton(
          icon: const Icon(Icons.category),
          onPressed: _exportObj,
        ),
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
                _metricTile('Largeur 25%', _metrics?.w25Cm),
                _metricTile('Largeur 50%', _metrics?.w50Cm),
                _metricTile('Largeur 75%', _metrics?.w75Cm),
                _metricTile('Largeur talon', _metrics?.heelWidthCm),
                _metricTile('Chippaux-Smirak %', _metrics?.csiPercent),
                _metricTile('Staheli ratio', _metrics?.staheliRatio),
                _metricTile('Clarke angle', _metrics?.clarkeAngleDeg),
                _metricTile('Volume cm³', _metrics?.volumeCm3),
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

  Future<void> _exportJson() async {
    if (_metrics == null) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Aucune métrique à exporter')));
      return;
    }
    final jsonStr = metricsToJson(_metrics!);
    final dir = await getApplicationDocumentsDirectory();
    final ts = DateTime.now().toIso8601String().replaceAll(':', '-');
    final file = File('${dir.path}/scan_$ts.json');
    await file.writeAsString(jsonStr);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('JSON enregistré: ${file.path}')));
  }

  Future<void> _exportPdf() async {
    if (_metrics == null) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Aucune métrique à exporter')));
      return;
    }
    final bytes = await generatePdfReport(_metrics!);
    final dir = await getApplicationDocumentsDirectory();
    final ts = DateTime.now().toIso8601String().replaceAll(':', '-');
    final file = File('${dir.path}/scan_$ts.pdf');
    await file.writeAsBytes(bytes, flush: true);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('PDF enregistré: ${file.path}')));
  }

  Future<void> _exportObj() async {
    if (_points.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Aucun nuage à exporter')));
      return;
    }
    final obj = generateHeightmapObj(_points);
    if (obj.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Maillage indisponible')));
      return;
    }
    final dir = await getApplicationDocumentsDirectory();
    final ts = DateTime.now().toIso8601String().replaceAll(':', '-');
    final file = File('${dir.path}/mesh_$ts.obj');
    await file.writeAsString(obj);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('OBJ enregistré: ${file.path}')));
  }
}
