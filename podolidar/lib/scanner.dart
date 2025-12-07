import 'dart:async';
import 'dart:typed_data';
import 'dart:io';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math_64.dart' as vmath;
import 'package:path_provider/path_provider.dart';
import 'point_cloud_view.dart';
import 'utils/metrics.dart';

class ScannerPage extends StatefulWidget {
  const ScannerPage({super.key});

  @override
  State<ScannerPage> createState() => _ScannerPageState();
}

class _ScannerPageState extends State<ScannerPage> {
  static const _method = MethodChannel('com.podo.lidar/scanner');
  static const _events = EventChannel('com.podo.lidar/points');
  static const _eventsVideo = EventChannel('com.podo.lidar/video');

  StreamSubscription? _sub;
  StreamSubscription? _videoSub;
  List<vmath.Vector3> _points = const [];
  Metrics? _metrics;
  SevenDims? _dims;
  Uint8List? _videoJpeg;
  bool _scanning = false;
  bool _accumulate = true;
  final Map<String, bool> _showDim = {
    'BOX': true,
    'BOX_STRICT': false,
    'FL': true, 'BFL': true, 'OBFL': true, 'FBH': true, 'FBD': true, 'HB': true, 'IH': true,
  };
  final Map<String, vmath.Vector3> _accum = {};
  final Map<String, int> _accumC = {};
  final double _cell = 0.005;
  int _lastMetricsMs = 0;
  final int _metricsIntervalMs = 200;

  Future<void> _start() async {
    await _method.invokeMethod('startScan');
    _sub?.cancel();
    _sub = _events.receiveBroadcastStream().listen(_onEvent);
    _videoSub?.cancel();
    _videoSub = _eventsVideo.receiveBroadcastStream().listen(_onVideo);
    setState(() => _scanning = true);
  }

  Future<void> _stop() async {
    await _method.invokeMethod('stopScan');
    await _sub?.cancel();
    _sub = null;
    await _videoSub?.cancel();
    _videoSub = null;
    setState(() => _scanning = false);
  }

  void _onEvent(dynamic data) {
    if (data is Uint8List) {
      final bd = ByteData.sublistView(data);
      if (bd.lengthInBytes < 4) return;
      int count = bd.getInt32(0, Endian.little);
      final maxCount = ((bd.lengthInBytes - 4) ~/ 12);
      if (count < 0 || count > maxCount) {
        count = maxCount;
        if (count <= 0) return;
      }
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
        final now = DateTime.now().millisecondsSinceEpoch;
        if (now - _lastMetricsMs >= _metricsIntervalMs) {
          try {
            final m = computeMetrics(agg);
            final d = computeSevenDimensions(agg);
            setState(() { _points = agg; _metrics = m; _dims = d; _lastMetricsMs = now; });
          } catch (_) {
            setState(() { _points = agg; });
          }
        } else {
          setState(() { _points = agg; });
        }
      } else {
        final now = DateTime.now().millisecondsSinceEpoch;
        if (now - _lastMetricsMs >= _metricsIntervalMs) {
          try {
            final m = computeMetrics(pts);
            final d = computeSevenDimensions(pts);
            setState(() { _points = pts; _metrics = m; _dims = d; _lastMetricsMs = now; });
          } catch (_) {
            setState(() { _points = pts; });
          }
        } else {
          setState(() { _points = pts; });
        }
      }
    }
  }

  void _onVideo(dynamic data) {
    if (data is Uint8List) {
      setState(() { _videoJpeg = data; });
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
        IconButton(
          icon: const Icon(Icons.straighten),
          onPressed: _toggleDims,
        ),
      ]),
      body: Stack(children: [
        Column(
          children: [
            Expanded(child: PointCloudView(points: _points, autoFit: true, dims: _dims, dimsShow: _showDim, backgroundJpeg: _videoJpeg)),
            Container(
              padding: const EdgeInsets.all(8),
              color: Colors.black12,
              child: SingleChildScrollView(
                scrollDirection: Axis.horizontal,
                child: Row(
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
            ),
          ],
        ),
        Positioned(
          top: 8,
          right: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
            decoration: BoxDecoration(color: Colors.black54, borderRadius: BorderRadius.circular(12)),
            child: Text(
              '${_accumulate ? 'Accumulation' : 'Instantané'} · ${_points.length} pts',
              style: const TextStyle(color: Colors.white),
            ),
          ),
        ),
      ]),
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
        Text(label, style: const TextStyle(fontWeight: FontWeight.bold, fontSize: 12)),
        Text(text, style: const TextStyle(fontSize: 12)),
      ],
    );
  }

  Future<void> _exportJson() async {
    if (_metrics == null) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Aucune métrique à exporter')));
      return;
    }
    final base = jsonDecode(metricsToJson(_metrics!));
    if (_dims != null) { base['seven_dimensions'] = sevenDimsToMap(_dims!); }
    final jsonStr = jsonEncode(base);
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
    final bytes = await generatePdfReportWithDims(_metrics!, _dims);
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

  void _toggleDims() {
    showModalBottomSheet(context: context, builder: (_) {
      return StatefulBuilder(builder: (context, setS) {
        Widget chip(String k) {
          return FilterChip(
            selected: _showDim[k] == true,
            label: Text(k),
            onSelected: (v) { setS(() { _showDim[k] = v; }); setState(() {}); },
          );
        }
        return Padding(
          padding: const EdgeInsets.all(12),
          child: Wrap(spacing: 8, runSpacing: 8, children: [
            chip('BOX'), chip('BOX_STRICT'), chip('FL'), chip('BFL'), chip('OBFL'), chip('FBH'), chip('FBD'), chip('HB'), chip('IH'),
          ]),
        );
      });
    });
  }
}
