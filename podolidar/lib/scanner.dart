import 'dart:async';
import 'dart:typed_data';
import 'dart:io';
import 'dart:convert';
import 'dart:ui' as ui;
import 'dart:math' as math;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:vector_math/vector_math_64.dart' as vmath;
import 'package:path_provider/path_provider.dart';
import 'point_cloud_view.dart';
import 'utils/metrics.dart';

enum ScanPhase { prep, scanning, result }

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
    'BOX_STRICT': true,
    'GRID': true,
    'FL': true, 'BFL': true, 'OBFL': true, 'FBH': true, 'FBD': true, 'HB': true, 'IH': true,
  };
  final String _quality = 'performance';
  final Map<String, vmath.Vector3> _accum = {};
  final Map<String, int> _accumC = {};
  final double _cell = 0.01;
  int _lastMetricsMs = 0;
  final int _metricsIntervalMs = 500;
  final int _targetVox = 3000;
  double _coverage = 0.0;
  SevenDims? _dimsPrev;
  String _lastGuide = '';
  double _fpsAvg = 0.0;
  int _lastEventMs = 0;
  int _startMs = 0;
  bool _calibrated = false;
  bool _thermalDown = false;
  int _lastTuneMs = 0;
  Timer? _watchdog;
  bool _depthOk = true;
  int _videoRotTurns = 0;
  ScanPhase _phase = ScanPhase.prep;
  final bool _hudMinimal = true;
  bool _doneTop = false;
  bool _doneMedial = false;
  bool _doneLateral = false;

  Future<void> _start() async {
    try {
      final caps = await _method.invokeMethod('getCapabilities');
      if (caps is Map) { _depthOk = (caps['depth'] == true); }
    } catch (_) { _depthOk = false; }
    await _method.invokeMethod('startScan');
    try {
      await _method.invokeMethod('setSegmentation', {
        'minDepthM': 0.02,
        'maxDepthM': 0.60,
        'removeGround': true,
        'clusterFoot': true,
        'trackingOnlyNormal': true,
      });
    } catch (_) {}
    _sub?.cancel();
    _sub = _events.receiveBroadcastStream().listen(_onEvent);
    _videoSub?.cancel();
    _videoSub = _eventsVideo.receiveBroadcastStream().listen(_onVideo);
    _watchdog?.cancel();
    _watchdog = Timer.periodic(const Duration(milliseconds: 1000), (_) {
      final now = DateTime.now().millisecondsSinceEpoch;
      if (_scanning && (_lastEventMs == 0 || now - _lastEventMs > 2000)) {
        if (mounted) {
          ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Tracking faible · Restez immobile')));
        }
      }
    });
    setState(() { _scanning = true; _fpsAvg = 0; _lastEventMs = 0; _startMs = DateTime.now().millisecondsSinceEpoch; _calibrated = false; _phase = ScanPhase.prep; _accumulate = false; _accum.clear(); _accumC.clear(); _coverage = 0.0; _dimsPrev = null; _doneTop = false; _doneMedial = false; _doneLateral = false; });
  }

  Future<void> _stop() async {
    await _method.invokeMethod('stopScan');
    await _sub?.cancel();
    _sub = null;
    await _videoSub?.cancel();
    _videoSub = null;
    _watchdog?.cancel();
    _watchdog = null;
    setState(() => _scanning = false);
  }

  void _onEvent(dynamic data) {
    if (data is Uint8List) {
      if (_phase != ScanPhase.scanning) {
        return;
      }
      final nowMs = DateTime.now().millisecondsSinceEpoch;
      if (_lastEventMs != 0) {
        final dt = nowMs - _lastEventMs;
        if (dt > 0 && dt < 1000) {
          final fps = 1000.0 / dt;
          _fpsAvg = _fpsAvg == 0 ? fps : (_fpsAvg * 0.8 + fps * 0.2);
        }
      }
      _lastEventMs = nowMs;
      final bd = ByteData.sublistView(data);
      if (bd.lengthInBytes < 4) return;
      int count = bd.getInt32(0, Endian.little);
      final maxCount = ((bd.lengthInBytes - 4) ~/ 12);
      if (count < 0 || count > maxCount) {
        count = maxCount;
        if (count <= 0) return;
      }
      final pts = <vmath.Vector3>[];
      int step = 1;
      if (count > 10000) { step = (count / 10000).ceil(); }
      for (int i = 0; i < count; i += step) {
        final base = 4 + i * 12;
        final x = bd.getFloat32(base, Endian.little);
        final y = bd.getFloat32(base + 4, Endian.little);
        final z = bd.getFloat32(base + 8, Endian.little);
        pts.add(vmath.Vector3(x, y, z));
      }
      if (_accumulate) {
        for (final p in pts) {
          if (_dimsPrev != null) {
            final d = _dimsPrev!;
            vmath.Vector3 c0 = (d.flB - d.flA).normalized();
            vmath.Vector3 c1 = (d.fbhB - d.fbhA).normalized();
            final c2 = vmath.Vector3(
              c0.y * c1.z - c0.z * c1.y,
              c0.z * c1.x - c0.x * c1.z,
              c0.x * c1.y - c0.y * c1.x,
            ).normalized();
            c1 = vmath.Vector3(
              c2.y * c0.z - c2.z * c0.y,
              c2.z * c0.x - c2.x * c0.z,
              c2.x * c0.y - c2.y * c0.x,
            ).normalized();
            final R = vmath.Matrix3.columns(c0, c1, c2);
            final anchors = [d.flA, d.flB, d.bflA, d.bflB, d.obflA, d.obflB, d.fbhA, d.fbhB, d.fbdA, d.fbdB, d.hbA, d.hbB, d.ihA, d.ihB];
            var mean = vmath.Vector3.zero();
            for (final a in anchors) { mean += a; }
            mean.scale(1.0 / anchors.length);
            double minX = double.infinity, maxX = -double.infinity;
            double minY = double.infinity, maxY = -double.infinity;
            double minZ = double.infinity, maxZ = -double.infinity;
            for (final a in anchors) {
              final r = R.transposed().transform(a - mean);
              if (r.x < minX) minX = r.x; if (r.x > maxX) maxX = r.x;
              if (r.y < minY) minY = r.y; if (r.y > maxY) maxY = r.y;
              if (r.z < minZ) minZ = r.z; if (r.z > maxZ) maxZ = r.z;
            }
            final rp = R.transposed().transform(p - mean);
            const m = 0.03;
            if (!(rp.x >= minX - m && rp.x <= maxX + m && rp.y >= minY - m && rp.y <= maxY + m && rp.z >= minZ - 0.04 && rp.z <= maxZ + 0.04)) {
              continue;
            }
          }
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
        if (_accum.length > 12000) {
          var m = vmath.Vector3.zero();
          for (final p in agg) { m += p; }
          if (agg.isNotEmpty) m.scale(1.0 / agg.length);
          final dist = <MapEntry<String, double>>[];
          _accum.forEach((k, p) { dist.add(MapEntry(k, (p - m).length)); });
          dist.sort((a,b)=>b.value.compareTo(a.value));
          final target = 10000;
          for (int i = 0; i < dist.length - target; i++) { _accum.remove(dist[i].key); _accumC.remove(dist[i].key); }
        }
        final now = DateTime.now().millisecondsSinceEpoch;
        _coverage = (_accum.length / _targetVox).clamp(0.0, 1.0);
        _checkZoneMilestones();
        final guide = _guideText();
        if (!_hudMinimal && _scanning && guide != _lastGuide) {
          _lastGuide = guide;
          if (mounted) {
            ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(guide == 'OK' ? 'Couverture OK' : 'Continuez: $guide')));
            SystemSound.play(SystemSoundType.click);
          }
        }
        if (_coverage >= _coverageNeeded()) {
          _onStopPressed();
        }
        if (!_calibrated && _scanning && (nowMs - _startMs) > 1500 && _fpsAvg > 0) {
          final tunedFps = _fpsAvg.clamp(18.0, 28.0);
          final defaults = _qualityDefaults(_quality);
          _method.invokeMethod('setQuality', {'targetFps': tunedFps, 'maxPoints': defaults.$2});
          _calibrated = true;
        }
        final sinceTune = nowMs - _lastTuneMs;
        if (_fpsAvg > 0 && sinceTune > 1000) {
          if (_fpsAvg < 16 && !_thermalDown) {
            _thermalDown = true; _lastTuneMs = nowMs;
            final defaults = _qualityDefaults(_quality);
            final mp = (defaults.$2 * 0.7).floor();
            final tf = defaults.$1 + 2.0;
            _method.invokeMethod('setQuality', {'targetFps': tf, 'maxPoints': mp});
          } else if (_fpsAvg > 22 && _thermalDown) {
            _thermalDown = false; _lastTuneMs = nowMs;
            final defaults = _qualityDefaults(_quality);
            _method.invokeMethod('setQuality', {'targetFps': defaults.$1, 'maxPoints': defaults.$2});
          }
        }
        if (now - _lastMetricsMs >= _metricsIntervalMs) {
          try {
            final m = computeMetrics(agg);
            final dRaw = computeSevenDimensions(agg);
            final d = _smoothDims(dRaw, _dimsPrev);
            _dimsPrev = d;
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
            final dRaw = computeSevenDimensions(pts);
            final d = _smoothDims(dRaw, _dimsPrev);
            _dimsPrev = d;
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
    Uint8List? bytes;
    int turnsHint = 0;
    if (data is Map) {
      final jpeg = data['jpeg'] ?? data['bytes'];
      if (jpeg is Uint8List) bytes = jpeg;
      final t = data['rotationQuarterTurns'] ?? data['turns'] ?? 0;
      if (t is int) turnsHint = t % 4;
    } else if (data is Uint8List) {
      bytes = data;
    }
    if (bytes == null) return;
    setState(() { _videoJpeg = bytes; });
    try {
      ui.decodeImageFromList(bytes, (img) {
        if (!mounted) return;
        int turns = turnsHint;
        if (turns == 0) {
          final size = MediaQuery.of(context).size;
          final isViewPortrait = size.height >= size.width;
          final isImgLandscape = img.width >= img.height;
          if (isViewPortrait && isImgLandscape) turns = 1;
          if (!isViewPortrait && !isImgLandscape) turns = 1;
        }
        setState(() { _videoRotTurns = turns; });
      });
    } catch (_) {}
  }

  @override
  void dispose() {
    _sub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Scanner Lidar'), actions: [
        IconButton(
          icon: const Icon(Icons.delete_outline),
          onPressed: () { setState(() { _accum.clear(); _accumC.clear(); _points = const []; _metrics = null; }); },
        ),
      ]),
      body: Stack(children: [
        Column(
          children: [
            Expanded(child: PointCloudView(points: _points, autoFit: true, dims: _dims, dimsShow: _showDim, backgroundJpeg: _videoJpeg, backgroundRotationQuarterTurns: _videoRotTurns)),
          ],
        ),
        Positioned(
          top: 8,
          right: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
            decoration: BoxDecoration(color: Colors.black54, borderRadius: BorderRadius.circular(12)),
            child: Text(
              '${_accumulate ? 'Accumulation' : 'Instantané'} · ${_points.length} pts · ${_fpsAvg.toStringAsFixed(0)} FPS',
              style: const TextStyle(color: Colors.white),
            ),
          ),
        ),
        if (!_depthOk) Positioned(
          top: 50,
          left: 8,
          right: 8,
          child: Container(
            padding: const EdgeInsets.all(8),
            decoration: BoxDecoration(color: Colors.redAccent, borderRadius: BorderRadius.circular(8)),
            child: const Text('Profondeur indisponible: appareil non compatible', style: TextStyle(color: Colors.white)),
          ),
        ),
        Positioned(
          top: 8,
          left: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
            decoration: BoxDecoration(color: Colors.black54, borderRadius: BorderRadius.circular(12)),
            child: Text(
              _phase == ScanPhase.prep ? 'Préparation' : (_phase == ScanPhase.scanning ? 'Scan' : 'Résultat'),
              style: const TextStyle(color: Colors.white),
            ),
          ),
        ),
        if (_phase == ScanPhase.scanning) Positioned(
          top: 8,
          left: 8,
          child: _hudMinimal
              ? Container(
                  padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(14),
                    color: Colors.black54,
                  ),
                  child: Row(mainAxisSize: MainAxisSize.min, children: [
                    SizedBox(
                      width: 180,
                      height: 6,
                      child: ClipRRect(
                        borderRadius: BorderRadius.circular(4),
                        child: LinearProgressIndicator(value: _coverage, backgroundColor: Colors.white24, color: Colors.lightGreenAccent),
                      ),
                    ),
                    const SizedBox(width: 10),
                    Row(children: [
                      _zoneChip('Dessus', _doneTop),
                      const SizedBox(width: 6),
                      _zoneChip('Médial', _doneMedial),
                      const SizedBox(width: 6),
                      _zoneChip('Latéral', _doneLateral),
                    ]),
                  ]),
                )
              : AnimatedSwitcher(
                  duration: const Duration(milliseconds: 250),
                  child: Container(
                    key: ValueKey(_guideText()),
                    padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
                    decoration: BoxDecoration(
                      borderRadius: BorderRadius.circular(14),
                      gradient: LinearGradient(colors: [Colors.black87, Colors.black54]),
                    ),
                    child: Row(
                      mainAxisSize: MainAxisSize.min,
                      children: [
                        Builder(builder: (_) {
                          final z = _zoneVisual(_guideText());
                          return CircleAvatar(backgroundColor: z.$2, child: Icon(z.$1, color: Colors.white));
                        }),
                        const SizedBox(width: 10),
                        Column(
                          mainAxisSize: MainAxisSize.min,
                          crossAxisAlignment: CrossAxisAlignment.start,
                          children: [
                            Text('Couverture ${(100*_coverage).toStringAsFixed(0)}% · ${_guideText()}', style: const TextStyle(color: Colors.white)),
                            const SizedBox(height: 6),
                            SizedBox(
                              width: 180,
                              height: 6,
                              child: ClipRRect(
                                borderRadius: BorderRadius.circular(4),
                                child: LinearProgressIndicator(value: _coverage, backgroundColor: Colors.white24, color: Colors.lightGreenAccent),
                              ),
                            ),
                          ],
                        ),
                      ],
                    ),
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
                  onPressed: () async {
                    if (!_scanning) { await _start(); }
                    else if (_phase == ScanPhase.prep) {
                      setState(() {
                        _accumulate = true;
                        _phase = ScanPhase.scanning;
                        _showDim['GRID'] = false;
                        _showDim['BOX'] = false;
                        _showDim['BOX_STRICT'] = false;
                        _showDim['FL'] = false; _showDim['BFL'] = false; _showDim['OBFL'] = false; _showDim['FBH'] = false; _showDim['FBD'] = false; _showDim['HB'] = false; _showDim['IH'] = false;
                      });
                    }
                    else if (_phase == ScanPhase.result) { await _start(); }
                  },
                  icon: const Icon(Icons.play_arrow),
                  label: Text(!_scanning ? 'Préparer' : (_phase == ScanPhase.prep ? 'Scanner' : 'Recommencer')),
                ),
              ),
              const SizedBox(width: 12),
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: (_phase == ScanPhase.scanning && _coverage >= _coverageNeeded()) || _phase == ScanPhase.result ? () async {
                    if (_phase == ScanPhase.scanning) {
                      await _onStopPressed();
                      setState(() {
                        _phase = ScanPhase.result;
                        _showDim['GRID'] = true;
                        _showDim['BOX'] = true; _showDim['BOX_STRICT'] = true;
                        _showDim['FL'] = true; _showDim['BFL'] = true; _showDim['OBFL'] = true; _showDim['FBH'] = true; _showDim['FBD'] = true; _showDim['HB'] = true; _showDim['IH'] = true;
                      });
                      await _showResultSheet();
                    }
                    else { await _exportPdf(); }
                  } : null,
                  icon: Icon(_phase == ScanPhase.result ? Icons.check : Icons.stop),
                  label: Text(_phase == ScanPhase.result ? 'Valider' : 'Terminer'),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  SevenDims? _smoothDims(SevenDims? cur, SevenDims? prev) {
    if (cur == null) return prev;
    if (prev == null) return cur;
    vmath.Vector3 lerp(vmath.Vector3 a, vmath.Vector3 b, double t) => vmath.Vector3(
      a.x * t + b.x * (1-t),
      a.y * t + b.y * (1-t),
      a.z * t + b.z * (1-t),
    );
    const t = 0.6; // favor current
    final flA = lerp(cur.flA, prev.flA, t);
    final flB = lerp(cur.flB, prev.flB, t);
    final bflA = lerp(cur.bflA, prev.bflA, t);
    final bflB = lerp(cur.bflB, prev.bflB, t);
    final obflA = lerp(cur.obflA, prev.obflA, t);
    final obflB = lerp(cur.obflB, prev.obflB, t);
    final fbhA = lerp(cur.fbhA, prev.fbhA, t);
    final fbhB = lerp(cur.fbhB, prev.fbhB, t);
    final fbdA = lerp(cur.fbdA, prev.fbdA, t);
    final fbdB = lerp(cur.fbdB, prev.fbdB, t);
    final hbA = lerp(cur.hbA, prev.hbA, t);
    final hbB = lerp(cur.hbB, prev.hbB, t);
    final ihA = lerp(cur.ihA, prev.ihA, t);
    final ihB = lerp(cur.ihB, prev.ihB, t);
    double dist(vmath.Vector3 a, vmath.Vector3 b) => (a - b).length;
    const cm = 100.0;
    return SevenDims(
      flCm: dist(flA, flB) * cm, flA: flA, flB: flB,
      bflCm: dist(bflA, bflB) * cm, bflA: bflA, bflB: bflB,
      obflCm: dist(obflA, obflB) * cm, obflA: obflA, obflB: obflB,
      fbhCm: dist(fbhA, fbhB) * cm, fbhA: fbhA, fbhB: fbhB,
      fbdCm: dist(fbdA, fbdB) * cm, fbdA: fbdA, fbdB: fbdB,
      hbCm: dist(hbA, hbB) * cm, hbA: hbA, hbB: hbB,
      ihCm: dist(ihA, ihB) * cm, ihA: ihA, ihB: ihB,
    );
  }

  String _guideText() {
    final p = _coverage;
    if (p < 0.3) return 'Vue dessus';
    if (p < 0.6) return 'Côté médial';
    if (p < 0.9) return 'Côté latéral';
    return 'OK';
  }

  (IconData, Color) _zoneVisual(String g) {
    if (g == 'Vue dessus') return (Icons.center_focus_strong, Colors.cyan);
    if (g == 'Côté médial') return (Icons.keyboard_arrow_left, Colors.orange);
    if (g == 'Côté latéral') return (Icons.keyboard_arrow_right, Colors.lightBlue);
    return (Icons.check_circle, Colors.lightGreen);
  }

  double _coverageNeeded() {
    final len = _metrics?.lengthCm ?? 20.0;
    final vol = _metrics?.volumeCm3 ?? 0.0;
    double t = 0.9;
    if (len < 22.0) { t = 0.85; } else if (len > 28.0) { t = 0.95; }
    if (vol > 3000.0) t = math.max(t, 0.93);
    return t;
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
    if (_coverage < _coverageNeeded()) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Couverture ${(100*_coverage).toStringAsFixed(0)}% insuffisante · Continuez: ${_guideText()}')));
      return;
    }
    if (_metrics == null) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Aucune métrique à exporter')));
      return;
    }
    final base = jsonDecode(metricsToJson(_metrics!));
    if (_dims != null) {
      base['seven_dimensions'] = sevenDimsToMap(_dims!);
      base['seven_dimensions_graph'] = sevenDimsSemanticGraph(_dims!);
    }
    final jsonStr = jsonEncode(base);
    final dir = await getApplicationDocumentsDirectory();
    final ts = DateTime.now().toIso8601String().replaceAll(':', '-');
    final file = File('${dir.path}/scan_$ts.json');
    await file.writeAsString(jsonStr);
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('JSON enregistré: ${file.path}')));
  }

  Future<void> _exportPdf() async {
    if (_coverage < _coverageNeeded()) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Couverture ${(100*_coverage).toStringAsFixed(0)}% insuffisante · Continuez: ${_guideText()}')));
      return;
    }
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

  

  

  

  (double, int) _qualityDefaults(String q) {
    if (q == 'detail' || q == 'ultra') return (18.0, 80000);
    if (q == 'balanced') return (22.0, 60000);
    return (26.0, 40000);
  }

  Future<void> _onStopPressed() async {
    await _stop();
    final agg = _accum.values.toList(growable: false);
    if (agg.isNotEmpty) {
      try {
        final m = computeMetrics(agg);
        final d = computeSevenDimensions(agg);
        setState(() { _metrics = m; _dims = d; _points = agg; });
      } catch (_) {
        setState(() { _points = agg; });
      }
    }
  }

  void _checkZoneMilestones() {
    if (!_doneTop && _coverage >= 0.30) {
      _doneTop = true;
      SystemSound.play(SystemSoundType.click);
    }
    if (!_doneMedial && _coverage >= 0.60) {
      _doneMedial = true;
      SystemSound.play(SystemSoundType.click);
    }
    if (!_doneLateral && _coverage >= 0.90) {
      _doneLateral = true;
      SystemSound.play(SystemSoundType.click);
    }
  }

  Widget _zoneChip(String label, bool done) {
    final c = done ? Colors.lightGreen : Colors.grey;
    final icon = done ? Icons.check : Icons.circle_outlined;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
      decoration: BoxDecoration(color: c.withValues(alpha: 0.25), borderRadius: BorderRadius.circular(12), border: Border.all(color: c)),
      child: Row(mainAxisSize: MainAxisSize.min, children: [Icon(icon, size: 14, color: c), const SizedBox(width: 4), Text(label, style: TextStyle(color: c, fontSize: 12))]),
    );
  }

  Future<void> _showResultSheet() async {
    if (!mounted) return;
    showModalBottomSheet(context: context, isScrollControlled: true, builder: (_) {
      return SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(16),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text('Résultat du scan', style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
              const SizedBox(height: 12),
              Wrap(spacing: 16, runSpacing: 12, children: [
                _metricTile('Longueur', _metrics?.lengthCm),
                _metricTile('Largeur', _metrics?.widthCm),
                _metricTile('Hauteur voûte', _metrics?.archHeightCm),
                _metricTile('Largeur avant-pied', _metrics?.forefootWidthCm),
                _metricTile('Largeur talon', _metrics?.heelWidthCm),
              ]),
              const SizedBox(height: 16),
              Row(children: [
                Expanded(child: ElevatedButton.icon(onPressed: _exportPdf, icon: const Icon(Icons.picture_as_pdf), label: const Text('Exporter PDF'))),
                const SizedBox(width: 12),
                Expanded(child: ElevatedButton.icon(onPressed: _exportJson, icon: const Icon(Icons.save_alt), label: const Text('Exporter JSON'))),
              ]),
            ],
          ),
        ),
      );
    });
  }
}
