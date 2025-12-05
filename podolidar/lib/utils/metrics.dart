import 'package:vector_math/vector_math_64.dart' as vmath;
import 'dart:math' as math;
import 'dart:convert';
import 'dart:typed_data';
import 'package:pdf/widgets.dart' as pw;

class Metrics {
  final double lengthCm;
  final double widthCm;
  final double archHeightCm;
  final double heelAngleDeg;
  final double archIndex;
  final double forefootWidthCm;
  final double archCurvature;
  final double archForefootRatio;
  final double w25Cm;
  final double w50Cm;
  final double w75Cm;
  final double heelWidthCm;
  final double csiPercent;
  final double staheliRatio;
  final double clarkeAngleDeg;
  final double volumeCm3;
  final vmath.Vector3 m1Head;
  final vmath.Vector3 m5Head;
  final vmath.Vector3 navicular;
  final vmath.Vector3 heelCenter;
  Metrics({required this.lengthCm, required this.widthCm, required this.archHeightCm, required this.heelAngleDeg, required this.archIndex, required this.forefootWidthCm, required this.archCurvature, required this.archForefootRatio, required this.w25Cm, required this.w50Cm, required this.w75Cm, required this.heelWidthCm, required this.csiPercent, required this.staheliRatio, required this.clarkeAngleDeg, required this.volumeCm3, required this.m1Head, required this.m5Head, required this.navicular, required this.heelCenter});
}

Metrics? computeMetrics(List<vmath.Vector3> pts) {
  if (pts.isEmpty) return null;
  final footPts = _segmentFoot(pts);
  if (footPts.isEmpty) return null;
  final mean = _mean(footPts);
  final cov = _covariance(footPts, mean);
  final eig = _eigenDecomposition3x3(cov);
  final axes = eig.vectors;
  
  final indices = [0, 1, 2];
  indices.sort((a, b) => eig.values[b].compareTo(eig.values[a]));
  final c0 = vmath.Vector3(
    axes.entry(0, indices[0]),
    axes.entry(1, indices[0]),
    axes.entry(2, indices[0]),
  );
  final c1 = vmath.Vector3(
    axes.entry(0, indices[1]),
    axes.entry(1, indices[1]),
    axes.entry(2, indices[1]),
  );
  final c2 = vmath.Vector3(
    axes.entry(0, indices[2]),
    axes.entry(1, indices[2]),
    axes.entry(2, indices[2]),
  );
  final R = vmath.Matrix3.columns(c0, c1, c2);
  double minX = double.infinity, maxX = -double.infinity;
  double minY = double.infinity, maxY = -double.infinity;
  double minZ = double.infinity, maxZ = -double.infinity;

  final transformed = <vmath.Vector3>[];
  for (final p in footPts) {
    final q = p - mean;
    final r = R.transposed().transform(q);
    transformed.add(r);
    if (r.x < minX) minX = r.x; if (r.x > maxX) maxX = r.x;
    if (r.y < minY) minY = r.y; if (r.y > maxY) maxY = r.y;
    if (r.z < minZ) minZ = r.z; if (r.z > maxZ) maxZ = r.z;
  }
  final lengthM = (maxX - minX).abs();
  final widthM = (maxY - minY).abs();
  final heightM = (maxZ - minZ).abs();

  final midStart = minX + 0.3 * (maxX - minX);
  final midEnd = minX + 0.7 * (maxX - minX);
  double aMin = double.infinity, aMax = -double.infinity;
  for (final r in transformed) {
    if (r.x >= midStart && r.x <= midEnd) {
      if (r.z < aMin) aMin = r.z; if (r.z > aMax) aMax = r.z;
    }
  }
  final archM = (aMax.isFinite && aMin.isFinite) ? (aMax - aMin).abs() : heightM;
  final archIndex = (lengthM > 1e-6) ? (archM / lengthM) : 0.0;

  final heelAngle = _heelAngleDeg(footPts, mean, R, minX, maxX);
  final forefootWidth = _forefootWidth(transformed, minX, maxX);
  final archCurv = _archCurvature(transformed, minX, maxX);
  final archForefootRatio = (forefootWidth > 1e-6) ? (archM / forefootWidth) : 0.0;
  final w25 = _widthAt(transformed, minX, maxX, 0.25);
  final w50 = _widthAt(transformed, minX, maxX, 0.50);
  final w75 = _widthAt(transformed, minX, maxX, 0.75);
  final heelW = _heelWidth(transformed, minX, maxX);
  final csi = (forefootWidth > 1e-6) ? (w50 / forefootWidth * 100.0) : 0.0;
  final staheli = (heelW > 1e-6) ? (w50 / heelW) : 0.0;
  final clarke = _clarkeAngle(transformed, minX, maxX);
  final vol = _voxelVolume(transformed, minX, maxX);
  final lm = _detectLandmarks(footPts, mean, R, minX, maxX, minY, maxY);

  return Metrics(
    lengthCm: lengthM * 100.0,
    widthCm: widthM * 100.0,
    archHeightCm: archM * 100.0,
    heelAngleDeg: heelAngle,
    archIndex: archIndex,
    forefootWidthCm: forefootWidth * 100.0,
    archCurvature: archCurv,
    archForefootRatio: archForefootRatio,
    w25Cm: w25 * 100.0,
    w50Cm: w50 * 100.0,
    w75Cm: w75 * 100.0,
    heelWidthCm: heelW * 100.0,
    csiPercent: csi,
    staheliRatio: staheli,
    clarkeAngleDeg: clarke,
    volumeCm3: vol,
    m1Head: lm.m1,
    m5Head: lm.m5,
    navicular: lm.nav,
    heelCenter: lm.heel,
  );
}

vmath.Vector3 _mean(List<vmath.Vector3> pts) {
  final m = vmath.Vector3.zero();
  for (final p in pts) { m.x += p.x; m.y += p.y; m.z += p.z; }
  m.scale(1.0 / pts.length);
  return m;
}

vmath.Matrix3 _covariance(List<vmath.Vector3> pts, vmath.Vector3 mean) {
  double cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
  for (final p in pts) {
    final x = p.x - mean.x;
    final y = p.y - mean.y;
    final z = p.z - mean.z;
    cxx += x * x; cxy += x * y; cxz += x * z; cyy += y * y; cyz += y * z; czz += z * z;
  }
  final invN = 1.0 / pts.length;
  cxx *= invN; cxy *= invN; cxz *= invN; cyy *= invN; cyz *= invN; czz *= invN;
  return vmath.Matrix3(
    cxx, cxy, cxz,
    cxy, cyy, cyz,
    cxz, cyz, czz,
  );
}

class Eigen3 {
  final List<double> values;
  final vmath.Matrix3 vectors;
  Eigen3(this.values, this.vectors);
}

Eigen3 _eigenDecomposition3x3(vmath.Matrix3 m) {
  final a = [
    [m.entry(0,0), m.entry(0,1), m.entry(0,2)],
    [m.entry(1,0), m.entry(1,1), m.entry(1,2)],
    [m.entry(2,0), m.entry(2,1), m.entry(2,2)],
  ];
  final v = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
  ];
  for (int iter = 0; iter < 24; iter++) {
    int p = 0, q = 1;
    double max = (a[0][1]).abs();
    void updateMax(int i,int j){ final val=(a[i][j]).abs(); if(val>max){max=val;p=i;q=j;} }
    updateMax(0,2); updateMax(1,2);
    if (max < 1e-9) break;
    final app = a[p][p]; final aqq = a[q][q]; final apq = a[p][q];
    final phi = 0.5 * (aqq - app) / apq;
    var t = 1.0 / (phi.abs() + math.sqrt(phi*phi + 1));
    if (phi < 0) { t = -t; }
    final c = 1.0 / math.sqrt(1 + t*t);
    final s = t * c;
    // Rotate A
    for (int k = 0; k < 3; k++) {
      final apk = a[p][k]; final aqk = a[q][k];
      a[p][k] = c*apk - s*aqk;
      a[q][k] = s*apk + c*aqk;
    }
    for (int k = 0; k < 3; k++) {
      final akp = a[k][p]; final akq = a[k][q];
      a[k][p] = c*akp - s*akq;
      a[k][q] = s*akp + c*akq;
    }
    a[p][q] = 0.0; a[q][p] = 0.0;
    // Rotate V
    for (int k = 0; k < 3; k++) {
      final vkp = v[k][p]; final vkq = v[k][q];
      v[k][p] = c*vkp - s*vkq;
      v[k][q] = s*vkp + c*vkq;
    }
  }
  final values = [a[0][0], a[1][1], a[2][2]];
  final vectors = vmath.Matrix3(
    v[0][0], v[0][1], v[0][2],
    v[1][0], v[1][1], v[1][2],
    v[2][0], v[2][1], v[2][2],
  );
  return Eigen3(values, vectors);
}

List<vmath.Vector3> _segmentFoot(List<vmath.Vector3> pts) {
  if (pts.length < 50) return pts;
  final plane = _adaptivePlane(pts);
  final n = plane[0];
  final d = plane[1];
  final thr = plane[2];
  final nonPlane = <vmath.Vector3>[];
  for (final p in pts) {
    final dist = (n.dot(p) + d).abs();
    if (dist >= thr) nonPlane.add(p);
  }
  if (nonPlane.isEmpty) return pts;
  final clustered = _dbscan(nonPlane, 0.03, 15);
  if (clustered.isEmpty) return nonPlane;
  clustered.sort((a, b) => b.length.compareTo(a.length));
  final mainCluster = clustered.first;
  final m = _mean(mainCluster);
  final filtered = <vmath.Vector3>[];
  final dists = <double>[];
  for (final p in mainCluster) { dists.add((p - m).length); }
  dists.sort();
  final med = dists[dists.length ~/ 2];
  double mad = 0.0; for (final d in dists) { mad += (d - med).abs(); }
  mad /= dists.length;
  final thrRad = math.max(0.12, med + 3.0 * mad);
  for (final p in mainCluster) {
    if ((p - m).length <= thrRad) filtered.add(p);
  }
  return filtered.isNotEmpty ? filtered : mainCluster;
}

double _heelAngleDeg(List<vmath.Vector3> pts, vmath.Vector3 mean, vmath.Matrix3 R, double minX, double maxX) {
  final heelPtsWorld = <vmath.Vector3>[];
  for (final p in pts) {
    final r = R.transposed().transform(p - mean);
    if (r.x <= minX + 0.2 * (maxX - minX)) heelPtsWorld.add(p);
  }
  if (heelPtsWorld.length < 20) return 0.0;
  final m = _mean(heelPtsWorld);
  final C = _covariance(heelPtsWorld, m);
  final e = _eigenDecomposition3x3(C);
  int minIdx = 0;
  if (e.values[1] < e.values[minIdx]) minIdx = 1;
  if (e.values[2] < e.values[minIdx]) minIdx = 2;
  final n = vmath.Vector3(e.vectors.entry(0, minIdx), e.vectors.entry(1, minIdx), e.vectors.entry(2, minIdx)).normalized();
  final up = vmath.Vector3(0,1,0);
  final cosang = n.dot(up).abs().clamp(0.0, 1.0);
  final ang = math.acos(cosang) * 180.0 / math.pi;
  return ang;
}

List<List<vmath.Vector3>> _dbscan(List<vmath.Vector3> pts, double eps, int minPts) {
  final n = pts.length;
  final visited = List<bool>.filled(n, false);
  final assigned = List<bool>.filled(n, false);
  final clusters = <List<vmath.Vector3>>[];
  for (int i = 0; i < n; i++) {
    if (visited[i]) continue;
    visited[i] = true;
    final neighbors = _regionQuery(pts, i, eps);
    if (neighbors.length < minPts) {
      continue;
    }
    final cluster = <vmath.Vector3>[];
    cluster.add(pts[i]);
    assigned[i] = true;
    int k = 0;
    while (k < neighbors.length) {
      final j = neighbors[k];
      if (!visited[j]) {
        visited[j] = true;
        final neighbors2 = _regionQuery(pts, j, eps);
        if (neighbors2.length >= minPts) {
          neighbors.addAll(neighbors2);
        }
      }
      if (!assigned[j]) {
        cluster.add(pts[j]);
        assigned[j] = true;
      }
      k++;
    }
    clusters.add(cluster);
  }
  return clusters;
}

List<int> _regionQuery(List<vmath.Vector3> pts, int idx, double eps) {
  final p = pts[idx];
  final res = <int>[];
  final e2 = eps * eps;
  for (int i = 0; i < pts.length; i++) {
    if (i == idx) continue;
    final d2 = (pts[i] - p).length2;
    if (d2 <= e2) res.add(i);
  }
  return res;
}

List<double> _profileZ(List<vmath.Vector3> transformed, double minX, double maxX, int bins) {
  final step = (maxX - minX) / bins;
  final zvals = List<List<double>>.generate(bins, (_) => <double>[]);
  for (final r in transformed) {
    final bi = ((r.x - minX) / step).floor();
    if (bi >= 0 && bi < bins) zvals[bi].add(r.z);
  }
  final profile = List<double>.filled(bins, 0.0);
  for (int i = 0; i < bins; i++) {
    if (zvals[i].isEmpty) { profile[i] = 0.0; continue; }
    zvals[i].sort();
    final m = zvals[i].length;
    profile[i] = zvals[i][m ~/ 2];
  }
  return profile;
}

double _archCurvature(List<vmath.Vector3> transformed, double minX, double maxX) {
  final bins = 32;
  var z = _profileZ(transformed, minX, maxX, bins);
  z = _savgol5(z);
  final dx = (maxX - minX) / bins;
  double maxK = 0.0;
  for (int i = 1; i < bins - 1; i++) {
    final d2 = (z[i+1] - 2*z[i] + z[i-1]) / (dx*dx);
    final k = d2.abs();
    if (k > maxK) maxK = k;
  }
  return maxK * 100.0;
}

List<double> _savgol5(List<double> x) {
  if (x.length < 5) return x;
  final y = List<double>.filled(x.length, 0.0);
  for (int i = 0; i < x.length; i++) {
    double s = 0.0;
    for (int k = -2; k <= 2; k++) {
      int j = i + k;
      if (j < 0) j = 0;
      if (j >= x.length) j = x.length - 1;
      final c = (k == -2 || k == 2) ? -3.0 : (k == -1 || k == 1) ? 12.0 : 17.0;
      s += c * x[j];
    }
    y[i] = s / 35.0;
  }
  return y;
}

double _forefootWidth(List<vmath.Vector3> transformed, double minX, double maxX) {
  final frontStart = minX + 0.8 * (maxX - minX);
  double minY = double.infinity, maxY = -double.infinity;
  for (final r in transformed) {
    if (r.x >= frontStart) {
      if (r.y < minY) minY = r.y;
      if (r.y > maxY) maxY = r.y;
    }
  }
  if (!minY.isFinite || !maxY.isFinite) return 0.0;
  return (maxY - minY).abs();
}

List<dynamic> _adaptivePlane(List<vmath.Vector3> pts) {
  final rnd = math.Random(12345);
  vmath.Vector3? bestN;
  double bestD = 0.0;
  double bestScore = double.infinity;
  for (int it = 0; it < 100; it++) {
    final p1 = pts[rnd.nextInt(pts.length)];
    final p2 = pts[rnd.nextInt(pts.length)];
    final p3 = pts[rnd.nextInt(pts.length)];
    final n = (p2 - p1).cross(p3 - p1).normalized();
    if (n.length < 1e-6) continue;
    final d = -n.dot(p1);
    double sum = 0.0;
    for (final p in pts) { sum += (n.dot(p) + d).abs(); }
    final score = sum / pts.length;
    if (score < bestScore) { bestScore = score; bestN = n; bestD = d; }
  }
  if (bestN == null) return [vmath.Vector3(0,1,0), 0.0, 0.01];
  final res = <double>[];
  for (final p in pts) { res.add((bestN.dot(p) + bestD).abs()); }
  res.sort();
  final med = res[res.length ~/ 2];
  double mad = 0.0;
  for (final r in res) { mad += (r - med).abs(); }
  mad /= res.length;
  final thr = math.max(0.008, 3.0 * mad);
  return [bestN, bestD, thr];
}
double _widthAt(List<vmath.Vector3> transformed, double minX, double maxX, double t) {
  final x0 = minX + t * (maxX - minX);
  final w = 0.05 * (maxX - minX);
  double minY = double.infinity, maxY = -double.infinity;
  for (final r in transformed) {
    if ((r.x - x0).abs() <= w) {
      if (r.y < minY) minY = r.y;
      if (r.y > maxY) maxY = r.y;
    }
  }
  if (!minY.isFinite || !maxY.isFinite) return 0.0;
  return (maxY - minY).abs();
}

double _heelWidth(List<vmath.Vector3> transformed, double minX, double maxX) {
  final rearEnd = minX + 0.2 * (maxX - minX);
  double minY = double.infinity, maxY = -double.infinity;
  for (final r in transformed) {
    if (r.x <= rearEnd) {
      if (r.y < minY) minY = r.y;
      if (r.y > maxY) maxY = r.y;
    }
  }
  if (!minY.isFinite || !maxY.isFinite) return 0.0;
  return (maxY - minY).abs();
}

double _clarkeAngle(List<vmath.Vector3> transformed, double minX, double maxX) {
  final bins = 32;
  var z = _profileZ(transformed, minX, maxX, bins);
  z = _savgol5(z);
  final dx = (maxX - minX) / bins;
  int start = (bins * 0.25).floor();
  int end = (bins * 0.75).floor();
  int idx = start;
  double minVal = double.infinity;
  for (int i = start; i < end; i++) { if (z[i] < minVal) { minVal = z[i]; idx = i; } }
  double slope = 0.0;
  if (idx > 0 && idx < bins - 1) { slope = (z[idx+1] - z[idx-1]) / (2*dx); }
  return math.atan(slope.abs()) * 180.0 / math.pi;
}

double _voxelVolume(List<vmath.Vector3> transformed, double minX, double maxX) {
  double minY = double.infinity, maxY = -double.infinity;
  for (final r in transformed) { if (r.y < minY) minY = r.y; if (r.y > maxY) maxY = r.y; }
  if (!minY.isFinite || !maxY.isFinite) return 0.0;
  final s = 0.01;
  final map = <String, List<double>>{};
  for (final r in transformed) {
    final xi = ((r.x - minX) / s).floor();
    final yi = ((r.y - minY) / s).floor();
    final key = "$xi:$yi";
    final v = map[key];
    if (v == null) { map[key] = [r.z, r.z, 1]; } else { if (r.z < v[0]) v[0] = r.z; if (r.z > v[1]) v[1] = r.z; v[2] += 1; }
  }
  double vol = 0.0;
  for (final e in map.entries) {
    final v = e.value;
    if (v[2] >= 4) { final h = (v[1] - v[0]).abs(); vol += h * s * s; }
  }
  return vol * 1e6;
}
class Landmarks {
  final vmath.Vector3 m1;
  final vmath.Vector3 m5;
  final vmath.Vector3 nav;
  final vmath.Vector3 heel;
  Landmarks(this.m1, this.m5, this.nav, this.heel);
}

Landmarks _detectLandmarks(List<vmath.Vector3> ptsWorld, vmath.Vector3 mean, vmath.Matrix3 R, double minX, double maxX, double minY, double maxY) {
  final transformed = <vmath.Vector3>[];
  for (final p in ptsWorld) {
    transformed.add(R.transposed().transform(p - mean));
  }
  final frontStart = minX + 0.85 * (maxX - minX);
  final midStart = minX + 0.4 * (maxX - minX);
  final midEnd = minX + 0.6 * (maxX - minX);
  final rearEnd = minX + 0.15 * (maxX - minX);
  final yMed = (minY + maxY) * 0.5;

  vmath.Vector3? m1r;
  vmath.Vector3? m5r;
  double m1z = -double.infinity;
  double m5z = -double.infinity;
  for (int i = 0; i < transformed.length; i++) {
    final r = transformed[i];
    if (r.x >= frontStart) {
      if (r.y <= yMed && r.z > m1z) { m1z = r.z; m1r = r; }
      if (r.y >= yMed && r.z > m5z) { m5z = r.z; m5r = r; }
    }
  }
  vmath.Vector3? navr;
  double navz = -double.infinity;
  for (final r in transformed) {
    if (r.x >= midStart && r.x <= midEnd && r.y <= yMed) {
      if (r.z > navz) { navz = r.z; navr = r; }
    }
  }
  vmath.Vector3 heelt = vmath.Vector3.zero();
  int heelCount = 0;
  for (final r in transformed) {
    if (r.x <= rearEnd) { heelt += r; heelCount++; }
  }
  if (heelCount > 0) heelt.scale(1.0 / heelCount);

  final m1 = mean + R.transform(m1r ?? vmath.Vector3(frontStart, minY, 0));
  final m5 = mean + R.transform(m5r ?? vmath.Vector3(frontStart, maxY, 0));
  final nav = mean + R.transform(navr ?? vmath.Vector3((midStart+midEnd)*0.5, yMed, 0));
  final heel = mean + R.transform(heelt);
  return Landmarks(m1, m5, nav, heel);
}

String metricsToJson(Metrics m) {
  final map = {
    'length_cm': m.lengthCm,
    'width_cm': m.widthCm,
    'arch_height_cm': m.archHeightCm,
    'heel_angle_deg': m.heelAngleDeg,
    'arch_index_percent': m.archIndex * 100.0,
    'forefoot_width_cm': m.forefootWidthCm,
    'arch_curvature_1_per_cm': m.archCurvature,
    'arch_forefoot_ratio': m.archForefootRatio,
    'width_25_cm': m.w25Cm,
    'width_50_cm': m.w50Cm,
    'width_75_cm': m.w75Cm,
    'heel_width_cm': m.heelWidthCm,
    'csi_percent': m.csiPercent,
    'staheli_ratio': m.staheliRatio,
    'clarke_angle_deg': m.clarkeAngleDeg,
    'volume_cm3': m.volumeCm3,
    'landmarks': {
      'm1': [m.m1Head.x, m.m1Head.y, m.m1Head.z],
      'm5': [m.m5Head.x, m.m5Head.y, m.m5Head.z],
      'navicular': [m.navicular.x, m.navicular.y, m.navicular.z],
      'heel_center': [m.heelCenter.x, m.heelCenter.y, m.heelCenter.z],
    }
  };
  return jsonEncode(map);
}

Future<Uint8List> generatePdfReport(Metrics m) async {
  final pdf = pw.Document();
  pdf.addPage(
    pw.Page(
      build: (context) => pw.Column(
        crossAxisAlignment: pw.CrossAxisAlignment.start,
        children: [
          pw.Text('Rapport Podologie - Scan LiDAR', style: pw.TextStyle(fontSize: 18)),
          pw.SizedBox(height: 12),
          pw.Text('Longueur: ${m.lengthCm.toStringAsFixed(1)} cm'),
          pw.Text('Largeur: ${m.widthCm.toStringAsFixed(1)} cm'),
          pw.Text('Hauteur voûte: ${m.archHeightCm.toStringAsFixed(1)} cm'),
          pw.Text('Angle talon: ${m.heelAngleDeg.toStringAsFixed(1)} °'),
          pw.Text('Indice voûte: ${(m.archIndex*100).toStringAsFixed(1)} %'),
          pw.Text('Largeur avant-pied: ${m.forefootWidthCm.toStringAsFixed(1)} cm'),
          pw.Text('Courbure voûte: ${m.archCurvature.toStringAsFixed(3)} 1/cm'),
          pw.Text('Ratio voûte/avant-pied: ${m.archForefootRatio.toStringAsFixed(3)}'),
          pw.SizedBox(height: 6),
          pw.Text('Largeurs 25/50/75%: ${m.w25Cm.toStringAsFixed(1)} / ${m.w50Cm.toStringAsFixed(1)} / ${m.w75Cm.toStringAsFixed(1)} cm'),
          pw.Text('Largeur talon: ${m.heelWidthCm.toStringAsFixed(1)} cm'),
          pw.Text('Chippaux-Smirak: ${m.csiPercent.toStringAsFixed(1)} %'),
          pw.Text('Staheli ratio: ${m.staheliRatio.toStringAsFixed(3)}'),
          pw.Text('Clarke angle: ${m.clarkeAngleDeg.toStringAsFixed(1)} °'),
          pw.Text('Volume: ${m.volumeCm3.toStringAsFixed(1)} cm³'),
          pw.SizedBox(height: 12),
          pw.Text('Landmarks (x,y,z):'),
          pw.Text('M1: ${m.m1Head.x.toStringAsFixed(2)}, ${m.m1Head.y.toStringAsFixed(2)}, ${m.m1Head.z.toStringAsFixed(2)}'),
          pw.Text('M5: ${m.m5Head.x.toStringAsFixed(2)}, ${m.m5Head.y.toStringAsFixed(2)}, ${m.m5Head.z.toStringAsFixed(2)}'),
          pw.Text('Naviculaire: ${m.navicular.x.toStringAsFixed(2)}, ${m.navicular.y.toStringAsFixed(2)}, ${m.navicular.z.toStringAsFixed(2)}'),
          pw.Text('Talon: ${m.heelCenter.x.toStringAsFixed(2)}, ${m.heelCenter.y.toStringAsFixed(2)}, ${m.heelCenter.z.toStringAsFixed(2)}'),
        ],
      ),
    ),
  );
  return await pdf.save();
}
