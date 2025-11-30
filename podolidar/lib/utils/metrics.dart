import 'package:vector_math/vector_math_64.dart' as vmath;
import 'dart:math' as math;

class Metrics {
  final double lengthCm;
  final double widthCm;
  final double archHeightCm;
  Metrics({required this.lengthCm, required this.widthCm, required this.archHeightCm});
}

Metrics? computeMetrics(List<vmath.Vector3> pts) {
  if (pts.isEmpty) return null;
  final mean = _mean(pts);
  final cov = _covariance(pts, mean);
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
  for (final p in pts) {
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

  return Metrics(
    lengthCm: lengthM * 100.0,
    widthCm: widthM * 100.0,
    archHeightCm: archM * 100.0,
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
    // Find largest off-diagonal
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
