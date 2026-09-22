// Copyright 1996-2024 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "WbMeshCore.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <istream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace wbmesh {

static const double EPS = 1e-12;

double Vec3::length() const {
  return std::sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::normalized() const {
  const double l = length();
  if (l < EPS)
    return Vec3(0.0, 0.0, 0.0);
  return Vec3(x / l, y / l, z / l);
}

double Vec3::distance(const Vec3 &o) const {
  return (*this - o).length();
}

bool Vec3::isZero(double epsilon) const {
  return length() <= epsilon;
}

Mat3 Mat3::identity() {
  Mat3 r;
  std::memset(r.m, 0, sizeof(r.m));
  r.m[0] = r.m[4] = r.m[8] = 1.0;
  return r;
}

Mat3 Mat3::fromAxisAngle(const Vec3 &axis, double angle) {
  const Vec3 a = axis.normalized();
  const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
  Mat3 r;
  r.m[0] = t * a.x * a.x + c;
  r.m[1] = t * a.x * a.y - s * a.z;
  r.m[2] = t * a.x * a.z + s * a.y;
  r.m[3] = t * a.x * a.y + s * a.z;
  r.m[4] = t * a.y * a.y + c;
  r.m[5] = t * a.y * a.z - s * a.x;
  r.m[6] = t * a.x * a.z - s * a.y;
  r.m[7] = t * a.y * a.z + s * a.x;
  r.m[8] = t * a.z * a.z + c;
  return r;
}

Mat3 Mat3::transposed() const {
  Mat3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r.m[i * 3 + j] = m[j * 3 + i];
  return r;
}

Vec3 Mat3::mul(const Vec3 &v) const {
  return Vec3(m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
              m[6] * v.x + m[7] * v.y + m[8] * v.z);
}

Mat3 Mat3::mul(const Mat3 &o) const {
  Mat3 r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      double s = 0.0;
      for (int k = 0; k < 3; ++k)
        s += m[i * 3 + k] * o.m[k * 3 + j];
      r.m[i * 3 + j] = s;
    }
  return r;
}

void toAxisAngle(const Mat3 &r, Vec3 &axis, double &angle) {
  const double trace = r.m[0] + r.m[4] + r.m[8];
  const double c = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
  angle = std::acos(c);
  if (angle < 1e-9) {
    axis = Vec3(0.0, 0.0, 1.0);
    angle = 0.0;
    return;
  }
  if (M_PI - angle < 1e-6) {
    // near pi: axis from the diagonal of R + I
    double x = std::sqrt(std::max(0.0, (r.m[0] + 1.0) * 0.5));
    double y = std::sqrt(std::max(0.0, (r.m[4] + 1.0) * 0.5));
    double z = std::sqrt(std::max(0.0, (r.m[8] + 1.0) * 0.5));
    if (r.m[1] < 0.0)
      y = -y;
    if (r.m[2] < 0.0)
      z = -z;
    axis = Vec3(x, y, z).normalized();
    return;
  }
  axis = Vec3(r.m[7] - r.m[5], r.m[2] - r.m[6], r.m[3] - r.m[1]).normalized();
}

void Mesh::computeBounds() {
  boundsMin = Vec3(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                   std::numeric_limits<double>::max());
  boundsMax = Vec3(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
                   -std::numeric_limits<double>::max());
  for (size_t i = 0; i < positions.size(); ++i) {
    const Vec3 &p = positions[i];
    boundsMin.x = std::min(boundsMin.x, p.x);
    boundsMin.y = std::min(boundsMin.y, p.y);
    boundsMin.z = std::min(boundsMin.z, p.z);
    boundsMax.x = std::max(boundsMax.x, p.x);
    boundsMax.y = std::max(boundsMax.y, p.y);
    boundsMax.z = std::max(boundsMax.z, p.z);
  }
  if (positions.empty())
    boundsMin = boundsMax = Vec3();
}

void weldSoup(const std::vector<Vec3> &soup, Mesh &out, double epsilon) {
  out.positions.clear();
  out.triangles.clear();
  const double e = std::max(epsilon, 1e-12);
  std::map<std::tuple<long, long, long>, int> index;
  for (size_t i = 0; i + 2 < soup.size(); i += 3) {
    int indices[3];
    for (int k = 0; k < 3; ++k) {
      const Vec3 &p = soup[i + k];
      const std::tuple<long, long, long> key((long)std::floor(p.x / e + 0.5), (long)std::floor(p.y / e + 0.5),
                                             (long)std::floor(p.z / e + 0.5));
      std::map<std::tuple<long, long, long>, int>::iterator it = index.find(key);
      if (it == index.end()) {
        const int id = (int)out.positions.size();
        index[key] = id;
        out.positions.push_back(p);
        indices[k] = id;
      } else
        indices[k] = it->second;
    }
    // skip degenerate triangles
    if (indices[0] != indices[1] && indices[1] != indices[2] && indices[0] != indices[2]) {
      out.triangles.push_back(indices[0]);
      out.triangles.push_back(indices[1]);
      out.triangles.push_back(indices[2]);
    }
  }
  out.computeBounds();
}

bool parseStlAscii(std::istream &in, Mesh &out, std::string &error) {
  std::vector<Vec3> soup;
  std::string token;
  while (in >> token) {
    if (token == "vertex") {
      Vec3 p;
      if (!(in >> p.x >> p.y >> p.z)) {
        error = "malformed STL vertex";
        return false;
      }
      soup.push_back(p);
    }
  }
  if (soup.empty() || soup.size() % 3 != 0) {
    error = "no complete facet found in ASCII STL";
    return false;
  }
  weldSoup(soup, out);
  return true;
}

static unsigned int readU32(const char *data) {
  unsigned int v;
  std::memcpy(&v, data, 4);
  return v;
}

static float readF32(const char *data) {
  float v;
  std::memcpy(&v, data, 4);
  return v;
}

bool parseStlBinary(const char *data, size_t size, Mesh &out, std::string &error) {
  if (size < 84) {
    error = "binary STL too small";
    return false;
  }
  const unsigned int count = readU32(data + 80);
  if (size < 84 + (size_t)count * 50) {
    error = "binary STL truncated";
    return false;
  }
  std::vector<Vec3> soup;
  soup.reserve((size_t)count * 3);
  const char *p = data + 84;
  for (unsigned int i = 0; i < count; ++i, p += 50) {
    // 12 floats: facet normal + 3 vertices (normals are ignored)
    for (int k = 0; k < 3; ++k) {
      const char *v = p + 12 + k * 12;
      soup.push_back(Vec3(readF32(v), readF32(v + 4), readF32(v + 8)));
    }
  }
  if (soup.empty()) {
    error = "empty binary STL";
    return false;
  }
  weldSoup(soup, out);
  return true;
}

bool parseObj(std::istream &in, Mesh &out, std::string &error) {
  out.positions.clear();
  out.triangles.clear();
  std::string line;
  while (std::getline(in, line)) {
    std::istringstream ls(line);
    std::string tag;
    ls >> tag;
    if (tag == "v") {
      Vec3 p;
      if (!(ls >> p.x >> p.y >> p.z)) {
        error = "malformed OBJ vertex";
        return false;
      }
      out.positions.push_back(p);
    } else if (tag == "f") {
      std::vector<int> face;
      std::string ref;
      while (ls >> ref) {
        // formats: "i", "i/t", "i/t/n", "i//n"; negative indices are relative
        const size_t slash = ref.find('/');
        const std::string indexStr = slash == std::string::npos ? ref : ref.substr(0, slash);
        int index = atoi(indexStr.c_str());
        if (index < 0)
          index = (int)out.positions.size() + index + 1;
        if (index < 1 || index > (int)out.positions.size()) {
          error = "OBJ face index out of range";
          return false;
        }
        face.push_back(index - 1);
      }
      if (face.size() < 3) {
        error = "OBJ face with less than 3 vertices";
        return false;
      }
      for (size_t k = 1; k + 1 < face.size(); ++k) {  // fan triangulation
        out.triangles.push_back(face[0]);
        out.triangles.push_back(face[k]);
        out.triangles.push_back(face[k + 1]);
      }
    }
  }
  if (out.triangles.empty()) {
    error = "no face found in OBJ";
    return false;
  }
  out.computeBounds();
  return true;
}

static bool readWholeFile(const std::string &path, std::string &data, std::string &error) {
  std::ifstream file(path.c_str(), std::ios::in | std::ios::binary);
  if (!file.is_open()) {
    error = "cannot open file: " + path;
    return false;
  }
  std::ostringstream ss;
  ss << file.rdbuf();
  data = ss.str();
  return true;
}

bool loadStl(const std::string &path, Mesh &out, std::string &error) {
  std::string data;
  if (!readWholeFile(path, data, error))
    return false;
  // decide binary vs ascii: binary files have an exact 84 + 50 * count size
  bool binary = false;
  if (data.size() >= 84) {
    const unsigned int count = readU32(data.data() + 80);
    binary = data.size() == 84 + (size_t)count * 50;
  }
  if (binary)
    return parseStlBinary(data.data(), data.size(), out, error);
  std::istringstream in(data);
  return parseStlAscii(in, out, error);
}

bool loadObj(const std::string &path, Mesh &out, std::string &error) {
  std::ifstream file(path.c_str());
  if (!file.is_open()) {
    error = "cannot open file: " + path;
    return false;
  }
  return parseObj(file, out, error);
}

bool loadMeshFile(const std::string &path, Mesh &out, std::string &error) {
  const size_t dot = path.find_last_of('.');
  const std::string ext = dot == std::string::npos ? std::string() : path.substr(dot);
  std::string lower;
  for (size_t i = 0; i < ext.size(); ++i)
    lower += (char)tolower(ext[i]);
  if (lower == ".stl")
    return loadStl(path, out, error);
  if (lower == ".obj")
    return loadObj(path, out, error);
  error = "unsupported mesh format (expected .stl or .obj): " + path;
  return false;
}

std::vector<std::pair<int, int> > computeEdges(const Mesh &mesh) {
  std::set<std::pair<int, int> > edges;
  for (size_t i = 0; i + 2 < mesh.triangles.size(); i += 3) {
    const int v[3] = {mesh.triangles[i], mesh.triangles[i + 1], mesh.triangles[i + 2]};
    for (int k = 0; k < 3; ++k) {
      const int a = v[k], b = v[(k + 1) % 3];
      edges.insert(a < b ? std::make_pair(a, b) : std::make_pair(b, a));
    }
  }
  return std::vector<std::pair<int, int> >(edges.begin(), edges.end());
}

Vec3 faceNormal(const Mesh &mesh, int triangleIndex) {
  const int i = triangleIndex * 3;
  if (i < 0 || i + 2 >= (int)mesh.triangles.size())
    return Vec3();
  const Vec3 &a = mesh.positions[mesh.triangles[i]];
  const Vec3 &b = mesh.positions[mesh.triangles[i + 1]];
  const Vec3 &c = mesh.positions[mesh.triangles[i + 2]];
  return (b - a).cross(c - a).normalized();
}

Vec3 faceCentroid(const Mesh &mesh, int triangleIndex) {
  const int i = triangleIndex * 3;
  if (i < 0 || i + 2 >= (int)mesh.triangles.size())
    return Vec3();
  const Vec3 &a = mesh.positions[mesh.triangles[i]];
  const Vec3 &b = mesh.positions[mesh.triangles[i + 1]];
  const Vec3 &c = mesh.positions[mesh.triangles[i + 2]];
  return (a + b + c) * (1.0 / 3.0);
}

bool rayTriangle(const Vec3 &origin, const Vec3 &direction, const Vec3 &a, const Vec3 &b, const Vec3 &c, double &t) {
  const Vec3 e1 = b - a;
  const Vec3 e2 = c - a;
  const Vec3 p = direction.cross(e2);
  const double det = e1.dot(p);
  if (std::fabs(det) < EPS)
    return false;
  const double invDet = 1.0 / det;
  const Vec3 s = origin - a;
  const double u = s.dot(p) * invDet;
  if (u < -1e-9 || u > 1.0 + 1e-9)
    return false;
  const Vec3 q = s.cross(e1);
  const double v = direction.dot(q) * invDet;
  if (v < -1e-9 || u + v > 1.0 + 1e-9)
    return false;
  const double hitT = e2.dot(q) * invDet;
  if (hitT <= 0.0)
    return false;
  t = hitT;
  return true;
}

bool rayMesh(const Vec3 &origin, const Vec3 &direction, const Mesh &mesh, int &triangleIndex, Vec3 &hit) {
  double best = std::numeric_limits<double>::max();
  int bestTri = -1;
  for (int i = 0; i < mesh.triangleCount(); ++i) {
    const int j = i * 3;
    double t;
    if (rayTriangle(origin, direction, mesh.positions[mesh.triangles[j]], mesh.positions[mesh.triangles[j + 1]],
                    mesh.positions[mesh.triangles[j + 2]], t) &&
        t < best) {
      best = t;
      bestTri = i;
    }
  }
  if (bestTri < 0)
    return false;
  triangleIndex = bestTri;
  hit = origin + direction * best;
  return true;
}

// Solve a 3x3 system (Gaussian elimination with partial pivoting).
static bool solve3(double a[3][4], double x[3]) {
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row)
      if (std::fabs(a[row][col]) > std::fabs(a[pivot][col]))
        pivot = row;
    if (std::fabs(a[pivot][col]) < 1e-12)
      return false;
    if (pivot != col)
      for (int k = col; k < 4; ++k)
        std::swap(a[pivot][k], a[col][k]);
    for (int row = 0; row < 3; ++row) {
      if (row == col)
        continue;
      const double f = a[row][col] / a[col][col];
      for (int k = col; k < 4; ++k)
        a[row][k] -= f * a[col][k];
    }
  }
  for (int i = 0; i < 3; ++i)
    x[i] = a[i][3] / a[i][i];
  return true;
}

bool fitCircle(const std::vector<Vec3> &points, Vec3 &center, Vec3 &normal, double &radius, double maxRelativeResidual) {
  const int n = (int)points.size();
  if (n < 4)
    return false;
  // plane fit: centroid + Newell normal
  Vec3 c;
  for (int i = 0; i < n; ++i)
    c = c + points[i];
  c = c * (1.0 / n);
  Vec3 newell;
  for (int i = 0; i < n; ++i) {
    const Vec3 &p = points[i];
    const Vec3 &q = points[(i + 1) % n];
    newell.x += (p.y - q.y) * (p.z + q.z);
    newell.y += (p.z - q.z) * (p.x + q.x);
    newell.z += (p.x - q.x) * (p.y + q.y);
  }
  const Vec3 N = newell.normalized();
  if (N.isZero())
    return false;
  Vec3 u = (points[0] - c).normalized();
  if (u.isZero())
    return false;
  const Vec3 v = N.cross(u);
  // Kåsa fit: x^2 + y^2 = A x + B y + C in the (u, v) plane
  double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, szx = 0, szy = 0, sz = 0;
  for (int i = 0; i < n; ++i) {
    const Vec3 d = points[i] - c;
    const double x = d.dot(u), y = d.dot(v);
    const double z = x * x + y * y;
    sxx += x * x;
    sxy += x * y;
    sx += x;
    syy += y * y;
    sy += y;
    szx += z * x;
    szy += z * y;
    sz += z;
  }
  double sys[3][4] = {{sxx, sxy, sx, szx}, {sxy, syy, sy, szy}, {sx, sy, (double)n, sz}};
  double coeff[3];
  if (!solve3(sys, coeff))
    return false;
  const double A = coeff[0], B = coeff[1], C = coeff[2];
  const double r2 = C + 0.25 * (A * A + B * B);
  if (r2 <= 0.0)
    return false;
  radius = std::sqrt(r2);
  const double cx = 0.5 * A, cy = 0.5 * B;
  center = c + u * cx + v * cy;
  normal = N;
  // residual: every point of the loop must lie on the circle
  double maxResidual = 0.0;
  for (int i = 0; i < n; ++i)
    maxResidual = std::max(maxResidual, std::fabs(points[i].distance(center) - radius));
  return maxResidual <= maxRelativeResidual * radius;
}

}  // namespace wbmesh
