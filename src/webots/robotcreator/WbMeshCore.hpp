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

// Description: geometry kernel of the Robot Creator: minimal math (Vec3/Mat3),
//              STL (ascii + binary) and OBJ loaders, mesh topology helpers,
//              ray casting and circle fitting.
//
// This module is intentionally free of Qt/Webots dependencies so that it can be
// unit-tested standalone (see tests/src/robotcreator).

#ifndef WB_MESH_CORE_HPP
#define WB_MESH_CORE_HPP

#include <cstddef>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

namespace wbmesh {

struct Vec3 {
  double x, y, z;
  Vec3() : x(0.0), y(0.0), z(0.0) {}
  Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  Vec3 operator+(const Vec3 &o) const { return Vec3(x + o.x, y + o.y, z + o.z); }
  Vec3 operator-(const Vec3 &o) const { return Vec3(x - o.x, y - o.y, z - o.z); }
  Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
  double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3 &o) const { return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x); }
  double length() const;
  Vec3 normalized() const;
  double distance(const Vec3 &o) const;
  bool isZero(double epsilon = 1e-12) const;
};

// Row-major 3x3 rotation matrix.
struct Mat3 {
  double m[9];
  static Mat3 identity();
  static Mat3 fromAxisAngle(const Vec3 &axis, double angle);
  Mat3 transposed() const;
  Vec3 mul(const Vec3 &v) const;
  Mat3 mul(const Mat3 &o) const;
};

// Convert a rotation matrix to Webots' axis + angle representation.
void toAxisAngle(const Mat3 &r, Vec3 &axis, double &angle);

struct Mesh {
  std::vector<Vec3> positions;  // welded vertex list
  std::vector<int> triangles;   // 3 vertex indices per triangle
  Vec3 boundsMin, boundsMax;

  int triangleCount() const { return (int)(triangles.size() / 3); }
  void computeBounds();
  bool isEmpty() const { return triangles.empty(); }
};

// Loaders replace the content of `out`; on failure they return false and set `error`.
bool loadStl(const std::string &path, Mesh &out, std::string &error);
bool loadObj(const std::string &path, Mesh &out, std::string &error);
bool loadMeshFile(const std::string &path, Mesh &out, std::string &error);  // dispatches on the extension

// In-memory parsing (used by the loaders and by the tests).
bool parseStlAscii(std::istream &in, Mesh &out, std::string &error);
bool parseStlBinary(const char *data, size_t size, Mesh &out, std::string &error);
bool parseObj(std::istream &in, Mesh &out, std::string &error);

// Weld coincident soup vertices (epsilon grid quantization) into an indexed mesh.
void weldSoup(const std::vector<Vec3> &soup, Mesh &out, double epsilon = 1e-9);

// Unique edge list (index pairs with a < b).
std::vector<std::pair<int, int> > computeEdges(const Mesh &mesh);

Vec3 faceNormal(const Mesh &mesh, int triangleIndex);
Vec3 faceCentroid(const Mesh &mesh, int triangleIndex);

// Möller–Trumbore ray/triangle intersection; `t` is the ray parameter.
bool rayTriangle(const Vec3 &origin, const Vec3 &direction, const Vec3 &a, const Vec3 &b, const Vec3 &c, double &t);
// Closest intersection with the mesh; returns false when the ray misses.
bool rayMesh(const Vec3 &origin, const Vec3 &direction, const Mesh &mesh, int &triangleIndex, Vec3 &hit);

// Algebraic (Kåsa) circle fit through a closed loop of (ordered) points.
// Returns false for degenerate loops or when the residual exceeds
// `maxRelativeResidual` (relative to the fitted radius).
bool fitCircle(const std::vector<Vec3> &points, Vec3 &center, Vec3 &normal, double &radius,
               double maxRelativeResidual = 0.1);

}  // namespace wbmesh

#endif
