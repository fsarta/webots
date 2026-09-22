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

#include "WbSnapCore.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace wbsnap {

using wbmesh::Mesh;
using wbmesh::Vec3;

int snapPriority(SnapMode mode) {
  switch (mode) {
    case SNAP_ENDPOINT:
      return 0;
    case SNAP_CIRCLE_CENTER:
      return 1;
    case SNAP_MIDPOINT:
      return 2;
    case SNAP_CENTROID:
      return 3;
    case SNAP_GRID:
      return 4;
    default:
      return 5;
  }
}

const char *snapLabel(SnapMode mode) {
  switch (mode) {
    case SNAP_ENDPOINT:
      return "endpoint";
    case SNAP_MIDPOINT:
      return "midpoint";
    case SNAP_CIRCLE_CENTER:
      return "circle center";
    case SNAP_CENTROID:
      return "centroid";
    case SNAP_GRID:
      return "grid";
    default:
      return "none";
  }
}

FeatureSet buildFeatures(const Mesh &mesh, double sharpAngleDeg) {
  FeatureSet features;
  features.endpoints = mesh.positions;
  features.centroids.reserve(mesh.triangleCount());
  for (int i = 0; i < mesh.triangleCount(); ++i)
    features.centroids.push_back(wbmesh::faceCentroid(mesh, i));

  // classify edges: boundary or sharp crease edges drive the circle detection
  std::vector<std::pair<int, int> > edges = wbmesh::computeEdges(mesh);
  std::map<std::pair<int, int>, std::vector<int> > edgeToTriangles;
  for (int t = 0; t < mesh.triangleCount(); ++t) {
    const int j = t * 3;
    const int v[3] = {mesh.triangles[j], mesh.triangles[j + 1], mesh.triangles[j + 2]};
    for (int k = 0; k < 3; ++k) {
      const int a = v[k], b = v[(k + 1) % 3];
      edgeToTriangles[a < b ? std::make_pair(a, b) : std::make_pair(b, a)].push_back(t);
    }
  }

  const double cosSharp = std::cos(sharpAngleDeg * M_PI / 180.0);
  std::vector<std::pair<int, int> > featureEdges;
  std::map<int, std::vector<int> > adjacency;  // vertex -> feature-edge indices
  for (size_t i = 0; i < edges.size(); ++i) {
    const std::pair<int, int> &e = edges[i];
    features.midpoints.push_back((mesh.positions[e.first] + mesh.positions[e.second]) * 0.5);
    const std::vector<int> &tris = edgeToTriangles[e];
    bool feature = tris.size() != 2;
    if (!feature) {
      const Vec3 n0 = wbmesh::faceNormal(mesh, tris[0]);
      const Vec3 n1 = wbmesh::faceNormal(mesh, tris[1]);
      feature = n0.dot(n1) < cosSharp;
    }
    if (feature) {
      const int id = (int)featureEdges.size();
      featureEdges.push_back(e);
      adjacency[e.first].push_back(id);
      adjacency[e.second].push_back(id);
    }
  }

  // walk the feature edges into closed loops and fit circles on them
  std::set<int> used;
  for (size_t start = 0; start < featureEdges.size(); ++start) {
    if (used.count((int)start))
      continue;
    used.insert((int)start);
    std::vector<int> chain;
    chain.push_back(featureEdges[start].first);
    chain.push_back(featureEdges[start].second);
    bool closed = false;
    for (;;) {
      const int vertex = chain.back();
      const std::vector<int> &candidates = adjacency[vertex];
      int nextEdge = -1;
      int nextVertex = -1;
      for (size_t k = 0; k < candidates.size(); ++k) {
        const int id = candidates[k];
        if (used.count(id))
          continue;
        if (nextEdge >= 0) {  // junction: the chain cannot continue
          nextEdge = -2;
          break;
        }
        nextEdge = id;
        const std::pair<int, int> &e = featureEdges[id];
        nextVertex = e.first == vertex ? e.second : e.first;
      }
      if (nextEdge < 0)
        break;
      used.insert(nextEdge);
      if (nextVertex == chain.front()) {
        closed = true;
        break;
      }
      chain.push_back(nextVertex);
    }
    if (closed && chain.size() >= 4) {
      std::vector<Vec3> loop;
      for (size_t i = 0; i < chain.size(); ++i)
        loop.push_back(mesh.positions[chain[i]]);
      CircleFeature circle;
      if (wbmesh::fitCircle(loop, circle.center, circle.normal, circle.radius, 0.05)) {
        // deduplicate
        bool duplicate = false;
        for (size_t i = 0; i < features.circles.size(); ++i)
          if (features.circles[i].center.distance(circle.center) < 1e-4 * std::max(1.0, circle.radius))
            duplicate = true;
        if (!duplicate)
          features.circles.push_back(circle);
      }
    }
  }
  return features;
}

std::vector<SnapCandidate> collectCandidates(const FeatureSet &features, int enabledModes) {
  std::vector<SnapCandidate> candidates;
  if (enabledModes & SNAP_ENDPOINT) {
    for (size_t i = 0; i < features.endpoints.size(); ++i) {
      SnapCandidate c;
      c.mode = SNAP_ENDPOINT;
      c.label = snapLabel(SNAP_ENDPOINT);
      c.position = features.endpoints[i];
      candidates.push_back(c);
    }
  }
  if (enabledModes & SNAP_CIRCLE_CENTER) {
    for (size_t i = 0; i < features.circles.size(); ++i) {
      SnapCandidate c;
      c.mode = SNAP_CIRCLE_CENTER;
      c.label = snapLabel(SNAP_CIRCLE_CENTER);
      c.position = features.circles[i].center;
      c.axis = features.circles[i].normal;
      c.hasAxis = true;
      candidates.push_back(c);
    }
  }
  if (enabledModes & SNAP_MIDPOINT) {
    for (size_t i = 0; i < features.midpoints.size(); ++i) {
      SnapCandidate c;
      c.mode = SNAP_MIDPOINT;
      c.label = snapLabel(SNAP_MIDPOINT);
      c.position = features.midpoints[i];
      candidates.push_back(c);
    }
  }
  if (enabledModes & SNAP_CENTROID) {
    for (size_t i = 0; i < features.centroids.size(); ++i) {
      SnapCandidate c;
      c.mode = SNAP_CENTROID;
      c.label = snapLabel(SNAP_CENTROID);
      c.position = features.centroids[i];
      candidates.push_back(c);
    }
  }
  return candidates;
}

namespace {
struct CandidateLess {
  double tieBand;
  bool operator()(const SnapCandidate &a, const SnapCandidate &b) const {
    const long bandA = (long)std::floor(a.screenDistance / tieBand);
    const long bandB = (long)std::floor(b.screenDistance / tieBand);
    if (bandA != bandB)
      return bandA < bandB;
    const int pA = snapPriority(a.mode), pB = snapPriority(b.mode);
    if (pA != pB)
      return pA < pB;
    if (a.screenDistance != b.screenDistance)
      return a.screenDistance < b.screenDistance;
    return a.depth < b.depth;
  }
};
}  // namespace

std::vector<SnapCandidate> rankCandidates(std::vector<SnapCandidate> candidates, double maxScreenDistance,
                                          double tieBandPixels) {
  std::vector<SnapCandidate> kept;
  for (size_t i = 0; i < candidates.size(); ++i)
    if (candidates[i].screenDistance <= maxScreenDistance)
      kept.push_back(candidates[i]);
  CandidateLess less;
  less.tieBand = std::max(tieBandPixels, 1e-6);
  std::stable_sort(kept.begin(), kept.end(), less);
  return kept;
}

wbmesh::Vec3 snapToGrid(const wbmesh::Vec3 &point, double gridSize) {
  if (gridSize <= 0.0)
    return point;
  return Vec3(std::floor(point.x / gridSize + 0.5) * gridSize, std::floor(point.y / gridSize + 0.5) * gridSize,
              std::floor(point.z / gridSize + 0.5) * gridSize);
}

bool gridRayHit(const wbmesh::Vec3 &origin, const wbmesh::Vec3 &direction, double gridSize, wbmesh::Vec3 &hit) {
  if (std::fabs(direction.z) < 1e-12)
    return false;
  const double t = -origin.z / direction.z;
  if (t <= 0.0)
    return false;
  hit = snapToGrid(origin + direction * t, gridSize);
  return true;
}

}  // namespace wbsnap
