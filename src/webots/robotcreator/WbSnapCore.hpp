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

// Description: CAD-style object snaps of the Robot Creator: feature extraction
//              (endpoints, edge midpoints, face centroids, circle centers) and
//              snap candidate ranking with Fusion-360-like priorities.
//
// This module is intentionally free of Qt/Webots dependencies (see tests/src/robotcreator).

#ifndef WB_SNAP_CORE_HPP
#define WB_SNAP_CORE_HPP

#include "WbMeshCore.hpp"

#include <string>
#include <vector>

namespace wbsnap {

// Snap kinds (bit flags, from the CAD toolbar).
enum SnapMode {
  SNAP_NONE = 0,
  SNAP_ENDPOINT = 1,       // mesh vertices ("endpoint")
  SNAP_MIDPOINT = 2,       // edge midpoints ("midpoint")
  SNAP_CIRCLE_CENTER = 4,  // fitted circle centers ("circle center")
  SNAP_CENTROID = 8,       // triangle centroids ("median")
  SNAP_GRID = 16           // work-plane grid ("grid")
};

struct SnapCandidate {
  SnapMode mode;
  wbmesh::Vec3 position;
  wbmesh::Vec3 axis;  // meaningful when hasAxis is true
  bool hasAxis;
  std::string label;
  double screenDistance;  // in pixels, set by the caller (viewport projection)
  double depth;           // tie-break: nearer to the camera first

  SnapCandidate() : mode(SNAP_NONE), hasAxis(false), screenDistance(0.0), depth(0.0) {}
};

struct CircleFeature {
  wbmesh::Vec3 center;
  wbmesh::Vec3 normal;  // circle axis, usable as joint axis
  double radius;
};

struct FeatureSet {
  std::vector<wbmesh::Vec3> endpoints;
  std::vector<wbmesh::Vec3> midpoints;
  std::vector<wbmesh::Vec3> centroids;
  std::vector<CircleFeature> circles;
};

// Extract the snap features of a mesh. Circles are fitted on closed feature-edge
// loops (boundary edges and sharp creases above `sharpAngleDeg`).
FeatureSet buildFeatures(const wbmesh::Mesh &mesh, double sharpAngleDeg = 30.0);

// All candidates of the enabled kinds (screenDistance left at zero).
std::vector<SnapCandidate> collectCandidates(const FeatureSet &features, int enabledModes);

// Filter by screen proximity and sort with the CAD priority policy:
// the nearest distance band wins; inside the same band (tieBandPixels) the snap
// kind priority wins: endpoint > circle center > midpoint > centroid > grid.
std::vector<SnapCandidate> rankCandidates(std::vector<SnapCandidate> candidates, double maxScreenDistance,
                                          double tieBandPixels = 8.0);

// 0 = highest priority.
int snapPriority(SnapMode mode);
// "endpoint", "midpoint", "circle center", "centroid", "grid".
const char *snapLabel(SnapMode mode);

wbmesh::Vec3 snapToGrid(const wbmesh::Vec3 &point, double gridSize);
// Ray against the z = 0 work plane, snapped to the grid.
bool gridRayHit(const wbmesh::Vec3 &origin, const wbmesh::Vec3 &direction, double gridSize, wbmesh::Vec3 &hit);

}  // namespace wbsnap

#endif
