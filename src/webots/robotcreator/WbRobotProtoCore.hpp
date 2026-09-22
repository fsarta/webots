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

// Description: robot assembly model of the Robot Creator (parts + joints) and
//              Webots .wbt text generation. Closed kinematic chains are exported
//              with SolidReference loop-closing joints and validated with the
//              Grübler-Kutzbach analysis of the kinematic graph core.
//
// This module is intentionally free of Qt/Webots dependencies (see tests/src/robotcreator).

#ifndef WB_ROBOT_PROTO_CORE_HPP
#define WB_ROBOT_PROTO_CORE_HPP

#include "WbKinematicGraphCore.hpp"
#include "WbMeshCore.hpp"

#include <string>
#include <vector>

namespace wbrobotproto {

// A rigid body of the assembly: one imported mesh (or an empty link) with a pose.
struct Part {
  std::string name;
  std::string meshUrl;  // written into the Shape/Mesh nodes; empty = empty link
  wbmesh::Mesh mesh;
  wbmesh::Vec3 translation;  // world pose
  wbmesh::Vec3 rotationAxis;
  double rotationAngle;

  Part() : rotationAngle(0.0) {}
};

// A joint between two parts, placed on a CAD snap point.
struct JointDef {
  std::string name;
  wbmechanism::JointType type;
  wbmesh::Vec3 anchor;  // world coordinates
  wbmesh::Vec3 axis;    // world direction (unused for FixedJoint)
  std::string parentPart;
  std::string childPart;

  JointDef() : type(wbmechanism::REVOLUTE_JOINT) {}
};

struct Assembly {
  std::vector<Part> parts;
  std::vector<JointDef> joints;

  int indexOfPart(const std::string &name) const;
};

// Kinematic graph used for the mobility validation (same topology as the export).
wbmechanism::Graph toGraph(const Assembly &assembly);

// spanningFlags[i] == true when joints[i] is a spanning-forest edge (exported as
// a nested tree joint); false marks loop-closing joints (SolidReference).
std::vector<bool> spanningFlags(const Assembly &assembly);

// Grübler-Kutzbach validation of the assembly.
wbmechanism::Analysis validate(const Assembly &assembly, bool planar = false);

// Webots robot text (.wbt snippet ready to paste into a world or save as PROTO body).
std::string generateWbt(const Assembly &assembly, const std::string &robotName);

}  // namespace wbrobotproto

#endif
