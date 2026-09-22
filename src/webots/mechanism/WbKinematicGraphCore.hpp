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

// Description: kinematic graph model and analysis (links, joints, closed kinematic
//              loops, mobility and 2D layout) used by the Mechanism Editor.
//
// This module is intentionally free of Qt/Webots dependencies so that the graph
// analysis can be unit-tested standalone (see tests/src/mechanism).

#ifndef WB_KINEMATIC_GRAPH_CORE_HPP
#define WB_KINEMATIC_GRAPH_CORE_HPP

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace wbmechanism {

// Joint kinds, in the order used by the Mechanism Editor toolbar.
enum JointType {
  FIXED_JOINT = 0,
  REVOLUTE_JOINT,
  PRISMATIC_JOINT,
  REVOLUTE2_JOINT,
  SPHERICAL_JOINT,
};

// Number of degrees of freedom allowed by a joint kind.
int jointDof(JointType type);

// Human readable joint kind names ("HingeJoint", ...).
const char *jointTypeName(JointType type);

struct Link {
  long id;
  std::string name;
  bool isStatic;  // grounded link (merged with the static environment in the analysis)

  Link() : id(-1), isStatic(false) {}
};

struct Joint {
  long id;
  std::string name;
  JointType type;
  long parentLinkId;  // link the parent side of the joint is attached to
  long childLinkId;   // link driven by the joint
  // true when the child link is reached through a SolidReference, i.e. this joint
  // closes a kinematic loop instead of extending the kinematic tree
  bool isLoopClosure;
  double axis[3];
  double anchor[3];  // in parent link coordinates
  bool hasLimits;
  double minStop;
  double maxStop;
  double springConstant;
  double dampingConstant;
  double position;  // current generalized position (display purpose)

  Joint() :
    id(-1), type(FIXED_JOINT), parentLinkId(-1), childLinkId(-1), isLoopClosure(false), hasLimits(false), minStop(0.0),
    maxStop(0.0), springConstant(0.0), dampingConstant(0.0), position(0.0) {
    axis[0] = 0.0;
    axis[1] = 0.0;
    axis[2] = 1.0;
    anchor[0] = anchor[1] = anchor[2] = 0.0;
  }
};

class Graph {
public:
  // building
  long addLink(const std::string &name, bool isStatic);
  long addJoint(const std::string &name, JointType type, long parentLinkId, long childLinkId, bool isLoopClosure);

  // access
  const std::vector<Link> &links() const { return mLinks; }
  const std::vector<Joint> &joints() const { return mJoints; }
  std::vector<Link> &links() { return mLinks; }
  std::vector<Joint> &joints() { return mJoints; }
  const Link *link(long id) const;
  const Joint *joint(long id) const;
  Link *link(long id);
  Joint *joint(long id);
  int linkCount() const { return (int)mLinks.size(); }
  int jointCount() const { return (int)mJoints.size(); }
  // joints attached to a given link (either side)
  std::vector<long> jointsOfLink(long linkId) const;
  bool isEmpty() const { return mLinks.empty(); }
  void clear();

private:
  std::vector<Link> mLinks;
  std::vector<Joint> mJoints;
};

struct Loop {
  long closingJointId;         // the joint which closes the loop (a loop-closure or a back edge)
  std::vector<long> jointIds;  // fundamental cycle: closing joint + spanning-tree path
  int loopDof;                 // total joint DOF along the cycle
  int requiredDof;             // 6 for spatial loops, 3 for planar loops
  // true when the loop has less joint DOF than the generic loop-closure requirement:
  // such a loop needs a special geometric alignment (e.g. parallelogram) and is
  // handled by the physics engine as a soft constraint
  bool isGeometricallyDependent;
};

enum MobilityClass {
  OVER_CONSTRAINED,     // negative mobility: more constraints than generic DOF
  STATIC_STRUCTURE,     // zero mobility: rigid assembly
  MECHANISM,            // positive mobility: M degrees of freedom
};

struct Analysis {
  int mobility;  // Grübler–Kutzbach mobility count
  int nonGroundLinkCount;
  int jointCount;
  int totalJointDof;
  bool planar;
  MobilityClass mobilityClass;
  std::vector<Loop> loops;
  std::vector<std::string> warnings;
  // true when the graph is fully connected through joints (a single mechanism)
  bool isConnected;
  std::string summary() const;
};

// Analyze a kinematic graph: connectivity, fundamental loops (closed kinematic
// chains) and Grübler–Kutzbach mobility. Grounded links (isStatic) are merged
// into one virtual ground; if none exists, the lowest link id is used instead.
Analysis analyze(const Graph &graph, bool planar = false);

struct LayoutOptions {
  double layerWidth;   // horizontal distance between two consecutive layers
  double layerHeight;  // vertical distance between layers
  double nodeSpacing;  // horizontal spacing between links of the same layer
  LayoutOptions() : layerWidth(260.0), layerHeight(150.0), nodeSpacing(190.0) {}
};

struct Layout {
  std::map<long, std::pair<double, double> > linkPositions;
  std::map<long, std::pair<double, double> > jointPositions;  // glyph position on the parent-child edge
};

// Layered 2D layout of the mechanism, from the ground link downwards. Tree joints
// become straight edges, loop closures are chords drawn as arcs by the canvas.
Layout computeLayout(const Graph &graph, const LayoutOptions &options = LayoutOptions());

}  // namespace wbmechanism

#endif
