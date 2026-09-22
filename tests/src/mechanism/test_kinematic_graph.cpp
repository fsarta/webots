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

// Standalone unit tests for WbKinematicGraphCore (closed kinematic chains included).

#include <cstdio>
#include <cstdlib>
#include <string>

#include "WbKinematicGraphCore.hpp"

static int gFailures = 0;
static int gChecks = 0;

#define CHECK(cond) \
  do { \
    ++gChecks; \
    if (!(cond)) { \
      ++gFailures; \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
  } while (0)

#define CHECK_EQ(a, b) \
  do { \
    ++gChecks; \
    if (!((a) == (b))) { \
      ++gFailures; \
      fprintf(stderr, "FAIL %s:%d: %s == %s (%ld vs %ld)\n", __FILE__, __LINE__, #a, #b, (long)(a), (long)(b)); \
    } \
  } while (0)

using namespace wbmechanism;

// one grounded link + one hinge = 1 DOF pendulum
static void testSimplePendulum() {
  Graph g;
  const long base = g.addLink("base", true);
  const long arm = g.addLink("arm", false);
  g.addJoint("j0", REVOLUTE_JOINT, base, arm, false);

  Analysis a = analyze(g);
  CHECK_EQ(a.mobility, 1);
  CHECK(a.mobilityClass == MECHANISM);
  CHECK(a.loops.empty());
  CHECK(a.isConnected);
}

// closed five-bar linkage: ground + 4 links + 5 revolute joints (1 loop)
// planar: 3*(4-5) + 5 = 2 DOF; spatial: 6*(4-5) + 5 = -1 (over-constrained)
static void testFiveBarLinkage() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long l1 = g.addLink("link1", false);
  const long l2 = g.addLink("link2", false);
  const long l3 = g.addLink("link3", false);
  const long l4 = g.addLink("link4", false);
  g.addJoint("j0", REVOLUTE_JOINT, ground, l1, false);
  g.addJoint("j1", REVOLUTE_JOINT, l1, l2, false);
  g.addJoint("j2", REVOLUTE_JOINT, l2, l3, false);
  g.addJoint("j3", REVOLUTE_JOINT, l3, l4, false);
  g.addJoint("loop", REVOLUTE_JOINT, l4, ground, true);

  Analysis planar = analyze(g, true);
  Analysis spatial = analyze(g, false);
  CHECK_EQ((long)planar.loops.size(), 1);
  CHECK_EQ((long)spatial.loops.size(), 1);
  // the loop contains the 5 revolute joints
  CHECK(planar.loops[0].loopDof == 5);
  CHECK(!planar.loops[0].isGeometricallyDependent);
  CHECK(spatial.loops[0].loopDof == 5);
  // spatially 5 revolutes cannot close a loop generically (6 required)
  CHECK(spatial.loops[0].isGeometricallyDependent);
  CHECK_EQ(planar.mobility, 2);
  CHECK(planar.mobilityClass == MECHANISM);
  CHECK_EQ(spatial.mobility, -1);
  CHECK(spatial.mobilityClass == OVER_CONSTRAINED);
  CHECK(planar.loops[0].closingJointId == 4);
}

// parallelogram (planar): ground + 3 links + 4 parallel revolute joints, mobility 1
static void testParallelogram() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long left = g.addLink("left_crank", false);
  const long right = g.addLink("right_crank", false);
  const long bar = g.addLink("moving_bar", false);
  g.addJoint("j0", REVOLUTE_JOINT, ground, left, false);
  g.addJoint("j1", REVOLUTE_JOINT, left, bar, false);
  g.addJoint("j2", REVOLUTE_JOINT, ground, right, false);
  g.addJoint("j3", REVOLUTE_JOINT, bar, right, true);  // explicit loop closure

  Analysis a = analyze(g, true);
  CHECK_EQ((long)a.loops.size(), 1);
  CHECK_EQ(a.loops[0].jointIds.size(), (size_t)4);
  // 3*(3 - 4) + 4 = 1
  CHECK_EQ(a.mobility, 1);
  CHECK(a.mobilityClass == MECHANISM);
  // the parallelogram loop needs the special parallel-axis alignment: 4 DOF < 3? no: 4 >= 3
  CHECK(!a.loops[0].isGeometricallyDependent);
}

// slider-crank (planar): ground + 2 links + revolute, revolute, prismatic, loop closure
static void testSliderCrank() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long crank = g.addLink("crank", false);
  const long rod = g.addLink("rod", false);
  const long piston = g.addLink("piston", false);
  g.addJoint("j0", REVOLUTE_JOINT, ground, crank, false);
  g.addJoint("j1", REVOLUTE_JOINT, crank, rod, false);
  g.addJoint("j2", PRISMATIC_JOINT, rod, piston, false);
  g.addJoint("loop", REVOLUTE_JOINT, ground, piston, true);

  Analysis a = analyze(g, true);
  CHECK_EQ((long)a.loops.size(), 1);
  // 3*(3 - 4) + (1+1+1+1) = 1
  CHECK_EQ(a.mobility, 1);
  CHECK(a.mobilityClass == MECHANISM);
  CHECK_EQ(a.loops[0].loopDof, 4);  // revolute + revolute + prismatic + revolute
}

// rigid box on ground (fixed joint only): mobility 0
static void testRigidStructure() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long box = g.addLink("box", false);
  g.addJoint("weld", FIXED_JOINT, ground, box, false);

  Analysis a = analyze(g);
  CHECK_EQ(a.mobility, 0);
  CHECK(a.mobilityClass == STATIC_STRUCTURE);
}

// over-constrained: two fixed joints between the same pair of links
static void testOverConstrained() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long box = g.addLink("box", false);
  g.addJoint("weld1", FIXED_JOINT, ground, box, false);
  g.addJoint("weld2", FIXED_JOINT, ground, box, true);  // explicit redundant loop

  Analysis a = analyze(g);
  CHECK_EQ((long)a.loops.size(), 1);
  CHECK(a.mobility < 0);
  CHECK(a.mobilityClass == OVER_CONSTRAINED);
}

// disconnected link produces a warning
static void testDisconnected() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long arm = g.addLink("arm", false);
  const long orphan = g.addLink("orphan", false);
  (void)orphan;
  g.addJoint("j0", REVOLUTE_JOINT, ground, arm, false);
  // orphan not connected: second component seed
  Analysis a = analyze(g);
  CHECK(!a.isConnected);
  CHECK(!a.warnings.empty());
}

// layout: ground at the top, links stratified by tree depth
static void testLayout() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long a = g.addLink("a", false);
  const long b = g.addLink("b", false);
  const long c = g.addLink("c", false);
  g.addJoint("j0", REVOLUTE_JOINT, ground, a, false);
  g.addJoint("j1", REVOLUTE_JOINT, a, b, false);
  g.addJoint("j2", REVOLUTE_JOINT, ground, c, true);

  Layout layout = computeLayout(g);
  CHECK_EQ((long)layout.linkPositions.size(), 4);
  CHECK_EQ((long)layout.jointPositions.size(), 3);
  const std::pair<double, double> &pGround = layout.linkPositions[ground];
  const std::pair<double, double> &pA = layout.linkPositions[a];
  const std::pair<double, double> &pB = layout.linkPositions[b];
  CHECK(pA.second > pGround.second);  // ground on top
  CHECK(pB.second > pA.second);
  // joint glyph between its endpoints
  const std::pair<double, double> &pJ0 = layout.jointPositions[0];
  CHECK(pJ0.second > pGround.second && pJ0.second < pA.second);
}

// ball joint allows 3 DOF
static void testBallJointDof() {
  Graph g;
  const long ground = g.addLink("ground", true);
  const long l1 = g.addLink("l1", false);
  const long l2 = g.addLink("l2", false);
  const long l3 = g.addLink("l3", false);
  g.addJoint("s0", SPHERICAL_JOINT, ground, l1, false);
  g.addJoint("s1", SPHERICAL_JOINT, l1, l2, false);
  g.addJoint("s2", SPHERICAL_JOINT, l2, l3, false);
  g.addJoint("s3", SPHERICAL_JOINT, ground, l3, true);

  Analysis a = analyze(g, false);
  // spatial: 6*(3 - 4) + 3*4 = 6
  CHECK_EQ(a.mobility, 6);
  CHECK_EQ(a.loops[0].loopDof, 12);  // 4 spherical joints
  CHECK(!a.loops[0].isGeometricallyDependent);
}

int main() {
  testSimplePendulum();
  testFiveBarLinkage();
  testParallelogram();
  testSliderCrank();
  testRigidStructure();
  testOverConstrained();
  testDisconnected();
  testLayout();
  testBallJointDof();

  if (gFailures) {
    fprintf(stderr, "%d/%d checks failed\n", gFailures, gChecks);
    return 1;
  }
  printf("test_kinematic_graph: all %d checks passed\n", gChecks);
  return 0;
}
