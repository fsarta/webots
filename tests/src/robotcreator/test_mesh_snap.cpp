// Standalone unit tests for the Robot Creator cores: mesh loading (STL/OBJ),
// CAD snap features (endpoints, midpoints, circle centers, centroids) and
// Webots robot text generation with closed kinematic chains.

#include "WbKinematicGraphCore.hpp"
#include "WbMeshCore.hpp"
#include "WbRobotProtoCore.hpp"
#include "WbSnapCore.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

static int gChecks = 0;

static void check(bool condition, const std::string &what) {
  ++gChecks;
  if (!condition) {
    std::fprintf(stderr, "FAILED: %s\n", what.c_str());
    std::exit(1);
  }
}

static void checkNear(double value, double expected, double tolerance, const std::string &what) {
  check(std::fabs(value - expected) <= tolerance,
        what + " (got " + std::to_string(value) + ", expected " + std::to_string(expected) + ")");
}

using wbmesh::Mesh;
using wbmesh::Vec3;

static Mesh makeQuadSoup() {
  // two triangles forming a unit square in the z = 0 plane (ASCII STL content)
  std::istringstream stl("solid quad\n"
                         " facet normal 0 0 1\n"
                         "  outer loop\n"
                         "   vertex 0 0 0\n"
                         "   vertex 1 0 0\n"
                         "   vertex 1 1 0\n"
                         "  endloop\n"
                         " endfacet\n"
                         " facet normal 0 0 1\n"
                         "  outer loop\n"
                         "   vertex 0 0 0\n"
                         "   vertex 1 1 0\n"
                         "   vertex 0 1 0\n"
                         "  endloop\n"
                         " endfacet\n"
                         "endsolid\n");
  Mesh mesh;
  std::string error;
  check(wbmesh::parseStlAscii(stl, mesh, error), "ASCII STL parses");
  return mesh;
}

static Mesh makeFan(int rimCount) {
  // triangle fan around the origin: rim boundary is a perfect circle
  Mesh mesh;
  mesh.positions.push_back(Vec3(0, 0, 0));
  for (int i = 0; i < rimCount; ++i) {
    const double a = 2.0 * M_PI * i / rimCount;
    mesh.positions.push_back(Vec3(std::cos(a), std::sin(a), 0));
  }
  for (int i = 0; i < rimCount; ++i) {
    mesh.triangles.push_back(0);
    mesh.triangles.push_back(1 + i);
    mesh.triangles.push_back(1 + (i + 1) % rimCount);
  }
  mesh.computeBounds();
  return mesh;
}

int main() {
  std::string error;

  // --- Vec3 / Mat3 ---------------------------------------------------------
  checkNear(Vec3(3, 4, 0).length(), 5.0, 1e-12, "Vec3 length");
  checkNear(Vec3(1, 0, 0).cross(Vec3(0, 1, 0)).z, 1.0, 1e-12, "Vec3 cross");
  {
    const wbmesh::Mat3 r = wbmesh::Mat3::fromAxisAngle(Vec3(0, 0, 1), M_PI / 2);
    const Vec3 v = r.mul(Vec3(1, 0, 0));
    checkNear(v.x, 0.0, 1e-9, "Mat3 rotate x->y (x)");
    checkNear(v.y, 1.0, 1e-9, "Mat3 rotate x->y (y)");
    Vec3 axis;
    double angle;
    wbmesh::toAxisAngle(r, axis, angle);
    checkNear(angle, M_PI / 2, 1e-9, "toAxisAngle angle");
    checkNear(axis.z, 1.0, 1e-9, "toAxisAngle axis");
  }

  // --- STL ascii + binary --------------------------------------------------
  const Mesh quad = makeQuadSoup();
  check(quad.triangleCount() == 2, "ASCII STL triangle count");
  check(quad.positions.size() == 4, "ASCII STL welded vertex count");
  checkNear(quad.boundsMax.x, 1.0, 1e-9, "ASCII STL bounds");
  {
    // one-facet binary STL
    std::vector<char> buffer(84 + 50, 0);
    const unsigned int count = 1;
    std::memcpy(&buffer[80], &count, 4);
    const float data[12] = {0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0};
    std::memcpy(&buffer[84 + 12], data + 3, 36);  // the 3 vertices (after the normal)
    Mesh mesh;
    check(wbmesh::parseStlBinary(buffer.data(), buffer.size(), mesh, error), "binary STL parses");
    check(mesh.triangleCount() == 1, "binary STL triangle count");
    check(!wbmesh::parseStlBinary(buffer.data(), 20, mesh, error), "truncated binary STL rejected");
  }

  // --- OBJ -----------------------------------------------------------------
  {
    std::istringstream obj("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n"
                           "f 1/1/1 2/2/2 3/3/3 4/4/4\n"
                           "f -4 -3 -2\n");
    Mesh mesh;
    check(wbmesh::parseObj(obj, mesh, error), "OBJ parses");
    check(mesh.triangleCount() == 3, "OBJ quad fan + triangle");
    check(mesh.positions.size() == 4, "OBJ vertex count");
  }

  // --- topology helpers ----------------------------------------------------
  check(wbmesh::computeEdges(quad).size() == 5, "quad edge count (5 unique)");
  checkNear(wbmesh::faceNormal(quad, 0).z, 1.0, 1e-9, "face normal");
  checkNear(wbmesh::faceCentroid(quad, 0).x, 2.0 / 3.0, 1e-9, "face centroid x");

  // --- ray casting ---------------------------------------------------------
  {
    double t;
    check(wbmesh::rayTriangle(Vec3(0.2, 0.2, 1), Vec3(0, 0, -1), Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), t),
          "ray hits triangle");
    checkNear(t, 1.0, 1e-9, "ray triangle t");
    check(!wbmesh::rayTriangle(Vec3(2, 2, 1), Vec3(0, 0, -1), Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), t),
          "ray misses triangle");
    const Mesh fan = makeFan(16);
    int tri;
    Vec3 hit;
    check(wbmesh::rayMesh(Vec3(0, 0, 5), Vec3(0, 0, -1), fan, tri, hit), "ray hits mesh");
    checkNear(hit.z, 0.0, 1e-9, "ray mesh hit z");
  }

  // --- circle fit ----------------------------------------------------------
  {
    std::vector<Vec3> loop;
    for (int i = 0; i < 16; ++i) {
      const double a = 2.0 * M_PI * i / 16;
      loop.push_back(Vec3(2.0 * std::cos(a), 2.0 * std::sin(a), 0.5));
    }
    Vec3 center, normal;
    double radius;
    check(wbmesh::fitCircle(loop, center, normal, radius), "circle fit on regular loop");
    checkNear(center.x, 0.0, 1e-6, "circle center x");
    checkNear(center.z, 0.5, 1e-6, "circle center z");
    checkNear(radius, 2.0, 1e-6, "circle radius");
    checkNear(std::fabs(normal.z), 1.0, 1e-6, "circle normal");
    std::vector<Vec3> line;
    for (int i = 0; i < 5; ++i)
      line.push_back(Vec3(i, 0, 0));
    check(!wbmesh::fitCircle(line, center, normal, radius), "collinear loop rejected");
  }

  // --- snap features -------------------------------------------------------
  const Mesh fan = makeFan(16);
  const wbsnap::FeatureSet features = wbsnap::buildFeatures(fan);
  check(features.endpoints.size() == 17, "feature endpoints (center + rim)");
  check(features.midpoints.size() == 32, "feature midpoints (16 rim + 16 spokes)");
  check(features.centroids.size() == 16, "feature centroids");
  check(features.circles.size() == 1, "rim circle detected");
  if (!features.circles.empty()) {
    checkNear(features.circles[0].center.x, 0.0, 1e-3, "rim circle center");
    checkNear(features.circles[0].radius, 1.0, 1e-3, "rim circle radius");
  }

  std::vector<wbsnap::SnapCandidate> candidates =
    wbsnap::collectCandidates(features, wbsnap::SNAP_ENDPOINT | wbsnap::SNAP_CIRCLE_CENTER | wbsnap::SNAP_MIDPOINT);
  check(candidates.size() == 17 + 32 + 1, "collectCandidates count");
  std::vector<wbsnap::SnapCandidate> endpointsOnly = wbsnap::collectCandidates(features, wbsnap::SNAP_ENDPOINT);
  check(endpointsOnly.size() == 17, "mode filtering");

  // --- snap ranking (CAD priority) -----------------------------------------
  {
    using wbsnap::SnapCandidate;
    SnapCandidate endpoint, midpoint, grid;
    endpoint.mode = wbsnap::SNAP_ENDPOINT;
    endpoint.label = wbsnap::snapLabel(endpoint.mode);
    endpoint.screenDistance = 6.0;
    midpoint.mode = wbsnap::SNAP_MIDPOINT;
    midpoint.screenDistance = 7.0;
    grid.mode = wbsnap::SNAP_GRID;
    grid.screenDistance = 1.0;
    std::vector<SnapCandidate> input;
    input.push_back(midpoint);
    input.push_back(endpoint);
    input.push_back(grid);
    std::vector<SnapCandidate> ranked = wbsnap::rankCandidates(input, 10.0, 8.0);
    check(ranked.size() == 3, "rankCandidates keeps all in range");
    // same distance band (0-8 px): object-snap priority wins (endpoint > midpoint > grid)
    check(ranked[0].mode == wbsnap::SNAP_ENDPOINT, "tie band: endpoint beats midpoint and grid");
    check(ranked[1].mode == wbsnap::SNAP_MIDPOINT, "tie band: midpoint beats grid");
    check(ranked[2].mode == wbsnap::SNAP_GRID, "grid is the lowest priority");
    ranked = wbsnap::rankCandidates(input, 5.0, 8.0);
    check(ranked.size() == 1 && ranked[0].mode == wbsnap::SNAP_GRID, "maxScreenDistance filters");
    endpoint.screenDistance = 10.0;  // next band
    input.clear();
    input.push_back(midpoint);
    input.push_back(endpoint);
    ranked = wbsnap::rankCandidates(input, 12.0, 8.0);
    check(ranked[0].mode == wbsnap::SNAP_MIDPOINT, "nearer band wins over priority");
  }

  // --- grid snapping -------------------------------------------------------
  checkNear(wbsnap::snapToGrid(Vec3(0.2, -0.31, 0.04), 0.25).x, 0.25, 1e-9, "grid snap x");
  checkNear(wbsnap::snapToGrid(Vec3(0.2, -0.31, 0.04), 0.25).y, -0.25, 1e-9, "grid snap y");
  {
    Vec3 hit;
    check(wbsnap::gridRayHit(Vec3(0.1, 0.1, 2), Vec3(0, 0, -1), 0.5, hit), "grid ray hit");
    checkNear(hit.x, 0.0, 1e-9, "grid ray snapped x");
  }

  // --- assembly: mobility + export ------------------------------------------
  wbrobotproto::Assembly assembly;
  const char *names[4] = {"CRANK", "COUPLER", "ROCKER", "GROUND"};
  for (int i = 0; i < 4; ++i) {
    wbrobotproto::Part part;
    part.name = names[i];
    part.meshUrl = std::string(names[i]) + ".stl";
    part.translation = Vec3(i, 0, 0);
    assembly.parts.push_back(part);
  }
  struct JointSpec {
    const char *parent;
    const char *child;
  } specs[4] = {{"GROUND", "CRANK"}, {"CRANK", "COUPLER"}, {"COUPLER", "ROCKER"}, {"ROCKER", "GROUND"}};
  for (int i = 0; i < 4; ++i) {
    wbrobotproto::JointDef def;
    def.name = std::string("J") + std::to_string(i);
    def.type = wbmechanism::REVOLUTE_JOINT;
    def.parentPart = specs[i].parent;
    def.childPart = specs[i].child;
    def.anchor = Vec3(0.5, 0, 0.5);
    def.axis = Vec3(0, 0, 1);
    assembly.joints.push_back(def);
  }
  const std::vector<bool> flags = wbrobotproto::spanningFlags(assembly);
  int treeCount = 0;
  for (size_t i = 0; i < flags.size(); ++i)
    if (flags[i])
      ++treeCount;
  check(treeCount == 3, "four-bar spanning tree has 3 edges (1 loop chord)");
  check(!flags[3], "the last joint closes the loop");

  const wbmechanism::Graph graph = wbrobotproto::toGraph(assembly);
  check(graph.linkCount() == 4, "assembly graph links");
  check(graph.jointCount() == 4, "assembly graph joints");

  const wbmechanism::Analysis planar = wbrobotproto::validate(assembly, true);
  check(planar.loops.size() == 1, "four-bar has one loop");
  check(planar.mobility == 1, "four-bar planar mobility is 1");
  const wbmechanism::Analysis spatial = wbrobotproto::validate(assembly, false);
  check(!spatial.loops.empty() && spatial.loops[0].isGeometricallyDependent, "four-bar is geometrically dependent");

  const std::string wbt = wbrobotproto::generateWbt(assembly, "four_bar");
  check(wbt.find("Robot {") != std::string::npos, "wbt has Robot");
  check(wbt.find("name \"four_bar\"") != std::string::npos, "wbt robot name");
  check(wbt.find("HingeJoint {") != std::string::npos, "wbt has HingeJoint");
  check(wbt.find("SolidReference { solidName \"GROUND\" }") != std::string::npos, "wbt loop joint uses SolidReference");
  check(wbt.find("geometry Mesh { url \"CRANK.stl\" }") != std::string::npos, "wbt has mesh url");
  check(wbt.find("SliderJoint") == std::string::npos, "wbt has no slider");
  check(wbt.find("physics Physics { density -1 mass 1 }") != std::string::npos, "wbt links have physics");

  // slider mapping + relative pose: child moved 1m along x under parent at 2m
  {
    wbrobotproto::Assembly slider;
    wbrobotproto::Part base;
    base.name = "BASE";
    base.translation = Vec3(2, 0, 0);
    slider.parts.push_back(base);
    wbrobotproto::Part carriage;
    carriage.name = "CARRIAGE";
    carriage.translation = Vec3(3, 0, 0);
    slider.parts.push_back(carriage);
    wbrobotproto::JointDef def;
    def.name = "SLIDE";
    def.type = wbmechanism::PRISMATIC_JOINT;
    def.parentPart = "BASE";
    def.childPart = "CARRIAGE";
    def.anchor = Vec3(3, 0, 0);
    def.axis = Vec3(1, 0, 0);
    slider.joints.push_back(def);
    const std::string text = wbrobotproto::generateWbt(slider, "slider_bot");
    check(text.find("SliderJoint {") != std::string::npos, "slider export");
    check(text.find("SliderJointParameters {") != std::string::npos, "slider parameters");
    check(text.find("translation 1 0 0") != std::string::npos, "child pose is parent-relative (1m)");
  }

  std::printf("test_mesh_snap: all %d checks passed\n", gChecks);
  return 0;
}
