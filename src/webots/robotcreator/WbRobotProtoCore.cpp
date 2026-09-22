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

#include "WbRobotProtoCore.hpp"

#include <cstdio>
#include <map>
#include <sstream>

namespace wbrobotproto {

using wbmesh::Mat3;
using wbmesh::Vec3;

int Assembly::indexOfPart(const std::string &name) const {
  for (size_t i = 0; i < parts.size(); ++i)
    if (parts[i].name == name)
      return (int)i;
  return -1;
}

// union-find over parts, used for the spanning-forest classification
static int ufFind(std::vector<int> &parent, int x) {
  while (parent[x] != x) {
    parent[x] = parent[parent[x]];
    x = parent[x];
  }
  return x;
}

std::vector<bool> spanningFlags(const Assembly &assembly) {
  std::vector<int> parent(assembly.parts.size());
  for (size_t i = 0; i < parent.size(); ++i)
    parent[i] = (int)i;
  std::vector<bool> flags(assembly.joints.size(), false);
  for (size_t j = 0; j < assembly.joints.size(); ++j) {
    const int p = assembly.indexOfPart(assembly.joints[j].parentPart);
    const int c = assembly.indexOfPart(assembly.joints[j].childPart);
    if (p < 0 || c < 0 || p == c)
      continue;
    const int rp = ufFind(parent, p), rc = ufFind(parent, c);
    if (rp != rc) {
      flags[j] = true;
      parent[rc] = rp;
    }
  }
  return flags;
}

wbmechanism::Graph toGraph(const Assembly &assembly) {
  wbmechanism::Graph graph;
  std::map<std::string, long> ids;
  for (size_t i = 0; i < assembly.parts.size(); ++i)
    ids[assembly.parts[i].name] = graph.addLink(assembly.parts[i].name, false);
  const std::vector<bool> flags = spanningFlags(assembly);
  for (size_t j = 0; j < assembly.joints.size(); ++j) {
    const JointDef &def = assembly.joints[j];
    std::map<std::string, long>::const_iterator p = ids.find(def.parentPart);
    std::map<std::string, long>::const_iterator c = ids.find(def.childPart);
    if (p == ids.end() || c == ids.end())
      continue;
    const long id = graph.addJoint(def.name, def.type, p->second, c->second, !flags[j]);
    wbmechanism::Joint *added = graph.joint(id);
    if (added) {
      added->anchor[0] = def.anchor.x;
      added->anchor[1] = def.anchor.y;
      added->anchor[2] = def.anchor.z;
      added->axis[0] = def.axis.x;
      added->axis[1] = def.axis.y;
      added->axis[2] = def.axis.z;
    }
  }
  return graph;
}

wbmechanism::Analysis validate(const Assembly &assembly, bool planar) {
  return wbmechanism::analyze(toGraph(assembly), planar);
}

static std::string fmt(double v) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.6g", v);
  return std::string(buffer);
}

static std::string fmtVec(const Vec3 &v) {
  return fmt(v.x) + " " + fmt(v.y) + " " + fmt(v.z);
}

static Mat3 partRotation(const Part &part) {
  return Mat3::fromAxisAngle(part.rotationAxis, part.rotationAngle);
}

// point `world` expressed in the frame of `part`
static Vec3 toPartLocal(const Part &part, const Vec3 &world) {
  return partRotation(part).transposed().mul(world - part.translation);
}

static void emitShape(std::ostringstream &out, const Part &part, const std::string &indent) {
  if (part.meshUrl.empty())
    return;
  out << indent << "Shape {\n";
  out << indent << "  geometry Mesh { url \"" << part.meshUrl << "\" }\n";
  out << indent << "}\n";
}

static void emitPart(std::ostringstream &out, const Assembly &assembly, int partIndex, const std::vector<bool> &flags,
                     const std::string &indent, int parentPartIndex);

static void emitJoint(std::ostringstream &out, const Assembly &assembly, int jointIndex, int childPartIndex,
                      const std::vector<bool> &flags, const std::string &indent) {
  const JointDef &def = assembly.joints[jointIndex];
  const Part &parent = assembly.parts[assembly.indexOfPart(def.parentPart)];
  const Part &child = assembly.parts[childPartIndex];

  std::string type;
  std::string parameters;
  switch (def.type) {
    case wbmechanism::REVOLUTE_JOINT:
      type = "HingeJoint";
      parameters = "JointParameters";
      break;
    case wbmechanism::PRISMATIC_JOINT:
      type = "SliderJoint";
      parameters = "SliderJointParameters";
      break;
    case wbmechanism::REVOLUTE2_JOINT:
      type = "Hinge2Joint";
      parameters = "Hinge2JointParameters";
      break;
    case wbmechanism::SPHERICAL_JOINT:
      type = "BallJoint";
      parameters = "JointParameters";
      break;
    case wbmechanism::FIXED_JOINT:
    default:
      type = "FixedJoint";
      break;
  }

  out << indent << type << " {\n";
  if (!parameters.empty()) {
    out << indent << "  " << parameters << " {\n";
    if (def.type != wbmechanism::SPHERICAL_JOINT) {
      const Vec3 axis = partRotation(child).transposed().mul(def.axis).normalized();
      out << indent << "    axis " << fmtVec(axis) << "\n";
      if (def.type == wbmechanism::REVOLUTE2_JOINT) {
        // axis2 must be perpendicular to axis
        Vec3 axis2 = axis.cross(Vec3(0.0, 0.0, 1.0));
        if (axis2.isZero())
          axis2 = axis.cross(Vec3(1.0, 0.0, 0.0));
        out << indent << "    axis2 " << fmtVec(axis2.normalized()) << "\n";
      }
    }
    if (def.type != wbmechanism::PRISMATIC_JOINT)
      out << indent << "    anchor " << fmtVec(toPartLocal(parent, def.anchor)) << "\n";
    out << indent << "  }\n";
  }

  if (childPartIndex >= 0 && !flags[jointIndex]) {
    // loop-closing joint: the child link lives elsewhere in the tree
    out << indent << "  endPoint SolidReference { solidName \"" << child.name << "\" }\n";
    out << indent << "}\n";
    return;
  }

  out << indent << "  endPoint ";
  emitPart(out, assembly, childPartIndex, flags, indent + "  ", assembly.indexOfPart(def.parentPart));
  out << indent << "}\n";
}

static void emitPart(std::ostringstream &out, const Assembly &assembly, int partIndex, const std::vector<bool> &flags,
                     const std::string &indent, int parentPartIndex) {
  const Part &part = assembly.parts[partIndex];
  // nested end point solids carry a pose relative to their parent link
  Vec3 translation = part.translation;
  Mat3 rotation = partRotation(part);
  if (parentPartIndex >= 0) {
    const Part &parent = assembly.parts[parentPartIndex];
    const Mat3 parentR = partRotation(parent);
    translation = parentR.transposed().mul(part.translation - parent.translation);
    rotation = parentR.transposed().mul(rotation);
  }
  Vec3 axis;
  double angle;
  toAxisAngle(rotation, axis, angle);
  out << indent << "Solid {\n";
  out << indent << "  name \"" << part.name << "\"\n";
  out << indent << "  translation " << fmtVec(translation) << "\n";
  out << indent << "  rotation " << fmtVec(axis) << " " << fmt(angle) << "\n";
  out << indent << "  children [\n";
  emitShape(out, part, indent + "    ");
  // tree joints with this part as parent (nested end point solids)
  for (size_t j = 0; j < assembly.joints.size(); ++j) {
    if (!flags[j] || assembly.joints[j].parentPart != part.name)
      continue;
    const int child = assembly.indexOfPart(assembly.joints[j].childPart);
    if (child < 0)
      continue;
    emitJoint(out, assembly, (int)j, child, flags, indent + "    ");
  }
  // loop-closing joints attached to this part (SolidReference end points)
  for (size_t j = 0; j < assembly.joints.size(); ++j) {
    if (flags[j] || assembly.joints[j].parentPart != part.name)
      continue;
    const int child = assembly.indexOfPart(assembly.joints[j].childPart);
    if (child < 0)
      continue;
    emitJoint(out, assembly, (int)j, child, flags, indent + "    ");
  }
  out << indent << "  ]\n";
  out << indent << "  physics Physics { density -1 mass 1 }\n";
  out << indent << "}";
}

std::string generateWbt(const Assembly &assembly, const std::string &robotName) {
  std::ostringstream out;
  out << "#VRML_SIM R2025a utf8\n";
  out << "# Generated by the Webots Robot Creator.\n";
  out << "# Loop-closing joints use SolidReference end points (closed kinematic chains).\n";
  out << "\n";
  out << "Robot {\n";
  out << "  name \"" << robotName << "\"\n";
  out << "  controller \"<none>\"\n";
  out << "  children [\n";

  const std::vector<bool> flags = spanningFlags(assembly);
  // root parts: never a tree child
  std::vector<bool> isTreeChild(assembly.parts.size(), false);
  for (size_t j = 0; j < assembly.joints.size(); ++j) {
    if (!flags[j])
      continue;
    const int child = assembly.indexOfPart(assembly.joints[j].childPart);
    if (child >= 0)
      isTreeChild[child] = true;
  }
  bool first = true;
  for (size_t i = 0; i < assembly.parts.size(); ++i) {
    if (isTreeChild[i])
      continue;
    if (!first)
      out << "\n";
    first = false;
    emitPart(out, assembly, (int)i, flags, "    ", -1);
  }
  out << "\n  ]\n";
  out << "}\n";
  return out.str();
}

}  // namespace wbrobotproto
