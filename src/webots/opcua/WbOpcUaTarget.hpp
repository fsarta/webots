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

// Description: resolution of OPC-UA mapping targets inside the Webots world.
//
// Supported target syntaxes:
//   device:<robot name>/<device name>/<tag>   motor/sensor runtime values
//     tags: targetPosition (rw), position|value (r), velocity (r), torque|force (r)
//   field:<DEF name>/<field>[/<sfnode field>][.x|.y|.z]   node field values
//     e.g. field:SHOULDER/HingeJointParameters/minStop

#ifndef WB_OPC_UA_TARGET_HPP
#define WB_OPC_UA_TARGET_HPP

#include <QtCore/QString>

#include "WbOpcUaValue.hpp"

class WbField;
class WbNode;

class WbOpcUaTarget {
public:
  // parse and resolve a target path against the current world
  static bool resolve(const QString &path, WbOpcUaTarget &target, QString &error);

  bool isValid() const { return mKind != INVALID; }
  const QString &path() const { return mPath; }

  // generic value access (converted to/from the mapping data type)
  bool read(wbopcua::Value &value) const;
  bool write(const wbopcua::Value &value, QString &error);

  // human readable description used in the mapping table
  QString describe() const;

private:
  enum Kind { INVALID, DEVICE, FIELD };

  bool resolveDevice(const QStringList &parts, QString &error);
  bool resolveField(const QStringList &parts, QString &error);

  Kind mKind = INVALID;
  QString mPath;
  // device targets
  QString mTag;
  class WbRobot *mRobot = nullptr;
  class WbDevice *mDevice = nullptr;
  // field targets
  WbField *mField = nullptr;
  WbNode *mFieldOwner = nullptr;
  QString mFieldName;
  int mComponent = -1;  // 0..2 for .x/.y/.z on vector fields
};

#endif
