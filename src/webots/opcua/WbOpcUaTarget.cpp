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

#include "WbOpcUaTarget.hpp"
#include "WbRotationalMotor.hpp"

#include "WbDevice.hpp"
#include "WbField.hpp"
#include "WbLinearMotor.hpp"
#include "WbMotor.hpp"
#include "WbNode.hpp"
#include "WbPositionSensor.hpp"
#include "WbRobot.hpp"
#include "WbSFBool.hpp"
#include "WbSFDouble.hpp"
#include "WbSFInt.hpp"
#include "WbSFString.hpp"
#include "WbSFVector3.hpp"
#include "WbSingleValue.hpp"
#include "WbVector3.hpp"
#include "WbWorld.hpp"

using namespace wbopcua;

bool WbOpcUaTarget::resolve(const QString &path, WbOpcUaTarget &target, QString &error) {
  target = WbOpcUaTarget();
  target.mPath = path;
  if (path.startsWith("device:")) {
    const QStringList parts = path.mid(7).split('/');
    if (parts.size() != 3) {
      error = QObject::tr("device target must be 'device:<robot>/<device>/<tag>'");
      return false;
    }
    return target.resolveDevice(parts, error);
  }
  if (path.startsWith("field:")) {
    const QStringList parts = path.mid(6).split('/');
    if (parts.size() < 2) {
      error = QObject::tr("field target must be 'field:<DEF>/<field>[/<field>][.x|.y|.z]'");
      return false;
    }
    return target.resolveField(parts, error);
  }
  error = QObject::tr("target must start with 'device:' or 'field:'");
  return false;
}

bool WbOpcUaTarget::resolveDevice(const QStringList &parts, QString &error) {
  WbWorld *world = WbWorld::instance();
  if (!world) {
    error = QObject::tr("no world loaded");
    return false;
  }
  WbRobot *robot = NULL;
  foreach (WbRobot *candidate, world->robots()) {
    if (candidate->name() == parts[0] || candidate->defName() == parts[0]) {
      robot = candidate;
      break;
    }
  }
  if (!robot) {
    error = QObject::tr("robot '%1' not found").arg(parts[0]);
    return false;
  }
  WbDevice *device = NULL;
  for (int i = 0; i < robot->deviceCount(); ++i) {
    if (robot->device(i)->deviceName() == parts[1]) {
      device = robot->device(i);
      break;
    }
  }
  if (!device) {
    error = QObject::tr("device '%1' not found in robot '%2'").arg(parts[1], parts[0]);
    return false;
  }
  static const QStringList knownTags = QStringList() << "targetPosition"
                                                     << "position"
                                                     << "value"
                                                     << "velocity"
                                                     << "torque"
                                                     << "force";
  if (!knownTags.contains(parts[2])) {
    error = QObject::tr("unknown device tag '%1' (expected one of %2)").arg(parts[2], knownTags.join(", "));
    return false;
  }
  mKind = DEVICE;
  mRobot = robot;
  mDevice = device;
  mTag = parts[2];
  return true;
}

bool WbOpcUaTarget::resolveField(const QStringList &parts, QString &error) {
  WbWorld *world = WbWorld::instance();
  if (!world || !world->root()) {
    error = QObject::tr("no world loaded");
    return false;
  }
  // find the node by DEF name (depth-first)
  WbNode *node = NULL;
  QList<WbNode *> stack;
  stack.append(world->root());
  while (!stack.isEmpty() && !node) {
    WbNode *current = stack.takeFirst();
    if (current->defName() == parts[0])
      node = current;
    else
      stack.append(current->subNodes(false));
  }
  if (!node) {
    error = QObject::tr("DEF node '%1' not found").arg(parts[0]);
    return false;
  }
  WbNode *owner = node;
  WbField *field = NULL;
  for (int i = 1; i < parts.size(); ++i) {
    QString name = parts[i];
    int component = -1;
    if (name.endsWith(".x") || name.endsWith(".y") || name.endsWith(".z")) {
      component = name.endsWith(".x") ? 0 : (name.endsWith(".y") ? 1 : 2);
      name.chop(2);
    }
    field = owner->findField(name);
    if (!field) {
      error = QObject::tr("field '%1' not found in %2").arg(name, owner->modelName());
      return false;
    }
    if (i == parts.size() - 1) {
      mComponent = component;
      break;
    }
    WbSFNode *sf = dynamic_cast<WbSFNode *>(field->value());
    if (!sf || !sf->value()) {
      error = QObject::tr("field '%1' of %2 is not a node field").arg(name, owner->modelName());
      return false;
    }
    owner = sf->value();
  }
  mKind = FIELD;
  mField = field;
  mFieldOwner = owner;
  mFieldName = field ? field->name() : QString();
  return true;
}

bool WbOpcUaTarget::read(Value &value) const {
  if (mKind == DEVICE) {
    WbMotor *motor = dynamic_cast<WbMotor *>(mDevice);
    WbPositionSensor *sensor = dynamic_cast<WbPositionSensor *>(mDevice);
    WbRotationalMotor *rotational = dynamic_cast<WbRotationalMotor *>(mDevice);
    WbLinearMotor *linear = dynamic_cast<WbLinearMotor *>(mDevice);
    if (mTag == "targetPosition") {
      if (!motor)
        return false;
      value = Value(motor->targetPosition());
      return true;
    }
    if (mTag == "position" || mTag == "value") {
      if (sensor) {
        value = Value(sensor->position());
        return true;
      }
      return false;
    }
    if (mTag == "velocity") {
      if (!motor)
        return false;
      value = Value(motor->currentVelocity());
      return true;
    }
    if (mTag == "torque") {
      if (!rotational)
        return false;
      value = Value(rotational->torque());
      return true;
    }
    if (mTag == "force") {
      if (!linear)
        return false;
      value = Value(linear->force());
      return true;
    }
    return false;
  }
  if (mKind == FIELD && mField) {
    WbSingleValue *single = dynamic_cast<WbSingleValue *>(mField->value());
    if (!single)
      return false;
    switch (mField->type()) {
      case WB_SF_FLOAT:
        value = Value(dynamic_cast<WbSFDouble *>(single)->value());
        return true;
      case WB_SF_INT32:
        value = Value((int32_t)dynamic_cast<WbSFInt *>(single)->value());
        return true;
      case WB_SF_BOOL:
        value = Value(dynamic_cast<WbSFBool *>(single)->value());
        return true;
      case WB_SF_STRING:
        value = Value(dynamic_cast<WbSFString *>(single)->value().toStdString());
        return true;
      case WB_SF_VEC3F: {
        const WbVector3 &v = dynamic_cast<WbSFVector3 *>(single)->value();
        if (mComponent == 0)
          value = Value(v.x());
        else if (mComponent == 1)
          value = Value(v.y());
        else if (mComponent == 2)
          value = Value(v.z());
        else {
          // whole vectors are exposed as their x component plus a warning-free fallback
          value = Value(v.x());
        }
        return true;
      }
      default:
        return false;
    }
  }
  return false;
}

bool WbOpcUaTarget::write(const Value &value, QString &error) {
  if (mKind == DEVICE) {
    WbMotor *motor = dynamic_cast<WbMotor *>(mDevice);
    if (mTag == "targetPosition") {
      if (!motor) {
        error = QObject::tr("tag 'targetPosition' requires a Motor device");
        return false;
      }
      motor->setTargetPosition(value.toDouble());
      return true;
    }
    error = QObject::tr("tag '%1' is read-only").arg(mTag);
    return false;
  }
  if (mKind == FIELD && mField) {
    WbSingleValue *single = dynamic_cast<WbSingleValue *>(mField->value());
    if (!single) {
      error = QObject::tr("field '%1' is not writable").arg(mFieldName);
      return false;
    }
    switch (mField->type()) {
      case WB_SF_FLOAT:
        dynamic_cast<WbSFDouble *>(single)->setValue(value.toDouble());
        return true;
      case WB_SF_INT32:
        dynamic_cast<WbSFInt *>(single)->setValue(value.int32Value());
        return true;
      case WB_SF_BOOL:
        dynamic_cast<WbSFBool *>(single)->setValue(value.toBool());
        return true;
      case WB_SF_STRING:
        dynamic_cast<WbSFString *>(single)->setValue(QString::fromStdString(value.toString()));
        return true;
      case WB_SF_VEC3F: {
        WbSFVector3 *vector = dynamic_cast<WbSFVector3 *>(single);
        WbVector3 v = vector->value();
        if (mComponent == 0)
          v.setX(value.toDouble());
        else if (mComponent == 1)
          v.setY(value.toDouble());
        else if (mComponent == 2)
          v.setZ(value.toDouble());
        else {
          error = QObject::tr("vector field '%1' needs a .x/.y/.z suffix to be written").arg(mFieldName);
          return false;
        }
        vector->setValue(v);
        return true;
      }
      default:
        error = QObject::tr("unsupported field type for '%1'").arg(mFieldName);
        return false;
    }
  }
  error = QObject::tr("invalid target");
  return false;
}

QString WbOpcUaTarget::describe() const {
  if (mKind == DEVICE)
    return QString("%1 (%2)").arg(mPath, mDevice ? mDevice->deviceName() : QString());
  if (mKind == FIELD)
    return QString("%1 (%2)").arg(mPath, mFieldOwner ? mFieldOwner->modelName() : QString());
  return mPath;
}
