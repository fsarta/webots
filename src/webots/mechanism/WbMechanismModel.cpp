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

#include "WbMechanismModel.hpp"
#include "WbBaseNode.hpp"

#include <QtCore/QPointF>
#include <QtCore/QSet>

#include "WbBasicJoint.hpp"
#include "WbEditCommand.hpp"
#include "WbField.hpp"
#include "WbGroup.hpp"
#include "WbJointParameters.hpp"
#include "WbMFNode.hpp"
#include "WbNodeFactory.hpp"
#include "WbSFDouble.hpp"
#include "WbSFNode.hpp"
#include "WbSFString.hpp"
#include "WbSFVector3.hpp"
#include "WbSingleValue.hpp"
#include "WbSimulationState.hpp"
#include "WbSolid.hpp"
#include "WbSolidReference.hpp"
#include "WbUndoStack.hpp"
#include "WbVariant.hpp"
#include "WbVector3.hpp"
#include "WbWorld.hpp"

WbMechanismModel *WbMechanismModel::cInstance = NULL;

WbMechanismModel *WbMechanismModel::instance() {
  if (!cInstance)
    cInstance = new WbMechanismModel();
  return cInstance;
}

void WbMechanismModel::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

WbMechanismModel::WbMechanismModel(QObject *parent) : QObject(parent) {
  // world content can change from many places: notify when the mode changes
  connect(WbSimulationState::instance(), &WbSimulationState::modeChanged, this, &WbMechanismModel::worldUpdated,
          Qt::UniqueConnection);
}

WbMechanismModel::~WbMechanismModel() {
}

void WbMechanismModel::worldUpdated() {
  // the graph is rebuilt on demand by the dock; just notify that contents may be stale
  emit layoutChanged();
}

void WbMechanismModel::clearMaps() {
  mGraph.clear();
  mSolids.clear();
  mJoints.clear();
  mLinkIds.clear();
  mJointIds.clear();
}

void WbMechanismModel::rebuild() {
  clearMaps();
  WbWorld *world = WbWorld::instance();
  if (!world || !world->root())
    return;
  collectNode(world->root(), -1);
  emit modelReset();
}

void WbMechanismModel::collectNode(WbNode *node, long parentLinkId) {
  if (!node)
    return;

  WbSolid *solid = dynamic_cast<WbSolid *>(node);
  if (solid) {
    long id = linkIdOfSolid(solid);
    if (id < 0) {
      id = mGraph.addLink(solid->name().toStdString(), (solid->physics() == NULL));
      mSolids[id] = solid;
      mLinkIds[solid] = id;
    }
    parentLinkId = id;
  }

  WbBasicJoint *joint = dynamic_cast<WbBasicJoint *>(node);
  if (joint) {
    WbSolid *parentSolid = joint->solidParent();
    long parent = parentSolid ? linkIdOfSolid(parentSolid) : parentLinkId;
    WbSolidReference *reference = joint->solidReference();
    WbSolid *childSolid = NULL;
    if (reference)
      childSolid = reference->solid();
    else
      childSolid = joint->solidEndPoint();
    bool isLoopClosure = reference != NULL;
    if (parent >= 0 && childSolid) {
      long child = linkIdOfSolid(childSolid);
      if (child < 0) {
        // endpoint solid not registered yet (e.g. forward reference): register it now
        const long id = mGraph.addLink(childSolid->name().toStdString(), (childSolid->physics() == NULL));
        mSolids[id] = childSolid;
        mLinkIds[childSolid] = id;
        child = id;
      }
      wbmechanism::JointType type = wbmechanism::REVOLUTE_JOINT;
      const QString modelName = joint->modelName();
      if (modelName == "HingeJoint")
        type = wbmechanism::REVOLUTE_JOINT;
      else if (modelName == "SliderJoint")
        type = wbmechanism::PRISMATIC_JOINT;
      else if (modelName == "Hinge2Joint")
        type = wbmechanism::REVOLUTE2_JOINT;
      else if (modelName == "BallJoint")
        type = wbmechanism::SPHERICAL_JOINT;
      const long jid = mGraph.addJoint(joint->modelName().toStdString() + std::string("_") +
                                       std::to_string(mGraph.jointCount()),
                                       type, parent, child, isLoopClosure);
      wbmechanism::Joint &gj = *mGraph.joint(jid);
      if (!joint->solidEndPoint() && !reference)
        gj.name += " (detached)";
      mJoints[jid] = joint;
      mJointIds[joint] = jid;
      // fill graph parameters for the analysis panel
      WbField *paramsField = joint->findField("jointParameters");
      WbSFNode *paramsNode = paramsField ? dynamic_cast<WbSFNode *>(paramsField->value()) : NULL;
      WbNode *params = paramsNode ? paramsNode->value() : NULL;
      if (params) {
        WbSFVector3 *axis = params->findSFVector3("axis");
        if (axis) {
          const WbVector3 &a = axis->value();
          gj.axis[0] = a.x();
          gj.axis[1] = a.y();
          gj.axis[2] = a.z();
        }
        WbSFDouble *minStop = params->findSFDouble("minStop");
        WbSFDouble *maxStop = params->findSFDouble("maxStop");
        WbSFDouble *spring = params->findSFDouble("springConstant");
        WbSFDouble *damping = params->findSFDouble("dampingConstant");
        if (minStop && maxStop) {
          gj.hasLimits = true;
          gj.minStop = minStop->value();
          gj.maxStop = maxStop->value();
        }
        if (spring)
          gj.springConstant = spring->value();
        if (damping)
          gj.dampingConstant = damping->value();
      }
      // inline endpoints continue the walk under this joint; loop closures do not own their solid
      if (childSolid && !isLoopClosure)
        collectNode(childSolid, child);
      return;
    }
  }

  // groups (incl. Pose/Slot/Robot containers): recurse over children
  WbGroup *group = dynamic_cast<WbGroup *>(node);
  if (group) {
    const WbMFNode &children = group->children();
    for (int i = 0; i < children.size(); ++i)
      collectNode(children.item(i), parentLinkId);
    return;
  }

  // generic node with possible child nodes (fields of type SFNode/MFNode)
  const QList<WbField *> fields = node->fields();
  for (int i = 0; i < fields.size(); ++i) {
    WbMFNode *mf = dynamic_cast<WbMFNode *>(fields[i]->value());
    if (mf) {
      for (int j = 0; j < mf->size(); ++j)
        collectNode(mf->item(j), parentLinkId);
    } else {
      WbSFNode *sf = dynamic_cast<WbSFNode *>(fields[i]->value());
      if (sf && sf->value())
        collectNode(sf->value(), parentLinkId);
    }
  }
}

wbmechanism::Layout WbMechanismModel::layout() const {
  wbmechanism::Layout computed = wbmechanism::computeLayout(mGraph);
  // apply user overrides
  for (int i = 0; i < mGraph.links().size(); ++i) {
    const wbmechanism::Link &link = mGraph.links()[i];
    const QHash<QString, QPointF>::const_iterator it = mUserPositions.find(QString::fromStdString(link.name));
    if (it != mUserPositions.constEnd())
      computed.linkPositions[link.id] = std::make_pair(it->x(), it->y());
  }
  // recompute joint glyph positions on the middle of their edges
  for (int i = 0; i < mGraph.joints().size(); ++i) {
    const wbmechanism::Joint &j = mGraph.joints()[i];
    std::map<long, std::pair<double, double> >::const_iterator itp = computed.linkPositions.find(j.parentLinkId);
    std::map<long, std::pair<double, double> >::const_iterator itc = computed.linkPositions.find(j.childLinkId);
    if (itp != computed.linkPositions.end() && itc != computed.linkPositions.end())
      computed.jointPositions[j.id] = std::make_pair(0.5 * (itp->second.first + itc->second.first),
                                                    0.5 * (itp->second.second + itc->second.second));
  }
  return computed;
}

wbmechanism::Analysis WbMechanismModel::analyze(bool planar) const {
  return wbmechanism::analyze(mGraph, planar);
}

WbSolid *WbMechanismModel::solidOfLink(long linkId) const {
  return mSolids.value(linkId, NULL);
}

WbBasicJoint *WbMechanismModel::jointOfJoint(long jointId) const {
  return mJoints.value(jointId, NULL);
}

long WbMechanismModel::linkIdOfSolid(const WbSolid *solid) const {
  return mLinkIds.value(solid, -1);
}

long WbMechanismModel::jointIdOfJoint(const WbBasicJoint *joint) const {
  return mJointIds.value(joint, -1);
}

bool WbMechanismModel::hasUserPosition(const QString &linkName) const {
  return mUserPositions.contains(linkName);
}

QPointF WbMechanismModel::userPosition(const QString &linkName) const {
  return mUserPositions.value(linkName);
}

void WbMechanismModel::setUserPosition(const QString &linkName, const QPointF &position) {
  mUserPositions[linkName] = position;
  emit layoutChanged();
}

void WbMechanismModel::resetUserLayout() {
  mUserPositions.clear();
  emit layoutChanged();
}

bool WbMechanismModel::topologyEditable(QString *reason) {
  const bool editable = WbSimulationState::instance()->isPaused();
  if (!editable && reason)
    *reason = QObject::tr("Pause the simulation to add or remove joints and links.");
  return editable;
}

void WbMechanismModel::markWorldModified() {
  WbWorld *world = WbWorld::instance();
  if (world)
    world->setModifiedFromSceneTree();
}

WbField *WbMechanismModel::valueField(WbNode *node, const QString &fieldName, QString &error) const {
  if (!node) {
    error = tr("invalid node");
    return NULL;
  }
  WbField *field = node->findField(fieldName);
  if (!field)
    error = tr("field '%1' not found in %2").arg(fieldName, node->modelName());
  return field;
}

WbField *WbMechanismModel::parameterField(long jointId, const QString &fieldName, QString &error) const {
  WbBasicJoint *joint = jointOfJoint(jointId);
  if (!joint) {
    error = tr("invalid joint");
    return NULL;
  }
  WbSFNode *paramsNode = joint->findSFNode("jointParameters");
  WbNode *params = paramsNode ? paramsNode->value() : NULL;
  if (!params) {
    error = tr("%1 has no jointParameters node").arg(joint->modelName());
    return NULL;
  }
  return valueField(params, fieldName, error);
}

void WbMechanismModel::applyFieldEdit(WbField *field, const WbVariant &next) {
  WbSingleValue *single = dynamic_cast<WbSingleValue *>(field->value());
  if (!single)
    return;
  WbUndoStack::instance()->push(new WbEditCommand(single, single->variantValue(), next));
}

bool WbMechanismModel::addLinkAndJoint(long parentLinkId, wbmechanism::JointType type, const QString &newLinkName,
                                       QString &error) {
  QString reason;
  if (!topologyEditable(&reason)) {
    error = reason;
    return false;
  }
  WbSolid *parent = solidOfLink(parentLinkId);
  if (!parent) {
    error = tr("invalid parent link");
    return false;
  }
  WbField *childrenField = parent->findField("children");
  WbMFNode *children = childrenField ? dynamic_cast<WbMFNode *>(childrenField->value()) : NULL;
  if (!children) {
    error = tr("%1 has no 'children' field").arg(parent->modelName());
    return false;
  }

  const QString modelName = QString::fromUtf8(wbmechanism::jointTypeName(type));
  if (modelName == "SolidJoint") {
    error = tr("Rigid joints are not supported: use a nested Solid instead.");
    return false;
  }
  WbNode *jointNode = WbNodeFactory::instance()->createNode(modelName);
  WbBasicJoint *joint = dynamic_cast<WbBasicJoint *>(jointNode);
  if (!joint) {
    delete jointNode;
    error = tr("cannot create %1").arg(modelName);
    return false;
  }
  WbNode *solidNode = WbNodeFactory::instance()->createNode("Solid");
  WbSolid *solid = dynamic_cast<WbSolid *>(solidNode);
  if (!solid) {
    delete jointNode;
    delete solidNode;
    error = tr("cannot create Solid");
    return false;
  }
  // name the new link
  WbSFString *nameField = solid->findSFString("name");
  if (nameField && !newLinkName.isEmpty())
    nameField->setValue(newLinkName);
  // attach the fresh Solid as endpoint of the joint
  WbSFNode *endPoint = joint->findSFNode("endPoint");
  if (!endPoint) {
    delete jointNode;
    delete solidNode;
    error = tr("%1 has no 'endPoint' field").arg(modelName);
    return false;
  }
  endPoint->setValue(solid);
  // insert the joint into the parent link
  children->addItem(jointNode);
  dynamic_cast<WbBaseNode *>(jointNode)->preFinalize();
  dynamic_cast<WbBaseNode *>(jointNode)->postFinalize();
  markWorldModified();
  rebuild();
  return true;
}

bool WbMechanismModel::connectLinks(long parentLinkId, long childLinkId, wbmechanism::JointType type, QString &error) {
  QString reason;
  if (!topologyEditable(&reason)) {
    error = reason;
    return false;
  }
  WbSolid *parent = solidOfLink(parentLinkId);
  WbSolid *child = solidOfLink(childLinkId);
  if (!parent || !child) {
    error = tr("invalid links");
    return false;
  }
  if (parent == child) {
    error = tr("a link cannot be connected to itself");
    return false;
  }
  if (child->name().isEmpty()) {
    error = tr("link '%1' must be named before it can be referenced").arg(child->modelName());
    return false;
  }
  WbField *childrenField = parent->findField("children");
  WbMFNode *children = childrenField ? dynamic_cast<WbMFNode *>(childrenField->value()) : NULL;
  if (!children) {
    error = tr("%1 has no 'children' field").arg(parent->modelName());
    return false;
  }

  const QString modelName = QString::fromUtf8(wbmechanism::jointTypeName(type));
  if (modelName == "SolidJoint") {
    error = tr("Rigid connections between existing links are not supported yet.");
    return false;
  }
  WbNode *jointNode = WbNodeFactory::instance()->createNode(modelName);
  WbBasicJoint *joint = dynamic_cast<WbBasicJoint *>(jointNode);
  if (!joint) {
    delete jointNode;
    error = tr("cannot create %1").arg(modelName);
    return false;
  }
  // endpoint: SolidReference to the existing link (this is what closes kinematic loops)
  WbSFNode *endPoint = joint->findSFNode("endPoint");
  if (!endPoint) {
    delete jointNode;
    error = tr("%1 has no 'endPoint' field").arg(modelName);
    return false;
  }
  WbNode *referenceNode = WbNodeFactory::instance()->createNode("SolidReference");
  WbSolidReference *reference = dynamic_cast<WbSolidReference *>(referenceNode);
  if (!reference) {
    delete jointNode;
    delete referenceNode;
    error = tr("cannot create SolidReference");
    return false;
  }
  WbSFString *solidName = reference->findSFString("solidName");
  if (solidName)
    solidName->setValue(child->name());
  endPoint->setValue(reference);
  children->addItem(jointNode);
  dynamic_cast<WbBaseNode *>(jointNode)->preFinalize();
  dynamic_cast<WbBaseNode *>(jointNode)->postFinalize();
  markWorldModified();
  rebuild();
  return true;
}

bool WbMechanismModel::deleteJoint(long jointId, QString &error) {
  QString reason;
  if (!topologyEditable(&reason)) {
    error = reason;
    return false;
  }
  WbBasicJoint *joint = jointOfJoint(jointId);
  if (!joint) {
    error = tr("invalid joint");
    return false;
  }
  // the joint lives in the 'children' field of its parent solid (or in a Slot)
  WbSolid *parent = joint->solidParent();
  WbNode *owner = parent ? (WbNode *)parent : joint->parentNode();
  int index = -1;
  WbMFNode *children = NULL;
  if (owner) {
    WbField *childrenField = owner->findField("children");
    children = childrenField ? dynamic_cast<WbMFNode *>(childrenField->value()) : NULL;
    if (children)
      for (int i = 0; i < children->size(); ++i)
        if (children->item(i) == joint) {
          index = i;
          break;
        }
  }
  if (index < 0 || !children) {
    error = tr("cannot locate joint '%1' in its parent 'children' field").arg(joint->modelName());
    return false;
  }
  children->removeItem(index);
  markWorldModified();
  rebuild();
  return true;
}

bool WbMechanismModel::jointProperties(long jointId, JointProperties &properties) const {
  WbBasicJoint *joint = jointOfJoint(jointId);
  if (!joint)
    return false;
  properties.name = joint->modelName();
  const wbmechanism::Joint *gj = mGraph.joint(jointId);
  properties.type = gj ? gj->type : wbmechanism::REVOLUTE_JOINT;
  properties.isLoopClosure = joint->solidReference() != NULL;
  properties.position = gj ? gj->position : 0.0;
  properties.hasAnchor = false;
  properties.hasLimits = false;
  properties.minStop = properties.maxStop = 0.0;
  properties.springConstant = properties.dampingConstant = 0.0;
  properties.axis = WbVector3(0.0, 0.0, 1.0);
  properties.anchor = WbVector3();

  QString error;
  WbField *axisField = parameterField(jointId, "axis", error);
  WbSFVector3 *axis = axisField ? dynamic_cast<WbSFVector3 *>(axisField->value()) : NULL;
  if (axis)
    properties.axis = axis->value();

  WbField *anchorField = parameterField(jointId, "anchor", error);
  WbSFVector3 *anchor = anchorField ? dynamic_cast<WbSFVector3 *>(anchorField->value()) : NULL;
  if (anchor) {
    properties.anchor = anchor->value();
    properties.hasAnchor = true;
  }

  WbField *minField = parameterField(jointId, "minStop", error);
  WbField *maxField = parameterField(jointId, "maxStop", error);
  WbSFDouble *minStop = minField ? dynamic_cast<WbSFDouble *>(minField->value()) : NULL;
  WbSFDouble *maxStop = maxField ? dynamic_cast<WbSFDouble *>(maxField->value()) : NULL;
  if (minStop && maxStop) {
    properties.hasLimits = true;
    properties.minStop = minStop->value();
    properties.maxStop = maxStop->value();
  }

  WbField *springField = parameterField(jointId, "springConstant", error);
  WbSFDouble *spring = springField ? dynamic_cast<WbSFDouble *>(springField->value()) : NULL;
  if (spring)
    properties.springConstant = spring->value();
  WbField *dampingField = parameterField(jointId, "dampingConstant", error);
  WbSFDouble *damping = dampingField ? dynamic_cast<WbSFDouble *>(dampingField->value()) : NULL;
  if (damping)
    properties.dampingConstant = damping->value();

  if (properties.isLoopClosure) {
    WbSolidReference *reference = joint->solidReference();
    properties.endPointName = reference ? reference->name() : QString();
  } else {
    WbSolid *endPoint = joint->solidEndPoint();
    properties.endPointName = endPoint ? endPoint->name() : QString();
  }
  return true;
}

bool WbMechanismModel::setLinkName(long linkId, const QString &name, QString &error) {
  WbSolid *solid = solidOfLink(linkId);
  if (!solid) {
    error = tr("invalid link");
    return false;
  }
  WbField *field = valueField(solid, "name", error);
  if (!field)
    return false;
  applyFieldEdit(field, WbVariant(name));
  markWorldModified();
  return true;
}

bool WbMechanismModel::setJointAxis(long jointId, const WbVector3 &axis, QString &error) {
  WbField *field = parameterField(jointId, "axis", error);
  if (!field)
    return false;
  if (axis.length() == 0.0) {
    error = tr("the axis vector cannot be null");
    return false;
  }
  applyFieldEdit(field, WbVariant(axis));
  markWorldModified();
  emit jointChanged(jointId);
  return true;
}

bool WbMechanismModel::setJointAnchor(long jointId, const WbVector3 &anchor, QString &error) {
  WbField *field = parameterField(jointId, "anchor", error);
  if (!field)
    return false;
  applyFieldEdit(field, WbVariant(anchor));
  markWorldModified();
  emit jointChanged(jointId);
  return true;
}

bool WbMechanismModel::setJointLimits(long jointId, bool hasLimits, double minStop, double maxStop, QString &error) {
  WbField *minField = parameterField(jointId, "minStop", error);
  WbField *maxField = parameterField(jointId, "maxStop", error);
  if (!minField || !maxField)
    return false;
  if (hasLimits && minStop >= maxStop) {
    error = tr("minStop must be smaller than maxStop");
    return false;
  }
  applyFieldEdit(minField, WbVariant(hasLimits ? minStop : 0.0));
  applyFieldEdit(maxField, WbVariant(hasLimits ? maxStop : 0.0));
  markWorldModified();
  emit jointChanged(jointId);
  return true;
}

bool WbMechanismModel::setJointSpringDamping(long jointId, double springConstant, double dampingConstant,
                                             QString &error) {
  WbField *springField = parameterField(jointId, "springConstant", error);
  WbField *dampingField = parameterField(jointId, "dampingConstant", error);
  if (!springField || !dampingField)
    return false;
  applyFieldEdit(springField, WbVariant(springConstant));
  applyFieldEdit(dampingField, WbVariant(dampingConstant));
  markWorldModified();
  emit jointChanged(jointId);
  return true;
}
