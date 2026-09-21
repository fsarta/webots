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

// Description: Qt bridge between the live Webots node tree and the kinematic
//              graph used by the Mechanism Editor (CAD-like joints/links tool
//              with closed kinematic chain support).

#ifndef WB_MECHANISM_MODEL_HPP
#define WB_MECHANISM_MODEL_HPP

#include <QtCore/QHash>
#include <QtCore/QObject>
#include <QtCore/QPointF>
#include <QtCore/QPointer>
#include <QtCore/QString>

#include "WbKinematicGraphCore.hpp"
#include "WbVector3.hpp"

class WbBasicJoint;
class WbField;
class WbNode;
class WbSolid;

class WbMechanismModel : public QObject {
  Q_OBJECT

public:
  static WbMechanismModel *instance();
  static void cleanup();

  // rebuild the kinematic graph from the current world
  void rebuild();

  const wbmechanism::Graph &graph() const { return mGraph; }
  // layered layout including user overrides
  wbmechanism::Layout layout() const;
  wbmechanism::Analysis analyze(bool planar = false) const;

  // mapping between graph ids and live nodes
  WbSolid *solidOfLink(long linkId) const;
  WbBasicJoint *jointOfJoint(long jointId) const;
  long linkIdOfSolid(const WbSolid *solid) const;
  long jointIdOfJoint(const WbBasicJoint *joint) const;

  // schematic (canvas) positions overridden by the user, by link name
  bool hasUserPosition(const QString &linkName) const;
  QPointF userPosition(const QString &linkName) const;
  void setUserPosition(const QString &linkName, const QPointF &position);
  void resetUserLayout();

  // ---------------- edit operations ----------------
  // Topology changes (add/delete/connect) require the simulation to be paused:
  // they restructure the node tree and would invalidate live physics objects.
  static bool topologyEditable(QString *reason = NULL);

  // create a new joint hanging from `parentLinkId` with a fresh inline Solid endpoint
  bool addLinkAndJoint(long parentLinkId, wbmechanism::JointType type, const QString &newLinkName, QString &error);
  // connect two existing links with a joint whose endpoint is a SolidReference;
  // when both links are already connected this closes a kinematic loop
  bool connectLinks(long parentLinkId, long childLinkId, wbmechanism::JointType type, QString &error);
  // delete a joint (and its inline endpoint subtree when it owns one)
  bool deleteJoint(long jointId, QString &error);

  struct JointProperties {
    QString name;
    wbmechanism::JointType type;
    WbVector3 axis;
    WbVector3 anchor;
    bool hasAnchor;
    bool hasLimits;
    double minStop;
    double maxStop;
    double springConstant;
    double dampingConstant;
    double position;
    bool isLoopClosure;
    QString endPointName;
  };
  bool jointProperties(long jointId, JointProperties &properties) const;

  // undoable field edits (same mechanism as the Scene Tree value editors)
  bool setLinkName(long linkId, const QString &name, QString &error);
  bool setJointAxis(long jointId, const WbVector3 &axis, QString &error);
  bool setJointAnchor(long jointId, const WbVector3 &anchor, QString &error);
  bool setJointLimits(long jointId, bool hasLimits, double minStop, double maxStop, QString &error);
  bool setJointSpringDamping(long jointId, double springConstant, double dampingConstant, QString &error);

  // forward a node selection to the Scene Tree
  void requestSelection(WbNode *node) { emit selectionRequested(node); }

signals:
  void modelReset();
  void layoutChanged();
  void jointChanged(long jointId);
  // forward a node selection to the Scene Tree
  void selectionRequested(WbNode *node);

private slots:
  void worldUpdated();

private:
  explicit WbMechanismModel(QObject *parent = NULL);
  virtual ~WbMechanismModel();
  static WbMechanismModel *cInstance;

  void collectNode(WbNode *node, long parentLinkId);
  WbField *parameterField(long jointId, const QString &fieldName, QString &error) const;
  WbField *valueField(WbNode *node, const QString &fieldName, QString &error) const;
  static void applyFieldEdit(WbField *field, const WbVariant &next);
  static void markWorldModified();
  void clearMaps();

  wbmechanism::Graph mGraph;
  QHash<long, QPointer<WbSolid> > mSolids;
  QHash<long, QPointer<WbBasicJoint> > mJoints;
  QHash<const WbSolid *, long> mLinkIds;
  QHash<const WbBasicJoint *, long> mJointIds;
  QHash<QString, QPointF> mUserPositions;
};

#endif
