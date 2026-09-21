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

// Description: interactive 3D manipulators for joint anchor and axis editing
//              (Fusion-360 style handles shown on the selected joint).
//
// The manipulator draws an anchor cross and an axis arrow on the selected joint
// and lets the user drag them in the 3D view; edits are undoable and mirrored
// into the Mechanism Editor property panel.

#ifndef WB_JOINT_MANIPULATOR_HPP
#define WB_JOINT_MANIPULATOR_HPP

#include <QtCore/QObject>
#include <QtCore/QPoint>

#include "WbRay.hpp"
#include "WbVector3.hpp"

class QMouseEvent;
class WbAffinePlane;
class WbBasicJoint;
class WbViewpoint;
struct WrDynamicMesh;
struct WrMaterial;
struct WrRenderable;
struct WrTransform;

class WbJointManipulator : public QObject {
  Q_OBJECT

public:
  explicit WbJointManipulator(WbViewpoint *viewpoint, QObject *parent = NULL);
  virtual ~WbJointManipulator();

  static WbJointManipulator *instance() { return cInstance; }

  // master switch (driven by the Mechanism Editor dock)
  void setActive(bool active);
  bool isActive() const { return mActive; }

  void setJoint(WbBasicJoint *joint);
  WbBasicJoint *joint() const { return mJoint; }

  // mouse hooks called from WbView3D; return true when the event was consumed
  bool mousePressEvent(QMouseEvent *event);
  bool mouseMoveEvent(QMouseEvent *event);
  bool mouseReleaseEvent(QMouseEvent *event);

  // redraw the handles (call after field edits or every physics step)
  void refresh();

public slots:
  void updateFromSelection();

private slots:
  void jointChanged(long jointId);

private:
  enum Handle { NO_HANDLE, ANCHOR_HANDLE, AXIS_HANDLE };

  Handle pickHandle(const QPoint &pos) const;
  void updateHandlesVisibility();
  void rebuildMesh();
  void pushUndo();
  WbVector3 anchorWorld() const;
  WbVector3 axisWorld() const;
  WbVector3 toParentFrame(const WbVector3 &worldPoint) const;
  WbVector3 directionToParentFrame(const WbVector3 &worldDirection) const;

  static WbJointManipulator *cInstance;

  WbViewpoint *mViewpoint;
  WbBasicJoint *mJoint;
  bool mActive;
  Handle mDragHandle;
  WbRay mDragRay;
  WbVector3 mDragPlaneNormal;
  WbVector3 mDragPlanePoint;
  WbVector3 mInitialAnchor;
  WbVector3 mInitialAxis;
  bool mDragging;

  WrTransform *mTransform;
  WrRenderable *mRenderable;
  WrDynamicMesh *mMesh;
  WrMaterial *mMaterial;
};

#endif
