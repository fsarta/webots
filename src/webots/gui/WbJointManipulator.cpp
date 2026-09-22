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

#include "WbJointManipulator.hpp"
#include <wren/material.h>
#include <wren/node.h>
#include <wren/transform.h>

#include <QtCore/QCoreApplication>
#include <QtGui/QMouseEvent>

#include <wren/dynamic_mesh.h>
#include <wren/material.h>
#include <wren/renderable.h>
#include <wren/scene.h>
#include <wren/transform.h>

#include "WbAffinePlane.hpp"
#include "WbBasicJoint.hpp"
#include "WbEditCommand.hpp"
#include "WbField.hpp"
#include "WbJointParameters.hpp"
#include "WbMatrix4.hpp"
#include "WbSFNode.hpp"
#include "WbSFVector3.hpp"
#include "WbSelection.hpp"
#include "WbSingleValue.hpp"
#include "WbSolid.hpp"
#include "WbUndoStack.hpp"
#include "WbVariant.hpp"
#include "WbViewpoint.hpp"
#include "WbWrenRenderingContext.hpp"
#include "WbWrenShaders.hpp"

static const double HANDLE_RADIUS = 0.035;
static const double AXIS_LENGTH = 0.22;

static double distanceToRay(const WbRay &ray, const WbVector3 &point) {
  const WbVector3 toPoint = point - ray.origin();
  const double along = toPoint.dot(ray.direction());
  const WbVector3 closest = ray.origin() + along * ray.direction();
  return (closest - point).length();
}

WbJointManipulator *WbJointManipulator::cInstance = NULL;

WbJointManipulator::WbJointManipulator(WbViewpoint *viewpoint, QObject *parent) :
  QObject(parent),
  mViewpoint(viewpoint),
  mJoint(NULL),
  mActive(false),
  mDragHandle(NO_HANDLE),
  mDragging(false),
  mTransform(NULL),
  mRenderable(NULL),
  mMesh(NULL),
  mMaterial(NULL) {
  cInstance = this;

  // Wren overlay: a line-set drawing the handles
  mMaterial = wr_phong_material_new();
  const float color[3] = {0.95f, 0.45f, 0.05f};
  wr_phong_material_set_color(mMaterial, color);
  wr_material_set_default_program(mMaterial, WbWrenShaders::lineSetShader());

  mMesh = wr_dynamic_mesh_new(false, false, false);
  mRenderable = wr_renderable_new();
  wr_renderable_set_cast_shadows(mRenderable, false);
  wr_renderable_set_receive_shadows(mRenderable, false);
  wr_renderable_set_mesh(mRenderable, WR_MESH(mMesh));
  wr_renderable_set_material(mRenderable, mMaterial, NULL);
  wr_renderable_set_drawing_mode(mRenderable, WR_RENDERABLE_DRAWING_MODE_LINES);
  wr_renderable_set_visibility_flags(mRenderable, WbWrenRenderingContext::VF_JOINT_AXES);
  wr_renderable_set_drawing_order(mRenderable, WR_RENDERABLE_DRAWING_ORDER_AFTER_1);

  mTransform = wr_transform_new();
  wr_transform_attach_child(mTransform, WR_NODE(mRenderable));
  WrTransform *root = wr_scene_get_root(wr_scene_get_instance());
  wr_transform_attach_child(root, WR_NODE(mTransform));
  wr_node_set_visible(WR_NODE(mTransform), false);

  if (WbSelection::instance()) {
    connect(WbSelection::instance(), &WbSelection::selectionChangedFromSceneTree, this,
            &WbJointManipulator::updateFromSelection);
    connect(WbSelection::instance(), &WbSelection::selectionChangedFromView3D, this,
            &WbJointManipulator::updateFromSelection);
  }
}

WbJointManipulator::~WbJointManipulator() {
  if (mTransform)
    wr_node_set_visible(WR_NODE(mTransform), false);
  if (mRenderable)
    wr_node_delete(WR_NODE(mRenderable));
  if (mMesh)
    wr_dynamic_mesh_delete(mMesh);
  if (mMaterial)
    wr_material_delete(mMaterial);
  if (mTransform)
    wr_node_delete(WR_NODE(mTransform));
  cInstance = NULL;
}

void WbJointManipulator::setActive(bool active) {
  mActive = active;
  if (!active) {
    mDragHandle = NO_HANDLE;
    mDragging = false;
  }
  if (active)
    updateFromSelection();
  updateHandlesVisibility();
  refresh();
}

void WbJointManipulator::setJoint(WbBasicJoint *joint) {
  mJoint = joint;
  mDragHandle = NO_HANDLE;
  mDragging = false;
  updateHandlesVisibility();
  refresh();
}

void WbJointManipulator::updateFromSelection() {
  if (!mActive)
    return;
  WbBasicJoint *joint = dynamic_cast<WbBasicJoint *>(WbSelection::instance()->selectedNode());
  setJoint(joint);
}

void WbJointManipulator::jointChanged(long jointId) {
  Q_UNUSED(jointId);
  refresh();
}

void WbJointManipulator::updateHandlesVisibility() {
  const bool visible = mActive && mJoint;
  if (mTransform)
    wr_node_set_visible(WR_NODE(mTransform), visible);
}

WbVector3 WbJointManipulator::anchorWorld() const {
  if (!mJoint)
    return WbVector3();
  WbVector3 localAnchor;
  WbVector3 localAxis;
  mJoint->getAnchorAndAxis(localAnchor, localAxis);
  WbSolid *parent = mJoint->solidParent();
  return parent ? parent->matrix() * localAnchor : localAnchor;
}

WbVector3 WbJointManipulator::axisWorld() const {
  if (!mJoint)
    return WbVector3(0.0, 0.0, 1.0);
  WbVector3 localAnchor;
  WbVector3 localAxis;
  mJoint->getAnchorAndAxis(localAnchor, localAxis);
  WbSolid *parent = mJoint->solidParent();
  if (!parent)
    return localAxis.normalized();
  const WbVector3 origin = parent->matrix() * WbVector3();
  const WbVector3 tip = parent->matrix() * localAxis;
  return (tip - origin).normalized();
}

WbVector3 WbJointManipulator::toParentFrame(const WbVector3 &worldPoint) const {
  WbSolid *parent = mJoint ? mJoint->solidParent() : NULL;
  if (!parent)
    return worldPoint;
  return parent->matrix().pseudoInversed() * worldPoint;
}

WbVector3 WbJointManipulator::directionToParentFrame(const WbVector3 &worldDirection) const {
  WbSolid *parent = mJoint ? mJoint->solidParent() : NULL;
  if (!parent)
    return worldDirection;
  const WbMatrix4 inverse = parent->matrix().pseudoInversed();
  const WbVector3 o = inverse * WbVector3();
  const WbVector3 t = inverse * worldDirection;
  return (t - o).normalized();
}

void WbJointManipulator::rebuildMesh() {
  if (!mMesh)
    return;
  wr_dynamic_mesh_clear(mMesh);
  if (!mJoint)
    return;

  const WbVector3 a = anchorWorld();
  const WbVector3 x = axisWorld();
  // an arbitrary perpendicular to draw the anchor cross
  WbVector3 up = qFuzzyCompare(x.z(), 1.0) ? WbVector3(1.0, 0.0, 0.0) : WbVector3(0.0, 0.0, 1.0);
  const WbVector3 y = x.cross(up).normalized();
  const WbVector3 z = x.cross(y).normalized();
  const double scale = mViewpoint ? mViewpoint->viewDistanceUnscaling(a) : 1.0;
  const double r = HANDLE_RADIUS * scale;
  const double len = AXIS_LENGTH * scale;

  int index = 0;
  // anchor cross
  const WbVector3 crossEnds[6] = {a + r * x,  a - r * x,  a + r * y,
                                  a - r * y,  a + r * z,  a - r * z};
  for (int i = 0; i < 6; i += 2) {
    const float p0[3] = {(float)crossEnds[i].x(), (float)crossEnds[i].y(), (float)crossEnds[i].z()};
    const float p1[3] = {(float)crossEnds[i + 1].x(), (float)crossEnds[i + 1].y(), (float)crossEnds[i + 1].z()};
    wr_dynamic_mesh_add_vertex(mMesh, p0);
    wr_dynamic_mesh_add_index(mMesh, index++);
    wr_dynamic_mesh_add_vertex(mMesh, p1);
    wr_dynamic_mesh_add_index(mMesh, index++);
  }
  // axis line with arrow head
  const WbVector3 tip = a + len * x;
  const float pa[3] = {(float)a.x(), (float)a.y(), (float)a.z()};
  const float pt[3] = {(float)tip.x(), (float)tip.y(), (float)tip.z()};
  wr_dynamic_mesh_add_vertex(mMesh, pa);
  wr_dynamic_mesh_add_index(mMesh, index++);
  wr_dynamic_mesh_add_vertex(mMesh, pt);
  wr_dynamic_mesh_add_index(mMesh, index++);
  const WbVector3 head1 = tip - 0.25 * len * x + 0.06 * len * y;
  const WbVector3 head2 = tip - 0.25 * len * x - 0.06 * len * y;
  const float ph1[3] = {(float)head1.x(), (float)head1.y(), (float)head1.z()};
  const float ph2[3] = {(float)head2.x(), (float)head2.y(), (float)head2.z()};
  wr_dynamic_mesh_add_vertex(mMesh, pt);
  wr_dynamic_mesh_add_index(mMesh, index++);
  wr_dynamic_mesh_add_vertex(mMesh, ph1);
  wr_dynamic_mesh_add_index(mMesh, index++);
  wr_dynamic_mesh_add_vertex(mMesh, pt);
  wr_dynamic_mesh_add_index(mMesh, index++);
  wr_dynamic_mesh_add_vertex(mMesh, ph2);
  wr_dynamic_mesh_add_index(mMesh, index++);
}

void WbJointManipulator::refresh() {
  if (mActive && mJoint)
    rebuildMesh();
}

WbJointManipulator::Handle WbJointManipulator::pickHandle(const QPoint &pos) const {
  if (!mJoint || !mViewpoint)
    return NO_HANDLE;
  WbRay ray;
  mViewpoint->viewpointRay(pos.x(), pos.y(), ray);
  const WbVector3 anchor = anchorWorld();
  const double scale = mViewpoint->viewDistanceUnscaling(anchor);
  const double radius = 2.5 * HANDLE_RADIUS * scale;

  // distance from the mouse ray to a point
  if (distanceToRay(ray, anchor) < radius)
    return ANCHOR_HANDLE;
  const WbVector3 tip = anchor + AXIS_LENGTH * scale * axisWorld();
  if (distanceToRay(ray, tip) < radius)
    return AXIS_HANDLE;
  return NO_HANDLE;
}

bool WbJointManipulator::mousePressEvent(QMouseEvent *event) {
  if (!mActive || !mJoint || event->button() != Qt::LeftButton)
    return false;
  const Handle handle = pickHandle(event->pos());
  if (handle == NO_HANDLE)
    return false;
  mDragHandle = handle;
  mDragging = true;
  mViewpoint->viewpointRay(event->pos().x(), event->pos().y(), mDragRay);
  // drag on the camera-facing plane through the grabbed point
  mDragPlaneNormal = mDragRay.direction();
  mDragPlanePoint = (handle == ANCHOR_HANDLE) ? anchorWorld() : anchorWorld() + AXIS_LENGTH * axisWorld();
  mInitialAnchor = toParentFrame(anchorWorld());
  WbVector3 localAnchor, localAxis;
  mJoint->getAnchorAndAxis(localAnchor, localAxis);
  mInitialAxis = localAxis;
  mViewpoint->lock();
  return true;
}

bool WbJointManipulator::mouseMoveEvent(QMouseEvent *event) {
  if (!mDragging)
    return false;
  WbRay ray;
  mViewpoint->viewpointRay(event->pos().x(), event->pos().y(), ray);
  const WbAffinePlane plane(mDragPlaneNormal, mDragPlanePoint);
  const std::pair<bool, double> hit = ray.intersects(plane);
  if (!hit.first)
    return true;
  const WbVector3 world = ray.point(hit.second);
  if (mDragHandle == ANCHOR_HANDLE) {
    WbField *field = mJoint->anchorField();
    WbSFVector3 *value = field ? dynamic_cast<WbSFVector3 *>(field->value()) : NULL;
    if (value)
      value->setValueByUser(toParentFrame(world), false);
  } else if (mDragHandle == AXIS_HANDLE) {
    const WbVector3 axisWorldDir = (world - anchorWorld()).normalized();
    WbField *field = mJoint->axisField();
    WbSFVector3 *value = field ? dynamic_cast<WbSFVector3 *>(field->value()) : NULL;
    if (value)
      value->setValueByUser(directionToParentFrame(axisWorldDir), false);
  }
  rebuildMesh();
  return true;
}

bool WbJointManipulator::mouseReleaseEvent(QMouseEvent *event) {
  Q_UNUSED(event);
  if (!mDragging)
    return false;
  mDragging = false;
  mViewpoint->unlock();
  if (mDragHandle == ANCHOR_HANDLE || mDragHandle == AXIS_HANDLE)
    pushUndo();
  mDragHandle = NO_HANDLE;
  return true;
}

void WbJointManipulator::pushUndo() {
  if (!mJoint)
    return;
  WbVector3 localAnchor, localAxis;
  mJoint->getAnchorAndAxis(localAnchor, localAxis);
  if (mDragHandle == ANCHOR_HANDLE) {
    WbField *field = mJoint->anchorField();
    if (field) {
      WbSingleValue *value = dynamic_cast<WbSingleValue *>(field->value());
      if (value)
        WbUndoStack::instance()->push(new WbEditCommand(value, WbVariant(mInitialAnchor), WbVariant(localAnchor)));
    }
  } else if (mDragHandle == AXIS_HANDLE) {
    WbField *field = mJoint->axisField();
    if (field) {
      WbSingleValue *value = dynamic_cast<WbSingleValue *>(field->value());
      if (value)
        WbUndoStack::instance()->push(new WbEditCommand(value, WbVariant(mInitialAxis), WbVariant(localAxis)));
    }
  }
}
