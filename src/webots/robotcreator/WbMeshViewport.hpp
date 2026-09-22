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

// Description: 3D mesh viewport of the Robot Creator window: orbit camera,
//              mesh rendering, CAD snap markers (endpoint, midpoint, circle
//              center, centroid, grid) and joint anchor/axis preview.

#ifndef WB_MESH_VIEWPORT_HPP
#define WB_MESH_VIEWPORT_HPP

#include "WbMeshCore.hpp"
#include "WbRobotProtoCore.hpp"
#include "WbSnapCore.hpp"

#include <QtGui/QOpenGLBuffer>
#include <QtGui/QOpenGLFunctions>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QtGui/QMatrix4x4>

class QMouseEvent;
class QOpenGLShaderProgram;
class QOpenGLVertexArrayObject;
class QKeyEvent;
class QWheelEvent;

class WbMeshViewport : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  explicit WbMeshViewport(QWidget *parent = NULL);
  virtual ~WbMeshViewport();

  void setAssembly(wbrobotproto::Assembly *assembly);
  void rebuild();  // the assembly changed: recompute features and buffers

  // CAD snap modes (wbsnap::SnapMode flags)
  void setSnapModes(int modes) { mSnapModes = modes; }
  int snapModes() const { return mSnapModes; }
  void setGridSize(double size) { mGridSize = size; }
  void setShowGrid(bool show) { mShowGrid = show; update(); }
  void setShowFeatures(bool show) { mShowFeatures = show; update(); }
  void setSelectedPart(int index);

  // "insert joint" mode: clicks report a snap pick instead of doing nothing
  void setPlacementActive(bool active);
  bool placementActive() const { return mPlacementActive; }

public slots:
  void resetCamera();

signals:
  void statusMessage(const QString &message);
  // a snap pick: candidate.mode == SNAP_NONE is a plain surface pick
  void snapPicked(const wbsnap::SnapCandidate &candidate, int partIndex);
  void placementCancelled();

protected:
  void initializeGL() override;
  void resizeGL(int w, int h) override;
  void paintGL() override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;
  void keyPressEvent(QKeyEvent *event) override;

private:
  struct PartBuffers {
    QOpenGLVertexArrayObject *vao;
    QOpenGLBuffer *vertexBuffer;
    QOpenGLBuffer *indexBuffer;
    int indexCount;
    PartBuffers() : vao(NULL), vertexBuffer(NULL), indexBuffer(NULL), indexCount(0) {}
  };

  wbmesh::Vec3 cameraPosition() const;
  QMatrix4x4 viewMatrix() const;
  QMatrix4x4 projectionMatrix() const;
  // ray through a pixel (origin = camera, direction normalized)
  void rayThroughPixel(const QPoint &pixel, wbmesh::Vec3 &origin, wbmesh::Vec3 &direction) const;
  QPoint project(const wbmesh::Vec3 &point, const QMatrix4x4 &mvp, bool *inFront = NULL) const;
  void updateHover(const QPoint &pixel);
  void uploadGeometry();
  void clearBuffers();

  wbrobotproto::Assembly *mAssembly;
  std::vector<wbsnap::FeatureSet> mFeatures;

  // camera (Webots style: z up)
  double mYaw, mPitch, mDistance;
  wbmesh::Vec3 mTarget;
  QPoint mLastMouse;
  int mOrbitButton;
  bool mPanning;

  // snapping
  int mSnapModes;
  double mGridSize;
  bool mShowGrid;
  bool mShowFeatures;
  bool mPlacementActive;
  double mSnapRadiusPx;
  std::vector<wbsnap::SnapCandidate> mRanked;
  std::vector<int> mRankedParts;
  size_t mHoverIndex;
  int mSelectedPart;

  // GL
  QOpenGLShaderProgram *mProgram;
  std::vector<PartBuffers *> mPartBuffers;
  QOpenGLBuffer *mScratchVertices;
  QOpenGLBuffer *mScratchIndices;
  QOpenGLVertexArrayObject *mScratchVao;
  bool mGlReady;
  bool mDirty;
};

#endif
