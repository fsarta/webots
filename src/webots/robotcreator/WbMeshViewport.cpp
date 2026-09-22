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

#include "WbMeshViewport.hpp"

#include <QtGui/QFont>
#include <QtGui/QKeyEvent>
#include <QtGui/QMouseEvent>
#include <QtGui/QOpenGLShaderProgram>
#include <QtGui/QOpenGLVertexArrayObject>
#include <QtGui/QPainter>
#include <QtGui/QSurfaceFormat>
#include <QtGui/QVector3D>
#include <QtGui/QVector4D>
#include <QtGui/QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdio>

static const int VERTEX_ATTRIBUTE = 0;
static const int NORMAL_ATTRIBUTE = 1;

static const char *VERTEX_SHADER =
  "#version 330 core\n"
  "layout(location = 0) in vec3 pos;\n"
  "layout(location = 1) in vec3 normal;\n"
  "uniform mat4 mvp;\n"
  "out vec3 vNormal;\n"
  "void main() {\n"
  "  vNormal = normal;\n"
  "  gl_Position = mvp * vec4(pos, 1.0);\n"
  "}\n";

static const char *FRAGMENT_SHADER =
  "#version 330 core\n"
  "in vec3 vNormal;\n"
  "uniform vec3 color;\n"
  "uniform int lit;\n"
  "out vec4 fragColor;\n"
  "void main() {\n"
  "  float k = 1.0;\n"
  "  if (lit != 0) {\n"
  "    vec3 n = normalize(vNormal);\n"
  "    vec3 l = normalize(vec3(0.4, -0.5, 0.8));\n"
  "    k = 0.35 + 0.65 * max(dot(n, l), 0.0);\n"
  "  }\n"
  "  fragColor = vec4(color * k, 1.0);\n"
  "}\n";

static QColor snapColor(wbsnap::SnapMode mode) {
  switch (mode) {
    case wbsnap::SNAP_ENDPOINT:
      return QColor(80, 220, 80);  // green
    case wbsnap::SNAP_MIDPOINT:
      return QColor(80, 160, 255);  // blue
    case wbsnap::SNAP_CIRCLE_CENTER:
      return QColor(255, 80, 80);  // red
    case wbsnap::SNAP_CENTROID:
      return QColor(240, 210, 60);  // yellow
    case wbsnap::SNAP_GRID:
      return QColor(200, 200, 200);  // gray
    default:
      return QColor(255, 160, 60);  // surface pick: orange
  }
}

WbMeshViewport::WbMeshViewport(QWidget *parent) :
  QOpenGLWidget(parent),
  mAssembly(NULL),
  mYaw(0.6),
  mPitch(0.5),
  mDistance(4.0),
  mOrbitButton(Qt::NoButton),
  mPanning(false),
  mSnapModes(wbsnap::SNAP_ENDPOINT | wbsnap::SNAP_MIDPOINT | wbsnap::SNAP_CIRCLE_CENTER | wbsnap::SNAP_CENTROID |
             wbsnap::SNAP_GRID),
  mGridSize(0.1),
  mShowGrid(true),
  mShowFeatures(true),
  mPlacementActive(false),
  mSnapRadiusPx(14.0),
  mHoverIndex(0),
  mSelectedPart(-1),
  mProgram(NULL),
  mScratchVertices(NULL),
  mScratchIndices(NULL),
  mScratchVao(NULL),
  mGlReady(false),
  mDirty(false) {
  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setDepthBufferSize(24);
  setFormat(format);
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
}

WbMeshViewport::~WbMeshViewport() {
  clearBuffers();
  delete mProgram;
}

void WbMeshViewport::setAssembly(wbrobotproto::Assembly *assembly) {
  mAssembly = assembly;
  rebuild();
}

void WbMeshViewport::setSelectedPart(int index) {
  mSelectedPart = index;
  update();
}

void WbMeshViewport::setPlacementActive(bool active) {
  mPlacementActive = active;
  setFocus();
  update();
}

void WbMeshViewport::resetCamera() {
  mTarget = wbmesh::Vec3();
  if (mAssembly) {
    // center on the assembly bounds
    wbmesh::Vec3 min(1e30, 1e30, 1e30), max(-1e30, -1e30, -1e30);
    for (size_t i = 0; i < mAssembly->parts.size(); ++i) {
      const wbrobotproto::Part &part = mAssembly->parts[i];
      wbmesh::Vec3 mn = part.mesh.isEmpty() ? part.translation : part.mesh.boundsMin + part.translation;
      wbmesh::Vec3 mx = part.mesh.isEmpty() ? part.translation : part.mesh.boundsMax + part.translation;
      min.x = std::min(min.x, mn.x);
      min.y = std::min(min.y, mn.y);
      min.z = std::min(min.z, mn.z);
      max.x = std::max(max.x, mx.x);
      max.y = std::max(max.y, mx.y);
      max.z = std::max(max.z, mx.z);
    }
    if (max.x > min.x)
      mTarget = (min + max) * 0.5;
    mDistance = std::max(1.0, max.distance(min) * 1.2);
  }
  mYaw = 0.6;
  mPitch = 0.5;
  update();
}

void WbMeshViewport::rebuild() {
  mFeatures.clear();
  if (mAssembly) {
    for (size_t i = 0; i < mAssembly->parts.size(); ++i)
      mFeatures.push_back(wbsnap::buildFeatures(mAssembly->parts[i].mesh));
  }
  mRanked.clear();
  mRankedParts.clear();
  mDirty = true;
  if (mGlReady)
    uploadGeometry();
  update();
}

void WbMeshViewport::initializeGL() {
  initializeOpenGLFunctions();
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.13f, 0.14f, 0.17f, 1.0f);
  mProgram = new QOpenGLShaderProgram(this);
  mProgram->addShaderFromSourceCode(QOpenGLShader::Vertex, VERTEX_SHADER);
  mProgram->addShaderFromSourceCode(QOpenGLShader::Fragment, FRAGMENT_SHADER);
  mProgram->link();
  mScratchVao = new QOpenGLVertexArrayObject();
  mScratchVao->create();
  mScratchVertices = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
  mScratchVertices->create();
  mScratchIndices = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
  mScratchIndices->create();
  mGlReady = true;
  if (mDirty || !mPartBuffers.empty())
    uploadGeometry();
}

void WbMeshViewport::clearBuffers() {
  for (size_t i = 0; i < mPartBuffers.size(); ++i) {
    delete mPartBuffers[i]->vao;
    delete mPartBuffers[i]->vertexBuffer;
    delete mPartBuffers[i]->indexBuffer;
    delete mPartBuffers[i];
  }
  mPartBuffers.clear();
}

void WbMeshViewport::uploadGeometry() {
  mDirty = false;
  if (!mGlReady || !mAssembly)
    return;
  clearBuffers();
  for (size_t i = 0; i < mAssembly->parts.size(); ++i) {
    const wbmesh::Mesh &mesh = mAssembly->parts[i].mesh;
    PartBuffers *buffers = new PartBuffers();
    buffers->vao = new QOpenGLVertexArrayObject();
    buffers->vao->create();
    buffers->vertexBuffer = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    buffers->vertexBuffer->create();
    buffers->indexBuffer = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    buffers->indexBuffer->create();
    buffers->indexCount = 0;
    if (!mesh.isEmpty()) {
      std::vector<float> vertices;
      vertices.reserve(mesh.positions.size() * 6);
      std::vector<float> normals(mesh.positions.size() * 3, 0.0f);
      for (int t = 0; t < mesh.triangleCount(); ++t) {
        const wbmesh::Vec3 n = wbmesh::faceNormal(mesh, t);
        for (int k = 0; k < 3; ++k) {
          const int v = mesh.triangles[t * 3 + k];
          normals[v * 3] += (float)n.x;
          normals[v * 3 + 1] += (float)n.y;
          normals[v * 3 + 2] += (float)n.z;
        }
      }
      for (size_t v = 0; v < mesh.positions.size(); ++v) {
        wbmesh::Vec3 n(normals[v * 3], normals[v * 3 + 1], normals[v * 3 + 2]);
        if (n.isZero())
          n = wbmesh::Vec3(0, 0, 1);
        n = n.normalized();
        vertices.push_back((float)mesh.positions[v].x);
        vertices.push_back((float)mesh.positions[v].y);
        vertices.push_back((float)mesh.positions[v].z);
        vertices.push_back((float)n.x);
        vertices.push_back((float)n.y);
        vertices.push_back((float)n.z);
      }
      std::vector<unsigned int> indices;
      indices.reserve(mesh.triangles.size());
      for (size_t t = 0; t < mesh.triangles.size(); ++t)
        indices.push_back((unsigned int)mesh.triangles[t]);

      QOpenGLVertexArrayObject::Binder binder(buffers->vao);
      buffers->vertexBuffer->bind();
      buffers->vertexBuffer->allocate(vertices.data(), (int)(vertices.size() * sizeof(float)));
      buffers->indexBuffer->bind();
      buffers->indexBuffer->allocate(indices.data(), (int)(indices.size() * sizeof(unsigned int)));
      mProgram->bind();
      mProgram->enableAttributeArray(VERTEX_ATTRIBUTE);
      mProgram->setAttributeBuffer(VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(float));
      mProgram->enableAttributeArray(NORMAL_ATTRIBUTE);
      mProgram->setAttributeBuffer(NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
      mProgram->release();
      buffers->indexCount = (int)indices.size();
    }
    mPartBuffers.push_back(buffers);
  }
}

void WbMeshViewport::resizeGL(int, int) {
}

wbmesh::Vec3 WbMeshViewport::cameraPosition() const {
  const double cp = std::cos(mPitch), sp = std::sin(mPitch);
  const double cy = std::cos(mYaw), sy = std::sin(mYaw);
  return mTarget + wbmesh::Vec3(mDistance * cp * cy, mDistance * cp * sy, mDistance * sp);
}

QMatrix4x4 WbMeshViewport::viewMatrix() const {
  const wbmesh::Vec3 eye = cameraPosition();
  QMatrix4x4 view;
  view.lookAt(QVector3D((float)eye.x, (float)eye.y, (float)eye.z), QVector3D((float)mTarget.x, (float)mTarget.y, (float)mTarget.z),
              QVector3D(0, 0, 1));
  return view;
}

QMatrix4x4 WbMeshViewport::projectionMatrix() const {
  QMatrix4x4 projection;
  projection.perspective(45.0f, (float)width() / std::max(1, height()), 0.01f, 1000.0f);
  return projection;
}

void WbMeshViewport::rayThroughPixel(const QPoint &pixel, wbmesh::Vec3 &origin, wbmesh::Vec3 &direction) const {
  const float x = 2.0f * pixel.x() / std::max(1, width()) - 1.0f;
  const float y = 1.0f - 2.0f * pixel.y() / std::max(1, height());
  const QMatrix4x4 inverse = (projectionMatrix() * viewMatrix()).inverted();
  QVector4D nearPoint = inverse * QVector4D(x, y, -1.0f, 1.0f);
  QVector4D farPoint = inverse * QVector4D(x, y, 1.0f, 1.0f);
  nearPoint /= nearPoint.w();
  farPoint /= farPoint.w();
  origin = wbmesh::Vec3(nearPoint.x(), nearPoint.y(), nearPoint.z());
  direction = wbmesh::Vec3(farPoint.x() - nearPoint.x(), farPoint.y() - nearPoint.y(), farPoint.z() - nearPoint.z())
                .normalized();
}

QPoint WbMeshViewport::project(const wbmesh::Vec3 &point, const QMatrix4x4 &mvp, bool *inFront) const {
  const QVector4D clip = mvp * QVector4D((float)point.x, (float)point.y, (float)point.z, 1.0f);
  if (inFront)
    *inFront = clip.w() > 0.0f;
  if (std::fabs(clip.w()) < 1e-9)
    return QPoint(-10000, -10000);
  const float nx = clip.x() / clip.w();
  const float ny = clip.y() / clip.w();
  return QPoint((int)((nx * 0.5f + 0.5f) * width()), (int)((0.5f - ny * 0.5f) * height()));
}

void WbMeshViewport::updateHover(const QPoint &pixel) {
  mRanked.clear();
  mRankedParts.clear();
  mHoverIndex = 0;
  if (!mAssembly)
    return;
  const QMatrix4x4 mvp = projectionMatrix() * viewMatrix();
  wbmesh::Vec3 origin, direction;
  rayThroughPixel(pixel, origin, direction);

  std::vector<wbsnap::SnapCandidate> candidates;
  std::vector<int> partIndices;
  for (size_t i = 0; i < mFeatures.size(); ++i) {
    std::vector<wbsnap::SnapCandidate> partCandidates = wbsnap::collectCandidates(mFeatures[i], mSnapModes);
    for (size_t c = 0; c < partCandidates.size(); ++c) {
      partCandidates[c].position = wbmesh::Vec3(
        partCandidates[c].position.x + mAssembly->parts[i].translation.x,
        partCandidates[c].position.y + mAssembly->parts[i].translation.y,
        partCandidates[c].position.z + mAssembly->parts[i].translation.z);
      candidates.push_back(partCandidates[c]);
      partIndices.push_back((int)i);
    }
  }
  // grid snap on the z = 0 work plane
  if (mSnapModes & wbsnap::SNAP_GRID) {
    wbmesh::Vec3 hit;
    if (wbsnap::gridRayHit(origin, direction, mGridSize, hit)) {
      wbsnap::SnapCandidate candidate;
      candidate.mode = wbsnap::SNAP_GRID;
      candidate.label = wbsnap::snapLabel(candidate.mode);
      candidate.position = hit;
      candidate.axis = wbmesh::Vec3(0, 0, 1);
      candidate.hasAxis = true;
      candidates.push_back(candidate);
      partIndices.push_back(-1);
    }
  }
  // screen-space proximity
  for (size_t c = 0; c < candidates.size(); ++c) {
    bool inFront = false;
    const QPoint projected = project(candidates[c].position, mvp, &inFront);
    const double dx = projected.x() - pixel.x();
    const double dy = projected.y() - pixel.y();
    candidates[c].screenDistance = inFront ? std::sqrt(dx * dx + dy * dy) : 1e9;
    candidates[c].depth = candidates[c].position.distance(origin);
  }
  const std::vector<wbsnap::SnapCandidate> ranked = wbsnap::rankCandidates(candidates, mSnapRadiusPx, 8.0);
  // keep the part association through the ranking (match by position)
  for (size_t r = 0; r < ranked.size(); ++r) {
    for (size_t c = 0; c < candidates.size(); ++c) {
      if (candidates[c].position.distance(ranked[r].position) < 1e-12 &&
          candidates[c].screenDistance == ranked[r].screenDistance) {
        mRankedParts.push_back(partIndices[c]);
        break;
      }
    }
    if (mRankedParts.size() <= r)
      mRankedParts.push_back(-1);
    mRanked.push_back(ranked[r]);
  }

  if (!mRanked.empty()) {
    const wbsnap::SnapCandidate &best = mRanked[mHoverIndex];
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "%s @ (%.4g, %.4g, %.4g)", best.label.c_str(), best.position.x,
                  best.position.y, best.position.z);
    QString message = buffer;
    if (mRanked.size() > 1)
      message += tr(" — Tab cycles through %1 candidates").arg(mRanked.size());
    if (best.hasAxis)
      message += tr(" — axis (%1, %2, %3)").arg(best.axis.x, 0, 'g', 3).arg(best.axis.y, 0, 'g', 3).arg(best.axis.z, 0, 'g', 3);
    emit statusMessage(message);
  } else
    emit statusMessage(tr("No snap in range (F1-F5 toggle snap modes)."));
  update();
}

void WbMeshViewport::mousePressEvent(QMouseEvent *event) {
  setFocus();
  mLastMouse = event->position().toPoint();
  if (event->button() == Qt::RightButton || event->button() == Qt::MiddleButton) {
    mOrbitButton = event->button();
    mPanning = event->button() == Qt::MiddleButton && (event->modifiers() & Qt::ShiftModifier);
  }
  event->accept();
}

void WbMeshViewport::mouseMoveEvent(QMouseEvent *event) {
  const QPoint position = event->position().toPoint();
  const QPoint delta = position - mLastMouse;
  mLastMouse = position;
  if (mOrbitButton == Qt::RightButton) {
    mYaw -= delta.x() * 0.01;
    mPitch += delta.y() * 0.01;
    mPitch = std::max(-1.55, std::min(1.55, mPitch));
    update();
  } else if (mOrbitButton == Qt::MiddleButton) {
    if (mPanning) {
      // pan in the camera plane
      const QMatrix4x4 view = viewMatrix();
      const wbmesh::Vec3 right(view(0, 0), view(0, 1), view(0, 2));
      const wbmesh::Vec3 up(view(1, 0), view(1, 1), view(1, 2));
      const double scale = mDistance * 0.0015;
      mTarget = mTarget - right * (delta.x() * scale) + up * (delta.y() * scale);
    } else {
      mDistance *= 1.0 + (delta.y() - delta.x()) * 0.005;
      mDistance = std::max(0.05, mDistance);
    }
    update();
  } else
    updateHover(position);
  event->accept();
}

void WbMeshViewport::mouseReleaseEvent(QMouseEvent *event) {
  if (mOrbitButton == Qt::NoButton && event->button() == Qt::LeftButton && mPlacementActive) {
    wbsnap::SnapCandidate candidate;
    int partIndex = -1;
    if (!mRanked.empty()) {
      candidate = mRanked[mHoverIndex];
      partIndex = mRankedParts[mHoverIndex];
    } else {
      // plain surface pick with the inferred face normal as joint axis
      wbmesh::Vec3 origin, direction;
      rayThroughPixel(event->position().toPoint(), origin, direction);
      for (size_t i = 0; i < mAssembly->parts.size(); ++i) {
        const wbrobotproto::Part &part = mAssembly->parts[i];
        // test the ray in the part's local frame
        const wbmesh::Mat3 rotation = wbmesh::Mat3::fromAxisAngle(part.rotationAxis, part.rotationAngle);
        const wbmesh::Vec3 localOrigin = rotation.transposed().mul(origin - part.translation);
        const wbmesh::Vec3 localDirection = rotation.transposed().mul(direction);
        int triangle = -1;
        wbmesh::Vec3 hit;
        if (wbmesh::rayMesh(localOrigin, localDirection, part.mesh, triangle, hit)) {
          candidate.mode = wbsnap::SNAP_NONE;
          candidate.label = "surface";
          candidate.position = part.translation + rotation.mul(hit);
          candidate.axis = rotation.mul(wbmesh::faceNormal(part.mesh, triangle));
          candidate.hasAxis = !candidate.axis.isZero();
          partIndex = (int)i;
          break;
        }
      }
    }
    if (partIndex >= 0)
      emit snapPicked(candidate, partIndex);
    else
      emit statusMessage(tr("Click on a mesh (or enable the grid snap) to place the joint anchor."));
  }
  mOrbitButton = Qt::NoButton;
  mPanning = false;
  event->accept();
}

void WbMeshViewport::wheelEvent(QWheelEvent *event) {
  mDistance *= 1.0 + event->angleDelta().y() * -0.001;
  mDistance = std::max(0.05, mDistance);
  update();
  event->accept();
}

void WbMeshViewport::keyPressEvent(QKeyEvent *event) {
  switch (event->key()) {
    case Qt::Key_Escape:
      if (mPlacementActive) {
        emit placementCancelled();
        break;
      }
      break;
    case Qt::Key_Tab:
      if (!mRanked.empty()) {
        mHoverIndex = (mHoverIndex + 1) % mRanked.size();
        const wbsnap::SnapCandidate &best = mRanked[mHoverIndex];
        emit statusMessage(tr("%1 @ (%2, %3, %4) — candidate %5 of %6")
                             .arg(best.label.c_str())
                             .arg(best.position.x, 0, 'g', 5)
                             .arg(best.position.y, 0, 'g', 5)
                             .arg(best.position.z, 0, 'g', 5)
                             .arg(mHoverIndex + 1)
                             .arg(mRanked.size()));
        update();
      }
      break;
    case Qt::Key_Home:
      resetCamera();
      break;
    default:
      QOpenGLWidget::keyPressEvent(event);
      return;
  }
  event->accept();
}

// draw small world-space crosses / lines through the scratch buffer
static void appendCross(std::vector<float> &vertices, std::vector<unsigned int> &indices, const wbmesh::Vec3 &center,
                        double size) {
  const double offsets[3][6] = {{-1, 0, 0, 1, 0, 0}, {0, -1, 0, 0, 1, 0}, {0, 0, -1, 0, 0, 1}};
  for (int axis = 0; axis < 3; ++axis) {
    const unsigned int base = (unsigned int)(vertices.size() / 6);
    for (int end = 0; end < 2; ++end) {
      const double s = end == 0 ? -size : size;
      vertices.push_back((float)(center.x + offsets[axis][end * 3] * s));
      vertices.push_back((float)(center.y + offsets[axis][end * 3 + 1] * s));
      vertices.push_back((float)(center.z + offsets[axis][end * 3 + 2] * s));
      vertices.push_back(0.0f);
      vertices.push_back(0.0f);
      vertices.push_back(1.0f);
    }
    indices.push_back(base);
    indices.push_back(base + 1);
  }
}

static void appendLine(std::vector<float> &vertices, std::vector<unsigned int> &indices, const wbmesh::Vec3 &a,
                       const wbmesh::Vec3 &b) {
  const unsigned int base = (unsigned int)(vertices.size() / 6);
  const wbmesh::Vec3 points[2] = {a, b};
  for (int i = 0; i < 2; ++i) {
    vertices.push_back((float)points[i].x);
    vertices.push_back((float)points[i].y);
    vertices.push_back((float)points[i].z);
    vertices.push_back(0.0f);
    vertices.push_back(0.0f);
    vertices.push_back(1.0f);
  }
  indices.push_back(base);
  indices.push_back(base + 1);
}

void WbMeshViewport::paintGL() {
  if (mDirty)
    uploadGeometry();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (!mProgram || !mProgram->isLinked())
    return;
  const QMatrix4x4 mvp = projectionMatrix() * viewMatrix();
  const wbmesh::Vec3 eye = cameraPosition();
  mProgram->bind();
  mProgram->setUniformValue("mvp", mvp);

  // parts
  for (size_t i = 0; i < mPartBuffers.size(); ++i) {
    if (mPartBuffers[i]->indexCount == 0)
      continue;
    const QColor color = (int)i == mSelectedPart ? QColor(235, 180, 90) : QColor(150, 160, 185);
    mProgram->setUniformValue("color", QVector3D(color.redF(), color.greenF(), color.blueF()));
    mProgram->setUniformValue("lit", 1);
    QOpenGLVertexArrayObject::Binder binder(mPartBuffers[i]->vao);
    glDrawElements(GL_TRIANGLES, mPartBuffers[i]->indexCount, GL_UNSIGNED_INT, NULL);
  }

  // grid + feature markers + joint glyphs in the scratch buffer
  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  if (mShowGrid) {
    const double extent = 2.0;
    for (int k = -20; k <= 20; ++k) {
      appendLine(vertices, indices, wbmesh::Vec3(k * mGridSize, -extent, 0), wbmesh::Vec3(k * mGridSize, extent, 0));
      appendLine(vertices, indices, wbmesh::Vec3(-extent, k * mGridSize, 0), wbmesh::Vec3(extent, k * mGridSize, 0));
    }
  }
  const double markerScale = std::max(0.01, mDistance * 0.004);
  if (mShowFeatures && mAssembly) {
    std::vector<std::pair<wbsnap::SnapMode, wbmesh::Vec3> > markers;
    for (size_t i = 0; i < mFeatures.size(); ++i) {
      const wbmesh::Vec3 offset = mAssembly->parts[i].translation;
      for (size_t k = 0; k < mFeatures[i].endpoints.size(); ++k)
        markers.push_back(std::make_pair(wbsnap::SNAP_ENDPOINT, mFeatures[i].endpoints[k] + offset));
      for (size_t k = 0; k < mFeatures[i].midpoints.size(); ++k)
        markers.push_back(std::make_pair(wbsnap::SNAP_MIDPOINT, mFeatures[i].midpoints[k] + offset));
      for (size_t k = 0; k < mFeatures[i].centroids.size(); ++k)
        markers.push_back(std::make_pair(wbsnap::SNAP_CENTROID, mFeatures[i].centroids[k] + offset));
      for (size_t k = 0; k < mFeatures[i].circles.size(); ++k)
        markers.push_back(std::make_pair(wbsnap::SNAP_CIRCLE_CENTER, mFeatures[i].circles[k].center + offset));
    }
    // one draw call per snap kind keeps the colors CAD-like
    for (int kind = 0; kind < 5; ++kind) {
      vertices.clear();
      indices.clear();
      for (size_t k = 0; k < markers.size(); ++k) {
        if (snapPriority(markers[k].first) != kind)
          continue;
        appendCross(vertices, indices, markers[k].second, markerScale * (kind == 1 ? 2.0 : 1.0));
      }
      if (indices.empty())
        continue;
      // map priority order back to the snap color
      static const wbsnap::SnapMode order[5] = {wbsnap::SNAP_ENDPOINT, wbsnap::SNAP_CIRCLE_CENTER, wbsnap::SNAP_MIDPOINT,
                                                wbsnap::SNAP_CENTROID, wbsnap::SNAP_GRID};
      const QColor c = snapColor(order[kind]);
      mProgram->setUniformValue("color", QVector3D(c.redF(), c.greenF(), c.blueF()));
      mProgram->setUniformValue("lit", 0);
      QOpenGLVertexArrayObject::Binder binder(mScratchVao);
      mScratchVertices->bind();
      mScratchVertices->allocate(vertices.data(), (int)(vertices.size() * sizeof(float)));
      mScratchIndices->bind();
      mScratchIndices->allocate(indices.data(), (int)(indices.size() * sizeof(unsigned int)));
      mProgram->enableAttributeArray(VERTEX_ATTRIBUTE);
      mProgram->setAttributeBuffer(VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(float));
      mProgram->enableAttributeArray(NORMAL_ATTRIBUTE);
      mProgram->setAttributeBuffer(NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
      glDrawElements(GL_LINES, (int)indices.size(), GL_UNSIGNED_INT, NULL);
    }
  }
  if (mAssembly) {
    vertices.clear();
    indices.clear();
    for (size_t j = 0; j < mAssembly->joints.size(); ++j) {
      const wbrobotproto::JointDef &def = mAssembly->joints[j];
      appendCross(vertices, indices, def.anchor, markerScale * 2.0);
      appendLine(vertices, indices, def.anchor - def.axis * (markerScale * 6.0),
                 def.anchor + def.axis * (markerScale * 6.0));
    }
    if (!indices.empty()) {
      mProgram->setUniformValue("color", QVector3D(1.0f, 0.5f, 0.2f));
      mProgram->setUniformValue("lit", 0);
      QOpenGLVertexArrayObject::Binder binder(mScratchVao);
      mScratchVertices->bind();
      mScratchVertices->allocate(vertices.data(), (int)(vertices.size() * sizeof(float)));
      mScratchIndices->bind();
      mScratchIndices->allocate(indices.data(), (int)(indices.size() * sizeof(unsigned int)));
      mProgram->enableAttributeArray(VERTEX_ATTRIBUTE);
      mProgram->setAttributeBuffer(VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(float));
      mProgram->enableAttributeArray(NORMAL_ATTRIBUTE);
      mProgram->setAttributeBuffer(NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
      glDrawElements(GL_LINES, (int)indices.size(), GL_UNSIGNED_INT, NULL);
    }
  }
  // hover highlight (bigger cross in the snap color)
  if (!mRanked.empty()) {
    const wbsnap::SnapCandidate &best = mRanked[mHoverIndex];
    vertices.clear();
    indices.clear();
    appendCross(vertices, indices, best.position, markerScale * 3.5);
    const QColor c = snapColor(best.mode);
    mProgram->setUniformValue("color", QVector3D(c.redF(), c.greenF(), c.blueF()));
    mProgram->setUniformValue("lit", 0);
    QOpenGLVertexArrayObject::Binder binder(mScratchVao);
    mScratchVertices->bind();
    mScratchVertices->allocate(vertices.data(), (int)(vertices.size() * sizeof(float)));
    mScratchIndices->bind();
    mScratchIndices->allocate(indices.data(), (int)(indices.size() * sizeof(unsigned int)));
    mProgram->enableAttributeArray(VERTEX_ATTRIBUTE);
    mProgram->setAttributeBuffer(VERTEX_ATTRIBUTE, GL_FLOAT, 0, 3, 6 * sizeof(float));
    mProgram->enableAttributeArray(NORMAL_ATTRIBUTE);
    mProgram->setAttributeBuffer(NORMAL_ATTRIBUTE, GL_FLOAT, 3 * sizeof(float), 3, 6 * sizeof(float));
    glDrawElements(GL_LINES, (int)indices.size(), GL_UNSIGNED_INT, NULL);
  }
  mProgram->release();

  // 2D overlay: candidate label next to the cursor
  if (!mRanked.empty()) {
    const wbsnap::SnapCandidate &best = mRanked[mHoverIndex];
    const QPoint p = project(best.position, mvp);
    QPainter painter(this);
    painter.setPen(snapColor(best.mode));
    painter.setFont(QFont("monospace", 9));
    painter.drawText(p + QPoint(12, -8), QString::fromStdString(best.label));
    painter.end();
  }
  Q_UNUSED(eye);
}
