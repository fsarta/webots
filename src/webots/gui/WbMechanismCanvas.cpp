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

#include "WbMechanismCanvas.hpp"

#include <QtCore/QMap>
#include <QtCore/QtMath>
#include <QtGui/QMouseEvent>
#include <QtGui/QPainter>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QGraphicsPathItem>
#include <QtWidgets/QGraphicsScene>
#include <QtWidgets/QGraphicsSimpleTextItem>

#include "WbKinematicGraphCore.hpp"
#include "WbMechanismModel.hpp"

// ---------------------------------------------------------------- items

class WbMechanismLinkItem : public QGraphicsRectItem {
public:
  enum { Type = UserType + 1 };
  WbMechanismLinkItem(long linkId, bool isStatic, const QString &name) :
    QGraphicsRectItem(NULL), mLinkId(linkId), mIsStatic(isStatic), mPending(false) {
    setRect(-70.0, -26.0, 140.0, 52.0);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setFlag(QGraphicsItem::ItemSendsGeometryChanges, true);
    setZValue(1.0);
    mNameItem = new QGraphicsSimpleTextItem(name, this);
    mNameItem->setBrush(QColor(30, 30, 30));
    updateStyle();
  }
  int type() const override { return Type; }
  long linkId() const { return mLinkId; }
  void setPending(bool pending) {
    mPending = pending;
    updateStyle();
  }
  void setSelectedMark(bool selected) {
    mSelected = selected;
    updateStyle();
  }
  void setName(const QString &name) { mNameItem->setText(name); }

private:
  void updateStyle() {
    QPen pen(mPending ? QColor(230, 120, 0) : (mIsStatic ? QColor(40, 90, 170) : QColor(70, 70, 70)));
    pen.setWidthF(mSelected || mPending ? 3.0 : 1.5);
    setPen(pen);
    setBrush(mIsStatic ? QColor(180, 210, 255) : (mSelected ? QColor(220, 235, 255) : QColor(245, 245, 245)));
    const QRectF r = rect();
    mNameItem->setPos(r.center().x() - mNameItem->boundingRect().width() / 2.0,
                      r.center().y() - mNameItem->boundingRect().height() / 2.0);
  }

  long mLinkId;
  bool mIsStatic;
  bool mPending = false;
  bool mSelected = false;
  QGraphicsSimpleTextItem *mNameItem;
};

class WbMechanismJointItem : public QGraphicsEllipseItem {
public:
  enum { Type = UserType + 2 };
  WbMechanismJointItem(long jointId, bool isLoopClosure, int jointDof) :
    QGraphicsEllipseItem(NULL), mJointId(jointId), mIsLoopClosure(isLoopClosure), mJointDof(jointDof) {
    setRect(-11.0, -11.0, 22.0, 22.0);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    setZValue(2.0);
    updateStyle();
  }
  int type() const override { return Type; }
  long jointId() const { return mJointId; }
  bool isLoopClosure() const { return mIsLoopClosure; }
  void setSelectedMark(bool selected) {
    mSelected = selected;
    updateStyle();
  }

protected:
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override {
    painter->setPen(pen());
    painter->setBrush(brush());
    if (mIsLoopClosure) {
      // diamond glyph for joints closing a kinematic loop
      QPolygonF diamond;
      diamond << QPointF(0.0, -13.0) << QPointF(13.0, 0.0) << QPointF(0.0, 13.0) << QPointF(-13.0, 0.0);
      painter->drawPolygon(diamond);
    } else {
      painter->drawEllipse(rect());
      if (mJointDof == 0) {
        // rigid: cross
        painter->drawLine(QPointF(-6.0, -6.0), QPointF(6.0, 6.0));
        painter->drawLine(QPointF(-6.0, 6.0), QPointF(6.0, -6.0));
      }
    }
  }

private:
  void updateStyle() {
    QColor color(200, 60, 60);  // revolute
    if (mJointDof == 0)
      color = QColor(90, 90, 90);
    else if (mJointDof == 2)
      color = QColor(60, 130, 200);
    else if (mJointDof == 3)
      color = QColor(150, 70, 190);
    if (mIsLoopClosure)
      color = QColor(20, 140, 80);
    QPen pen(mSelected ? QColor(230, 120, 0) : color);
    pen.setWidthF(mSelected ? 3.0 : 1.8);
    setPen(pen);
    setBrush(mSelected ? QColor(255, 235, 200) : QColor(255, 255, 255));
  }

  long mJointId;
  bool mIsLoopClosure;
  int mJointDof;
  bool mSelected = false;
};

class WbMechanismEdgeItem : public QGraphicsPathItem {
public:
  enum { Type = UserType + 3 };
  WbMechanismEdgeItem(bool isLoopClosure, const QLineF &line) : QGraphicsPathItem(NULL), mLoopFrom() {
    QPen pen(isLoopClosure ? QColor(20, 140, 80) : QColor(120, 120, 120));
    pen.setWidthF(isLoopClosure ? 2.0 : 1.6);
    if (isLoopClosure) {
      pen.setStyle(Qt::DashLine);
    }
    setPen(pen);
    setZValue(0.5);
    setLine(line);
  }
  int type() const override { return Type; }
  void setLine(const QLineF &line) {
    QPainterPath path;
    path.moveTo(line.p1());
    if (pen().style() == Qt::DashLine) {
      // loop closures are drawn as arcs bulging to the side
      const QPointF mid = 0.5 * (line.p1() + line.p2());
      QPointF normal(line.p2().y() - line.p1().y(), line.p1().x() - line.p2().x());
      const double len = qMax(1.0, qSqrt(normal.x() * normal.x() + normal.y() * normal.y()));
      normal /= len;
      path.quadTo(mid + normal * 40.0, line.p2());
    } else
      path.lineTo(line.p2());
    setPath(path);
  }

private:
  QPointF mLoopFrom;
};

// ---------------------------------------------------------------- canvas

WbMechanismCanvas::WbMechanismCanvas(QWidget *parent) :
  QGraphicsView(parent),
  mScene(new QGraphicsScene(this)),
  mTool(SELECT_TOOL),
  mSelectedLinkId(-1),
  mSelectedJointId(-1),
  mPendingParentLinkId(-1),
  mDraggingLink(false),
  mNeedsRebuild(true) {
  setScene(mScene);
  setRenderHint(QPainter::Antialiasing, true);
  setDragMode(QGraphicsView::NoDrag);
  setBackgroundBrush(QColor(250, 250, 252));
  setMinimumHeight(240);
  connect(WbMechanismModel::instance(), &WbMechanismModel::modelReset, this, &WbMechanismCanvas::rebuild,
          Qt::QueuedConnection);
  connect(WbMechanismModel::instance(), &WbMechanismModel::layoutChanged, this, &WbMechanismCanvas::applyLayout,
          Qt::QueuedConnection);
}

WbMechanismCanvas::~WbMechanismCanvas() {
}

void WbMechanismCanvas::setTool(int tool) {
  mTool = tool;
  mPendingParentLinkId = -1;
  clearSelectionMarks();
  switch (tool) {
    case ADD_JOINT_TOOL:
      emit statusMessage(tr("Joint tool: click the parent link, then the child link."));
      break;
    case CLOSE_LOOP_TOOL:
      emit statusMessage(tr("Loop tool: click two already connected links to close the loop."));
      break;
    case DELETE_TOOL:
      emit statusMessage(tr("Delete tool: click a joint to remove it."));
      break;
    default:
      emit statusMessage(tr("Select: click links or joints, drag links to rearrange the diagram."));
  }
  applyLayout();
}

void WbMechanismCanvas::rebuild() {
  mNeedsRebuild = true;
  applyLayout();
}

void WbMechanismCanvas::applyLayout() {
  WbMechanismModel *model = WbMechanismModel::instance();
  const wbmechanism::Graph &graph = model->graph();
  const wbmechanism::Layout layout = model->layout();

  mScene->clear();
  mNeedsRebuild = false;

  // edges first
  QMap<long, WbMechanismLinkItem *> linkItems;
  QMap<long, WbMechanismJointItem *> jointItems;
  for (int i = 0; i < graph.joints().size(); ++i) {
    const wbmechanism::Joint &j = graph.joints()[i];
    std::map<long, std::pair<double, double> >::const_iterator itp = layout.linkPositions.find(j.parentLinkId);
    std::map<long, std::pair<double, double> >::const_iterator itc = layout.linkPositions.find(j.childLinkId);
    if (itp == layout.linkPositions.end() || itc == layout.linkPositions.end())
      continue;
    const QLineF line(QPointF(itp->second.first, itp->second.second), QPointF(itc->second.first, itc->second.second));
    WbMechanismEdgeItem *edge = new WbMechanismEdgeItem(j.isLoopClosure, line);
    mScene->addItem(edge);
  }
  // joints
  for (int i = 0; i < graph.joints().size(); ++i) {
    const wbmechanism::Joint &j = graph.joints()[i];
    std::map<long, std::pair<double, double> >::const_iterator it = layout.jointPositions.find(j.id);
    if (it == layout.jointPositions.end())
      continue;
    WbMechanismJointItem *item =
      new WbMechanismJointItem(j.id, j.isLoopClosure, wbmechanism::jointDof(j.type));
    item->setPos(it->second.first, it->second.second);
    item->setToolTip(QString::fromStdString(j.name) + "\n" + QString::fromUtf8(wbmechanism::jointTypeName(j.type)) +
                     (j.isLoopClosure ? tr("\n(loop closure)") : QString()));
    mScene->addItem(item);
    jointItems[j.id] = item;
  }
  // links
  for (int i = 0; i < graph.links().size(); ++i) {
    const wbmechanism::Link &link = graph.links()[i];
    std::map<long, std::pair<double, double> >::const_iterator it = layout.linkPositions.find(link.id);
    if (it == layout.linkPositions.end())
      continue;
    WbMechanismLinkItem *item = new WbMechanismLinkItem(link.id, link.isStatic, QString::fromStdString(link.name));
    item->setPos(it->second.first, it->second.second);
    if (model->hasUserPosition(QString::fromStdString(link.name)))
      item->setPos(model->userPosition(QString::fromStdString(link.name)));
    mScene->addItem(item);
    linkItems[link.id] = item;
  }
  // keep item references in scene properties via dynamic lookup (items hold their ids)
  clearSelectionMarks();
  mScene->setSceneRect(mScene->itemsBoundingRect().adjusted(-80.0, -80.0, 80.0, 80.0));
  if (!linkItems.isEmpty())
    fitInView(mScene->sceneRect(), Qt::KeepAspectRatio);
}

void WbMechanismCanvas::clearSelectionMarks() {
  foreach (QGraphicsItem *item, mScene->items()) {
    WbMechanismLinkItem *link = dynamic_cast<WbMechanismLinkItem *>(item);
    if (link) {
      link->setSelectedMark(link->linkId() == mSelectedLinkId);
      link->setPending(link->linkId() == mPendingParentLinkId);
    }
    WbMechanismJointItem *joint = dynamic_cast<WbMechanismJointItem *>(item);
    if (joint)
      joint->setSelectedMark(joint->jointId() == mSelectedJointId);
  }
}

void WbMechanismCanvas::setSelectedJoint(long jointId) {
  mSelectedJointId = jointId;
  clearSelectionMarks();
}

void WbMechanismCanvas::setSelectedLink(long linkId) {
  mSelectedLinkId = linkId;
  clearSelectionMarks();
}

WbMechanismLinkItem *WbMechanismCanvas::linkItemAt(const QPoint &viewPos) const {
  foreach (QGraphicsItem *item, items(viewPos))
    if (item->type() == WbMechanismLinkItem::Type)
      return static_cast<WbMechanismLinkItem *>(item);
  return NULL;
}

WbMechanismJointItem *WbMechanismCanvas::jointItemAt(const QPoint &viewPos) const {
  foreach (QGraphicsItem *item, items(viewPos))
    if (item->type() == WbMechanismJointItem::Type)
      return static_cast<WbMechanismJointItem *>(item);
  return NULL;
}

void WbMechanismCanvas::mousePressEvent(QMouseEvent *event) {
  const QPoint pos = event->pos();
  WbMechanismJointItem *joint = jointItemAt(pos);
  WbMechanismLinkItem *link = linkItemAt(pos);

  switch (mTool) {
    case ADD_JOINT_TOOL:
    case CLOSE_LOOP_TOOL: {
      if (link) {
        if (mPendingParentLinkId < 0) {
          mPendingParentLinkId = link->linkId();
          setSelectedLink(mPendingParentLinkId);
          emit statusMessage(tr("Now click the child link."));
        } else if (mPendingParentLinkId != link->linkId()) {
          const long parent = mPendingParentLinkId;
          const long child = link->linkId();
          mPendingParentLinkId = -1;
          emit createJointRequested(parent, child, mTool == CLOSE_LOOP_TOOL);
        }
        clearSelectionMarks();
        return;
      }
      break;
    }
    case DELETE_TOOL: {
      if (joint) {
        emit deleteJointRequested(joint->jointId());
        return;
      }
      break;
    }
    default: {
      if (joint) {
        mSelectedJointId = joint->jointId();
        mSelectedLinkId = -1;
        clearSelectionMarks();
        emit jointSelected(mSelectedJointId);
        return;
      }
      if (link) {
        mSelectedLinkId = link->linkId();
        mSelectedJointId = -1;
        mDraggingLink = true;
        clearSelectionMarks();
        emit linkSelected(mSelectedLinkId);
        QGraphicsView::mousePressEvent(event);  // start item drag
        return;
      }
      // empty canvas: clear selection
      mSelectedLinkId = mSelectedJointId = -1;
      clearSelectionMarks();
      emit linkSelected(-1);
      emit jointSelected(-1);
      break;
    }
  }
  QGraphicsView::mousePressEvent(event);
}

void WbMechanismCanvas::mouseMoveEvent(QMouseEvent *event) {
  QGraphicsView::mouseMoveEvent(event);
}

void WbMechanismCanvas::mouseReleaseEvent(QMouseEvent *event) {
  if (mDraggingLink) {
    mDraggingLink = false;
    WbMechanismModel *model = WbMechanismModel::instance();
    foreach (QGraphicsItem *item, mScene->items()) {
      WbMechanismLinkItem *link = dynamic_cast<WbMechanismLinkItem *>(item);
      if (link && link->linkId() == mSelectedLinkId) {
        const wbmechanism::Link *l = model->graph().link(mSelectedLinkId);
        if (l)
          emit linkPositionChanged(QString::fromStdString(l->name), link->pos());
        break;
      }
    }
  }
  QGraphicsView::mouseReleaseEvent(event);
}

void WbMechanismCanvas::wheelEvent(QWheelEvent *event) {
  const double factor = event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15;
  scale(factor, factor);
}
