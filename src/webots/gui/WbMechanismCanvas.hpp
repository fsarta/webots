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

// Description: schematic 2D canvas of the Mechanism Editor: links are blocks,
//              joints are glyphs on the edges, closed kinematic chains are
//              dashed arcs. Drag & drop in the style of a CAD joint diagram.

#ifndef WB_MECHANISM_CANVAS_HPP
#define WB_MECHANISM_CANVAS_HPP

#include <QtWidgets/QGraphicsView>

class QGraphicsEllipseItem;
class QGraphicsPathItem;
class QGraphicsScene;
class QGraphicsSimpleTextItem;
class WbMechanismEdgeItem;
class WbMechanismJointItem;
class WbMechanismLinkItem;

class WbMechanismCanvas : public QGraphicsView {
  Q_OBJECT

public:
  enum Tool {
    SELECT_TOOL = 0,
    ADD_JOINT_TOOL,
    CLOSE_LOOP_TOOL,
    DELETE_TOOL,
  };

  explicit WbMechanismCanvas(QWidget *parent = NULL);
  virtual ~WbMechanismCanvas();

  void setTool(int tool);
  int tool() const { return mTool; }

  // currently selected graph ids (-1 when none)
  long selectedLinkId() const { return mSelectedLinkId; }
  long selectedJointId() const { return mSelectedJointId; }
  void setSelectedJoint(long jointId);
  void setSelectedLink(long linkId);

public slots:
  // rebuild all items from the mechanism model
  void rebuild();
  void applyLayout();

signals:
  void linkSelected(long linkId);
  void jointSelected(long jointId);
  void linkPositionChanged(const QString &linkName, const QPointF &position);
  void createJointRequested(long parentLinkId, long childLinkId, bool closeLoop);
  void deleteJointRequested(long jointId);
  void statusMessage(const QString &message);

protected:
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

private:
  WbMechanismLinkItem *linkItemAt(const QPoint &viewPos) const;
  WbMechanismJointItem *jointItemAt(const QPoint &viewPos) const;
  void clearSelectionMarks();

  QGraphicsScene *mScene;
  int mTool;
  long mSelectedLinkId;
  long mSelectedJointId;
  long mPendingParentLinkId;  // first pick of the two-click joint creation flow
  bool mDraggingLink;
  bool mNeedsRebuild;

  friend class WbMechanismLinkItem;
  friend class WbMechanismJointItem;
  friend class WbMechanismEdgeItem;
};

#endif
