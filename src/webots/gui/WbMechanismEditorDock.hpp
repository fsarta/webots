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

// Description: CAD-like Mechanism Editor dock: 2D kinematic diagram with joint
//              and link tools, closed-loop creation, joint property editor and
//              mobility (Gruebler-Kutzbach) validation.

#ifndef WB_MECHANISM_EDITOR_DOCK_HPP
#define WB_MECHANISM_EDITOR_DOCK_HPP

#include "WbDockWidget.hpp"

class WbMechanismCanvas;
class WbNode;

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QToolButton;

class WbMechanismEditorDock : public WbDockWidget {
  Q_OBJECT

public:
  explicit WbMechanismEditorDock(QWidget *parent = NULL);
  virtual ~WbMechanismEditorDock();

  // update the canvas and the analysis report from the current world
  void refresh();

public slots:
  void notifyNodeSelected(WbNode *node);

private slots:
  void onSelectTool() { setTool(0); }
  void onAddJointTool() { setTool(1); }
  void onCloseLoopTool() { setTool(2); }
  void onDeleteTool() { setTool(3); }
  void setTool(int tool);
  void addLinkAndJoint();
  void onCreateJointRequested(long parentLinkId, long childLinkId, bool closeLoop);
  void onDeleteJointRequested(long jointId);
  void onLinkSelected(long linkId);
  void onJointSelected(long jointId);
  void onLinkPositionChanged(const QString &linkName, const QPointF &position);
  void relayout();
  void resetLayout();
  void updateAnalysis();
  void applyJointProperties();
  void applyLinkName();
  void onManipulatorToggled(bool checked);

private:
  QWidget *createJointPanel();
  QWidget *createLinkPanel();
  void fillJointPanel(long jointId);
  void fillLinkPanel(long linkId);
  void showMessage(const QString &message, bool isError = false);

  WbMechanismCanvas *mCanvas;
  QToolButton *mSelectButton;
  QToolButton *mAddJointButton;
  QToolButton *mCloseLoopButton;
  QToolButton *mDeleteButton;
  QComboBox *mJointTypeCombo;
  QPushButton *mAddLinkButton;
  QCheckBox *mPlanarCheckBox;
  QCheckBox *mManipulatorCheckBox;
  QLabel *mAnalysisLabel;
  QLabel *mStatusLabel;
  QStackedWidget *mPropertyStack;
  // joint panel widgets
  QLineEdit *mJointNameLabel;
  QLineEdit *mEndPointLabel;
  QDoubleSpinBox *mAxis[3];
  QDoubleSpinBox *mAnchor[3];
  QCheckBox *mLimitsCheckBox;
  QDoubleSpinBox *mMinStop;
  QDoubleSpinBox *mMaxStop;
  QDoubleSpinBox *mSpring;
  QDoubleSpinBox *mDamping;
  QLabel *mPositionLabel;
  // link panel widgets
  QLineEdit *mLinkNameEdit;

  long mCurrentJointId;
  long mCurrentLinkId;
  bool mFillingPanels;
};

#endif
