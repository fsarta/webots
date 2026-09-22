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

// Description: Robot Creator window: build a robot starting from STL/OBJ mesh
//              files and place its joints on CAD-style snap points (endpoint,
//              midpoint, circle center, centroid, grid). Exports a Webots robot
//              with closed kinematic chain support (SolidReference joints).

#ifndef WB_ROBOT_CREATOR_WINDOW_HPP
#define WB_ROBOT_CREATOR_WINDOW_HPP

#include "WbRobotProtoCore.hpp"
#include "WbSnapCore.hpp"

#include <QtWidgets/QMainWindow>

class QActionGroup;
class QComboBox;
class QDoubleSpinBox;
class QTreeWidget;
class QTreeWidgetItem;
class QLabel;
class QLineEdit;
class QCheckBox;
class WbMeshViewport;

class WbRobotCreatorWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit WbRobotCreatorWindow(QWidget *parent = NULL);
  virtual ~WbRobotCreatorWindow();

private slots:
  void importMesh();
  void addEmptyLink();
  void removeSelected();
  void exportRobot();
  void copyToClipboard();
  void startJointPlacement();
  void onSnapPicked(const wbsnap::SnapCandidate &candidate, int partIndex);
  void onPlacementCancelled();
  void onTreeSelectionChanged();
  void applyProperties();
  void toggleAllSnaps();
  void clearAllSnaps();

private:
  void createActions();
  void rebuildTree();
  void rebuildProperties();
  void updateValidation();
  void showMessage(const QString &message, bool isError = false);
  void markChanged();
  wbrobotproto::Part *selectedPart();
  wbrobotproto::JointDef *selectedJoint();
  QString uniquePartName(const QString &base) const;

  wbrobotproto::Assembly mAssembly;

  WbMeshViewport *mViewport;
  QTreeWidget *mTree;
  QLabel *mStatusLabel;
  QLabel *mValidationLabel;
  QCheckBox *mPlanarCheck;

  // joint property editor
  QWidget *mJointProperties;
  QLineEdit *mJointNameEdit;
  QComboBox *mJointTypeCombo;
  QComboBox *mJointParentCombo;
  QComboBox *mJointChildCombo;
  QDoubleSpinBox *mAnchorSpin[3];
  QDoubleSpinBox *mAxisSpin[3];

  // part property editor
  QWidget *mPartProperties;
  QLineEdit *mPartNameEdit;
  QDoubleSpinBox *mPartTranslationSpin[3];

  int mPendingJointType;  // -1 = not placing a joint
  int mJointCounter;

  QActionGroup *mSnapActionGroup;
};

#endif
