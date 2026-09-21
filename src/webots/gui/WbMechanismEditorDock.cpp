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

#include "WbMechanismEditorDock.hpp"

#include <QtGui/QCloseEvent>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>

#include "WbBasicJoint.hpp"
#include "WbKinematicGraphCore.hpp"
#include "WbJointManipulator.hpp"
#include "WbMechanismCanvas.hpp"
#include "WbMechanismModel.hpp"
#include "WbNode.hpp"
#include "WbSelection.hpp"
#include "WbSolid.hpp"
#include "WbVector3.hpp"

WbMechanismEditorDock::WbMechanismEditorDock(QWidget *parent) :
  WbDockWidget(parent),
  mCurrentJointId(-1),
  mCurrentLinkId(-1),
  mFillingPanels(false) {
  setWindowTitle(tr("Mechanism Editor"));
  setObjectName("MechanismEditorWidget");  // for dock perspective

  QWidget *container = new QWidget(this);
  QVBoxLayout *layout = new QVBoxLayout(container);
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(4);

  // toolbar
  QToolBar *toolBar = new QToolBar(container);
  toolBar->setIconSize(QSize(20, 20));
  toolBar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  mSelectButton = new QToolButton(toolBar);
  mSelectButton->setText(tr("Select"));
  mSelectButton->setCheckable(true);
  mSelectButton->setChecked(true);
  connect(mSelectButton, &QToolButton::clicked, this, &WbMechanismEditorDock::onSelectTool);
  toolBar->addWidget(mSelectButton);
  mAddJointButton = new QToolButton(toolBar);
  mAddJointButton->setText(tr("Joint"));
  mAddJointButton->setCheckable(true);
  mAddJointButton->setToolTip(tr("Create a joint between two existing links (click parent, then child)."));
  connect(mAddJointButton, &QToolButton::clicked, this, &WbMechanismEditorDock::onAddJointTool);
  toolBar->addWidget(mAddJointButton);
  mCloseLoopButton = new QToolButton(toolBar);
  mCloseLoopButton->setText(tr("Close Loop"));
  mCloseLoopButton->setCheckable(true);
  mCloseLoopButton->setToolTip(tr("Close a kinematic loop between two links already connected through the tree."));
  connect(mCloseLoopButton, &QToolButton::clicked, this, &WbMechanismEditorDock::onCloseLoopTool);
  toolBar->addWidget(mCloseLoopButton);
  mDeleteButton = new QToolButton(toolBar);
  mDeleteButton->setText(tr("Delete"));
  mDeleteButton->setCheckable(true);
  connect(mDeleteButton, &QToolButton::clicked, this, &WbMechanismEditorDock::onDeleteTool);
  toolBar->addWidget(mDeleteButton);
  toolBar->addSeparator();
  mJointTypeCombo = new QComboBox(toolBar);
  mJointTypeCombo->addItem(tr("Hinge"), 1);      // REVOLUTE_JOINT
  mJointTypeCombo->addItem(tr("Hinge2"), 3);     // REVOLUTE2_JOINT
  mJointTypeCombo->addItem(tr("Slider"), 2);     // PRISMATIC_JOINT
  mJointTypeCombo->addItem(tr("Ball"), 4);       // SPHERICAL_JOINT
  toolBar->addWidget(mJointTypeCombo);
  mAddLinkButton = new QPushButton(tr("Add Link + Joint"), toolBar);
  mAddLinkButton->setToolTip(tr("Create a new link attached to the selected link with a new joint."));
  connect(mAddLinkButton, &QPushButton::clicked, this, &WbMechanismEditorDock::addLinkAndJoint);
  toolBar->addWidget(mAddLinkButton);
  toolBar->addSeparator();
  QPushButton *relayoutButton = new QPushButton(tr("Auto Layout"), toolBar);
  connect(relayoutButton, &QPushButton::clicked, this, &WbMechanismEditorDock::relayout);
  toolBar->addWidget(relayoutButton);
  QPushButton *resetButton = new QPushButton(tr("Reset Layout"), toolBar);
  connect(resetButton, &QPushButton::clicked, this, &WbMechanismEditorDock::resetLayout);
  toolBar->addWidget(resetButton);
  toolBar->addSeparator();
  mPlanarCheckBox = new QCheckBox(tr("Planar"), toolBar);
  mPlanarCheckBox->setToolTip(tr("Use planar (3-DOF) mobility analysis instead of spatial (6-DOF)."));
  connect(mPlanarCheckBox, &QCheckBox::toggled, this, &WbMechanismEditorDock::updateAnalysis);
  toolBar->addWidget(mPlanarCheckBox);
  mManipulatorCheckBox = new QCheckBox(tr("3D handles"), toolBar);
  mManipulatorCheckBox->setToolTip(tr("Show Fusion-style anchor/axis manipulators on the selected joint in the 3D view."));
  connect(mManipulatorCheckBox, &QCheckBox::toggled, this, &WbMechanismEditorDock::onManipulatorToggled);
  toolBar->addWidget(mManipulatorCheckBox);
  layout->addWidget(toolBar);

  // canvas + property panel
  mCanvas = new WbMechanismCanvas(container);
  connect(mCanvas, &WbMechanismCanvas::linkSelected, this, &WbMechanismEditorDock::onLinkSelected);
  connect(mCanvas, &WbMechanismCanvas::jointSelected, this, &WbMechanismEditorDock::onJointSelected);
  connect(mCanvas, &WbMechanismCanvas::createJointRequested, this, &WbMechanismEditorDock::onCreateJointRequested);
  connect(mCanvas, &WbMechanismCanvas::deleteJointRequested, this, &WbMechanismEditorDock::onDeleteJointRequested);
  connect(mCanvas, &WbMechanismCanvas::linkPositionChanged, this, &WbMechanismEditorDock::onLinkPositionChanged);
  connect(mCanvas, &WbMechanismCanvas::statusMessage, this, &WbMechanismEditorDock::showMessage);

  mPropertyStack = new QStackedWidget(container);
  mPropertyStack->addWidget(new QLabel(tr("Select a link or a joint to edit its properties."), mPropertyStack));
  mPropertyStack->addWidget(createJointPanel());
  mPropertyStack->addWidget(createLinkPanel());
  mPropertyStack->setMaximumWidth(310);

  QScrollArea *propertyArea = new QScrollArea(container);
  propertyArea->setWidget(mPropertyStack);
  propertyArea->setWidgetResizable(true);
  propertyArea->setMaximumWidth(330);

  QHBoxLayout *middle = new QHBoxLayout();
  middle->addWidget(mCanvas, 1);
  middle->addWidget(propertyArea, 0);
  layout->addLayout(middle, 1);

  // analysis report
  mAnalysisLabel = new QLabel(container);
  mAnalysisLabel->setWordWrap(true);
  layout->addWidget(mAnalysisLabel);
  mStatusLabel = new QLabel(tr("Ready."), container);
  mStatusLabel->setWordWrap(true);
  layout->addWidget(mStatusLabel);
  setWidget(container);

  refresh();
}

WbMechanismEditorDock::~WbMechanismEditorDock() {
}

QWidget *WbMechanismEditorDock::createJointPanel() {
  QWidget *panel = new QWidget(this);
  QFormLayout *form = new QFormLayout(panel);
  mJointNameLabel = new QLineEdit(panel);
  mJointNameLabel->setReadOnly(true);
  form->addRow(tr("Joint:"), mJointNameLabel);
  mEndPointLabel = new QLineEdit(panel);
  mEndPointLabel->setReadOnly(true);
  form->addRow(tr("Endpoint:"), mEndPointLabel);
  QWidget *axisBox = new QWidget(panel);
  QHBoxLayout *axisLayout = new QHBoxLayout(axisBox);
  axisLayout->setContentsMargins(0, 0, 0, 0);
  for (int i = 0; i < 3; ++i) {
    mAxis[i] = new QDoubleSpinBox(axisBox);
    mAxis[i]->setRange(-1000.0, 1000.0);
    mAxis[i]->setDecimals(3);
    connect(mAxis[i], &QDoubleSpinBox::editingFinished, this, &WbMechanismEditorDock::applyJointProperties);
    axisLayout->addWidget(mAxis[i]);
  }
  form->addRow(tr("Axis:"), axisBox);
  QWidget *anchorBox = new QWidget(panel);
  QHBoxLayout *anchorLayout = new QHBoxLayout(anchorBox);
  anchorLayout->setContentsMargins(0, 0, 0, 0);
  for (int i = 0; i < 3; ++i) {
    mAnchor[i] = new QDoubleSpinBox(anchorBox);
    mAnchor[i]->setRange(-10000.0, 10000.0);
    mAnchor[i]->setDecimals(4);
    mAnchor[i]->setSingleStep(0.01);
    connect(mAnchor[i], &QDoubleSpinBox::editingFinished, this, &WbMechanismEditorDock::applyJointProperties);
    anchorLayout->addWidget(mAnchor[i]);
  }
  form->addRow(tr("Anchor:"), anchorBox);
  mLimitsCheckBox = new QCheckBox(tr("limited"), panel);
  connect(mLimitsCheckBox, &QCheckBox::toggled, this, &WbMechanismEditorDock::applyJointProperties);
  form->addRow(tr("Stops:"), mLimitsCheckBox);
  mMinStop = new QDoubleSpinBox(panel);
  mMinStop->setRange(-10000.0, 10000.0);
  mMinStop->setDecimals(4);
  connect(mMinStop, &QDoubleSpinBox::editingFinished, this, &WbMechanismEditorDock::applyJointProperties);
  form->addRow(tr("minStop:"), mMinStop);
  mMaxStop = new QDoubleSpinBox(panel);
  mMaxStop->setRange(-10000.0, 10000.0);
  mMaxStop->setDecimals(4);
  connect(mMaxStop, &QDoubleSpinBox::editingFinished, this, &WbMechanismEditorDock::applyJointProperties);
  form->addRow(tr("maxStop:"), mMaxStop);
  mSpring = new QDoubleSpinBox(panel);
  mSpring->setRange(0.0, 1e9);
  mSpring->setDecimals(2);
  connect(mSpring, &QDoubleSpinBox::editingFinished, this, &WbMechanismEditorDock::applyJointProperties);
  form->addRow(tr("springConstant:"), mSpring);
  mDamping = new QDoubleSpinBox(panel);
  mDamping->setRange(0.0, 1e9);
  mDamping->setDecimals(2);
  connect(mDamping, &QDoubleSpinBox::editingFinished, this, &WbMechanismEditorDock::applyJointProperties);
  form->addRow(tr("dampingConstant:"), mDamping);
  mPositionLabel = new QLabel("-", panel);
  form->addRow(tr("position:"), mPositionLabel);
  return panel;
}

QWidget *WbMechanismEditorDock::createLinkPanel() {
  QWidget *panel = new QWidget(this);
  QFormLayout *form = new QFormLayout(panel);
  mLinkNameEdit = new QLineEdit(panel);
  connect(mLinkNameEdit, &QLineEdit::editingFinished, this, &WbMechanismEditorDock::applyLinkName);
  form->addRow(tr("name:"), mLinkNameEdit);
  return panel;
}

void WbMechanismEditorDock::refresh() {
  WbMechanismModel::instance()->rebuild();
  mCanvas->rebuild();
  updateAnalysis();
}

void WbMechanismEditorDock::notifyNodeSelected(WbNode *node) {
  WbMechanismModel *model = WbMechanismModel::instance();
  WbSolid *solid = dynamic_cast<WbSolid *>(node);
  if (solid) {
    const long id = model->linkIdOfSolid(solid);
    if (id >= 0)
      onLinkSelected(id);
    return;
  }
  WbBasicJoint *joint = dynamic_cast<WbBasicJoint *>(node);
  if (joint) {
    const long id = model->jointIdOfJoint(joint);
    if (id >= 0)
      onJointSelected(id);
  }
}

void WbMechanismEditorDock::setTool(int tool) {
  mSelectButton->setChecked(tool == 0);
  mAddJointButton->setChecked(tool == 1);
  mCloseLoopButton->setChecked(tool == 2);
  mDeleteButton->setChecked(tool == 3);
  mCanvas->setTool(tool);
}

void WbMechanismEditorDock::addLinkAndJoint() {
  WbMechanismModel *model = WbMechanismModel::instance();
  if (mCurrentLinkId < 0) {
    showMessage(tr("Select the parent link first."), true);
    return;
  }
  QString error;
  const int type = mJointTypeCombo->currentData().toInt();
  if (!model->addLinkAndJoint(mCurrentLinkId, (wbmechanism::JointType)type, QString(), error))
    showMessage(error, true);
  else {
    showMessage(tr("New link and joint created."), false);
    refresh();
  }
}

void WbMechanismEditorDock::onCreateJointRequested(long parentLinkId, long childLinkId, bool closeLoop) {
  WbMechanismModel *model = WbMechanismModel::instance();
  // for the loop tool, verify that both links are already connected
  if (closeLoop) {
    const wbmechanism::Analysis analysis = model->analyze(mPlanarCheckBox->isChecked());
    if (!analysis.isConnected)
      showMessage(tr("Note: the two links are not connected yet: this joint will join two sub-mechanisms."), false);
    else
      showMessage(tr("Closing a kinematic loop: the joint will use a SolidReference endpoint."), false);
  }
  // choose the joint type
  QDialog typeDialog(this);
  typeDialog.setWindowTitle(tr("Joint type"));
  QVBoxLayout *typeLayout = new QVBoxLayout(&typeDialog);
  QComboBox *typeCombo = new QComboBox(&typeDialog);
  typeCombo->addItem(tr("Hinge"), 1);
  typeCombo->addItem(tr("Hinge2"), 3);
  typeCombo->addItem(tr("Slider"), 2);
  typeCombo->addItem(tr("Ball"), 4);
  typeLayout->addWidget(typeCombo);
  QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &typeDialog);
  connect(buttons, &QDialogButtonBox::accepted, &typeDialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &typeDialog, &QDialog::reject);
  typeLayout->addWidget(buttons);
  if (typeDialog.exec() != QDialog::Accepted) {
    mCanvas->setTool(mCanvas->tool());
    return;
  }
  QString error;
  const bool ok = model->connectLinks(parentLinkId, childLinkId, (wbmechanism::JointType)typeCombo->currentData().toInt(),
                                      error);
  if (!ok)
    showMessage(error, true);
  else {
    if (closeLoop)
      showMessage(tr("Joint created with SolidReference endpoint (closed kinematic chain)."), false);
    else
      showMessage(tr("Joint created."), false);
  }
  refresh();
  mCanvas->setTool(mCanvas->tool());
}

void WbMechanismEditorDock::onDeleteJointRequested(long jointId) {
  QString error;
  if (!WbMechanismModel::instance()->deleteJoint(jointId, error))
    showMessage(error, true);
  else {
    showMessage(tr("Joint deleted."), false);
    mCurrentJointId = -1;
    refresh();
  }
}

void WbMechanismEditorDock::onLinkSelected(long linkId) {
  mCurrentLinkId = linkId;
  mCurrentJointId = -1;
  mCanvas->setSelectedLink(linkId);
  if (linkId < 0) {
    mPropertyStack->setCurrentIndex(0);
    return;
  }
  fillLinkPanel(linkId);
  mPropertyStack->setCurrentIndex(2);
  WbMechanismModel *model = WbMechanismModel::instance();
  WbSolid *solid = model->solidOfLink(linkId);
  if (solid)
    model->requestSelection(solid);
}

void WbMechanismEditorDock::onJointSelected(long jointId) {
  mCurrentJointId = jointId;
  mCurrentLinkId = -1;
  mCanvas->setSelectedJoint(jointId);
  if (jointId < 0) {
    mPropertyStack->setCurrentIndex(0);
    return;
  }
  fillJointPanel(jointId);
  mPropertyStack->setCurrentIndex(1);
  WbMechanismModel *model = WbMechanismModel::instance();
  WbBasicJoint *joint = model->jointOfJoint(jointId);
  if (joint)
    model->requestSelection(joint);
}

void WbMechanismEditorDock::onLinkPositionChanged(const QString &linkName, const QPointF &position) {
  WbMechanismModel::instance()->setUserPosition(linkName, position);
  showMessage(tr("Diagram layout updated."), false);
}

void WbMechanismEditorDock::fillJointPanel(long jointId) {
  WbMechanismModel::JointProperties p;
  if (!WbMechanismModel::instance()->jointProperties(jointId, p))
    return;
  mFillingPanels = true;
  QString title = QString::fromUtf8(wbmechanism::jointTypeName(p.type));
  if (p.isLoopClosure)
    title += tr(" — loop closure");
  mJointNameLabel->setText(title);
  mEndPointLabel->setText(p.endPointName.isEmpty() ? tr("<none>") : p.endPointName);
  mAxis[0]->setValue(p.axis.x());
  mAxis[1]->setValue(p.axis.y());
  mAxis[2]->setValue(p.axis.z());
  mAnchor[0]->setValue(p.anchor.x());
  mAnchor[1]->setValue(p.anchor.y());
  mAnchor[2]->setValue(p.anchor.z());
  for (int i = 0; i < 3; ++i)
    mAnchor[i]->setEnabled(p.hasAnchor);
  mLimitsCheckBox->setChecked(p.hasLimits);
  mMinStop->setValue(p.minStop);
  mMaxStop->setValue(p.maxStop);
  mMinStop->setEnabled(p.hasLimits);
  mMaxStop->setEnabled(p.hasLimits);
  mSpring->setValue(p.springConstant);
  mDamping->setValue(p.dampingConstant);
  mPositionLabel->setText(QString::number(p.position, 'g', 4));
  mFillingPanels = false;
}

void WbMechanismEditorDock::fillLinkPanel(long linkId) {
  WbMechanismModel *model = WbMechanismModel::instance();
  WbSolid *solid = model->solidOfLink(linkId);
  if (!solid)
    return;
  mFillingPanels = true;
  mLinkNameEdit->setText(solid->name());
  mFillingPanels = false;
}

void WbMechanismEditorDock::applyJointProperties() {
  if (mFillingPanels || mCurrentJointId < 0)
    return;
  WbMechanismModel *model = WbMechanismModel::instance();
  QString error;
  if (!model->setJointAxis(mCurrentJointId, WbVector3(mAxis[0]->value(), mAxis[1]->value(), mAxis[2]->value()), error)) {
    showMessage(error, true);
    return;
  }
  if (mAnchor[0]->isEnabled() &&
      !model->setJointAnchor(mCurrentJointId, WbVector3(mAnchor[0]->value(), mAnchor[1]->value(), mAnchor[2]->value()),
                             error)) {
    showMessage(error, true);
    return;
  }
  if (!model->setJointLimits(mCurrentJointId, mLimitsCheckBox->isChecked(), mMinStop->value(), mMaxStop->value(),
                             error)) {
    showMessage(error, true);
    return;
  }
  if (!model->setJointSpringDamping(mCurrentJointId, mSpring->value(), mDamping->value(), error)) {
    showMessage(error, true);
    return;
  }
  showMessage(tr("Joint properties applied."), false);
  updateAnalysis();
}

void WbMechanismEditorDock::applyLinkName() {
  if (mFillingPanels || mCurrentLinkId < 0)
    return;
  QString error;
  if (!WbMechanismModel::instance()->setLinkName(mCurrentLinkId, mLinkNameEdit->text(), error)) {
    showMessage(error, true);
    return;
  }
  showMessage(tr("Link renamed."), false);
  refresh();
}

void WbMechanismEditorDock::relayout() {
  WbMechanismModel::instance()->resetUserLayout();
  showMessage(tr("Automatic layout applied."), false);
}

void WbMechanismEditorDock::resetLayout() {
  WbMechanismModel::instance()->resetUserLayout();
  showMessage(tr("Layout reset."), false);
}

void WbMechanismEditorDock::updateAnalysis() {
  WbMechanismModel *model = WbMechanismModel::instance();
  const bool planar = mPlanarCheckBox->isChecked();
  const wbmechanism::Analysis analysis = model->analyze(planar);
  QString text = "<b>" + QString::fromStdString(analysis.summary()) + "</b>";
  if (!analysis.warnings.empty()) {
    text += "<ul style='margin:2px'>";
    for (size_t i = 0; i < analysis.warnings.size(); ++i)
      text += "<li>" + QString::fromStdString(analysis.warnings[i]) + "</li>";
    text += "</ul>";
  }
  mAnalysisLabel->setText(text);
  const wbmechanism::Graph &graph = model->graph();
  if (graph.isEmpty())
    showMessage(tr("The world contains no mechanism. Select a robot or create links with 'Add Link + Joint'."), false);
}

void WbMechanismEditorDock::showMessage(const QString &message, bool isError) {
  mStatusLabel->setText(isError ? "<span style='color:#b00000'>" + message + "</span>" : message);
}

void WbMechanismEditorDock::onManipulatorToggled(bool checked) {
  WbJointManipulator *manipulator = WbJointManipulator::instance();
  if (manipulator)
    manipulator->setActive(checked);
  if (checked)
    showMessage(tr("3D handles active: drag the anchor cross or the axis arrow of the selected joint."), false);
}
