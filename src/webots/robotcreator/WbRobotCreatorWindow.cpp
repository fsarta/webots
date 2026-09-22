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

#include "WbRobotCreatorWindow.hpp"

#include "WbMeshViewport.hpp"

#include <QtGui/QActionGroup>
#include <QtGui/QClipboard>
#include <QtGui/QGuiApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QInputDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTreeWidget>
#include <QtWidgets/QVBoxLayout>

#include <algorithm>
#include <fstream>

static const int PART_ITEM_TYPE = QTreeWidgetItem::UserType;
static const int JOINT_ITEM_TYPE = QTreeWidgetItem::UserType + 1;

static const char *JOINT_NAMES[] = {"Hinge", "Slider", "Hinge2", "Ball", "Fixed"};

static int jointDofIndex(wbmechanism::JointType type) {
  switch (type) {
    case wbmechanism::REVOLUTE_JOINT:
      return 0;
    case wbmechanism::PRISMATIC_JOINT:
      return 1;
    case wbmechanism::REVOLUTE2_JOINT:
      return 2;
    case wbmechanism::SPHERICAL_JOINT:
      return 3;
    default:
      return 4;
  }
}

static wbmechanism::JointType jointTypeOf(int index) {
  switch (index) {
    case 0:
      return wbmechanism::REVOLUTE_JOINT;
    case 1:
      return wbmechanism::PRISMATIC_JOINT;
    case 2:
      return wbmechanism::REVOLUTE2_JOINT;
    case 3:
      return wbmechanism::SPHERICAL_JOINT;
    default:
      return wbmechanism::FIXED_JOINT;
  }
}

WbRobotCreatorWindow::WbRobotCreatorWindow(QWidget *parent) :
  QMainWindow(parent),
  mViewport(NULL),
  mTree(NULL),
  mStatusLabel(NULL),
  mValidationLabel(NULL),
  mPlanarCheck(NULL),
  mJointProperties(NULL),
  mJointNameEdit(NULL),
  mJointTypeCombo(NULL),
  mJointParentCombo(NULL),
  mJointChildCombo(NULL),
  mPartProperties(NULL),
  mPartNameEdit(NULL),
  mPendingJointType(-1),
  mJointCounter(0),
  mSnapActionGroup(NULL) {
  setWindowTitle(tr("Robot Creator"));
  setObjectName("RobotCreatorWindow");
  setWindowFlag(Qt::Window);  // dedicated top-level window
  resize(1280, 800);

  mViewport = new WbMeshViewport(this);
  mViewport->setAssembly(&mAssembly);
  setCentralWidget(mViewport);
  connect(mViewport, &WbMeshViewport::statusMessage, this, [this](const QString &message) { showMessage(message); });
  connect(mViewport, &WbMeshViewport::snapPicked, this, &WbRobotCreatorWindow::onSnapPicked);
  connect(mViewport, &WbMeshViewport::placementCancelled, this, &WbRobotCreatorWindow::onPlacementCancelled);

  // assembly dock: tree + properties + validation
  QDockWidget *dock = new QDockWidget(tr("Assembly"), this);
  dock->setObjectName("RobotCreatorAssemblyDock");
  QWidget *container = new QWidget(dock);
  QVBoxLayout *layout = new QVBoxLayout(container);

  mTree = new QTreeWidget(container);
  mTree->setHeaderLabels(QStringList() << tr("name") << tr("info"));
  mTree->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(mTree, &QTreeWidget::itemSelectionChanged, this, &WbRobotCreatorWindow::onTreeSelectionChanged);
  layout->addWidget(mTree, 1);

  QHBoxLayout *buttons = new QHBoxLayout();
  QPushButton *importButton = new QPushButton(tr("Import Mesh..."), container);
  connect(importButton, &QPushButton::clicked, this, &WbRobotCreatorWindow::importMesh);
  buttons->addWidget(importButton);
  QPushButton *emptyButton = new QPushButton(tr("Empty Link"), container);
  connect(emptyButton, &QPushButton::clicked, this, &WbRobotCreatorWindow::addEmptyLink);
  buttons->addWidget(emptyButton);
  QPushButton *removeButton = new QPushButton(tr("Remove"), container);
  connect(removeButton, &QPushButton::clicked, this, &WbRobotCreatorWindow::removeSelected);
  buttons->addWidget(removeButton);
  layout->addLayout(buttons);

  // joint properties
  mJointProperties = new QWidget(container);
  QFormLayout *jointForm = new QFormLayout(mJointProperties);
  mJointNameEdit = new QLineEdit(mJointProperties);
  jointForm->addRow(tr("name"), mJointNameEdit);
  mJointTypeCombo = new QComboBox(mJointProperties);
  for (int i = 0; i < 5; ++i)
    mJointTypeCombo->addItem(QString(JOINT_NAMES[i]) + tr(" joint"));
  jointForm->addRow(tr("type"), mJointTypeCombo);
  mJointParentCombo = new QComboBox(mJointProperties);
  jointForm->addRow(tr("parent link"), mJointParentCombo);
  mJointChildCombo = new QComboBox(mJointProperties);
  jointForm->addRow(tr("child link"), mJointChildCombo);
  for (int i = 0; i < 3; ++i) {
    mAnchorSpin[i] = new QDoubleSpinBox(mJointProperties);
    mAnchorSpin[i]->setDecimals(6);
    mAnchorSpin[i]->setRange(-1000, 1000);
    mAnchorSpin[i]->setSingleStep(0.01);
    mAxisSpin[i] = new QDoubleSpinBox(mJointProperties);
    mAxisSpin[i]->setDecimals(6);
    mAxisSpin[i]->setRange(-1, 1);
    mAxisSpin[i]->setSingleStep(0.1);
  }
  jointForm->addRow(tr("anchor x"), mAnchorSpin[0]);
  jointForm->addRow(tr("anchor y"), mAnchorSpin[1]);
  jointForm->addRow(tr("anchor z"), mAnchorSpin[2]);
  jointForm->addRow(tr("axis x"), mAxisSpin[0]);
  jointForm->addRow(tr("axis y"), mAxisSpin[1]);
  jointForm->addRow(tr("axis z"), mAxisSpin[2]);
  QPushButton *applyButton = new QPushButton(tr("Apply"), mJointProperties);
  connect(applyButton, &QPushButton::clicked, this, &WbRobotCreatorWindow::applyProperties);
  jointForm->addRow(applyButton);
  mJointProperties->setVisible(false);
  layout->addWidget(mJointProperties);

  // part properties
  mPartProperties = new QWidget(container);
  QFormLayout *partForm = new QFormLayout(mPartProperties);
  mPartNameEdit = new QLineEdit(mPartProperties);
  partForm->addRow(tr("name"), mPartNameEdit);
  for (int i = 0; i < 3; ++i) {
    mPartTranslationSpin[i] = new QDoubleSpinBox(mPartProperties);
    mPartTranslationSpin[i]->setDecimals(6);
    mPartTranslationSpin[i]->setRange(-1000, 1000);
    mPartTranslationSpin[i]->setSingleStep(0.01);
    partForm->addRow(QString(tr("translation %1")).arg("xyz"[i]), mPartTranslationSpin[i]);
  }
  QPushButton *applyPartButton = new QPushButton(tr("Apply"), mPartProperties);
  connect(applyPartButton, &QPushButton::clicked, this, &WbRobotCreatorWindow::applyProperties);
  partForm->addRow(applyPartButton);
  mPartProperties->setVisible(false);
  layout->addWidget(mPartProperties);

  mPlanarCheck = new QCheckBox(tr("Planar mechanism (mobility analysis)"), container);
  mPlanarCheck->setChecked(true);
  connect(mPlanarCheck, &QCheckBox::toggled, this, [this](bool) { updateValidation(); });
  layout->addWidget(mPlanarCheck);
  mValidationLabel = new QLabel(tr("Import a mesh to start."), container);
  mValidationLabel->setWordWrap(true);
  layout->addWidget(mValidationLabel);

  container->setLayout(layout);
  dock->setWidget(container);
  addDockWidget(Qt::RightDockWidgetArea, dock);

  createActions();
  mStatusLabel = new QLabel(this);
  statusBar()->addPermanentWidget(mStatusLabel, 1);
  showMessage(tr("Import STL/OBJ meshes, then insert joints with CAD snaps (Tools in the Joint menu)."));
  mViewport->resetCamera();
}

WbRobotCreatorWindow::~WbRobotCreatorWindow() {
}

void WbRobotCreatorWindow::createActions() {
  QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
  QAction *action = fileMenu->addAction(tr("Import &Mesh..."), this, &WbRobotCreatorWindow::importMesh);
  action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_I));
  action = fileMenu->addAction(tr("Add Empty &Link"), this, &WbRobotCreatorWindow::addEmptyLink);
  action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_L));
  fileMenu->addSeparator();
  action = fileMenu->addAction(tr("&Export Robot..."), this, &WbRobotCreatorWindow::exportRobot);
  action->setShortcut(QKeySequence::Save);
  action = fileMenu->addAction(tr("&Copy .wbt to Clipboard"), this, &WbRobotCreatorWindow::copyToClipboard);
  action->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_C));
  fileMenu->addSeparator();
  fileMenu->addAction(tr("&Close"), this, &QWidget::close, QKeySequence(Qt::CTRL | Qt::Key_W));

  QMenu *jointMenu = menuBar()->addMenu(tr("&Joint"));
  struct Entry {
    const char *text;
    int type;
    Qt::Key key;
  } entries[5] = {
    {"Insert &Hinge Joint", 0, Qt::Key_H}, {"Insert &Slider Joint", 1, Qt::Key_S},
    {"Insert Hinge&2 Joint", 2, Qt::Key_2}, {"Insert &Ball Joint", 3, Qt::Key_B},
    {"Insert Fi&xed Joint", 4, Qt::Key_F}};
  for (int i = 0; i < 5; ++i) {
    QAction *jointAction = jointMenu->addAction(tr(entries[i].text));
    jointAction->setShortcut(QKeySequence(Qt::SHIFT | entries[i].key));
    jointAction->setData(entries[i].type);
    connect(jointAction, &QAction::triggered, this, &WbRobotCreatorWindow::startJointPlacement);
  }
  jointMenu->addSeparator();
  jointMenu->addAction(tr("Cancel Placement"), this, &WbRobotCreatorWindow::onPlacementCancelled,
                       QKeySequence(Qt::Key_Escape));

  QMenu *snapMenu = menuBar()->addMenu(tr("&Snap"));
  struct SnapEntry {
    const char *text;
    int mode;
    Qt::Key key;
  } snaps[5] = {
    {"&Endpoints (vertices)", wbsnap::SNAP_ENDPOINT, Qt::Key_F1},
    {"&Midpoints (edges)", wbsnap::SNAP_MIDPOINT, Qt::Key_F2},
    {"&Circle Centers", wbsnap::SNAP_CIRCLE_CENTER, Qt::Key_F3},
    {"Centroi&ds (medians)", wbsnap::SNAP_CENTROID, Qt::Key_F4},
    {"&Grid", wbsnap::SNAP_GRID, Qt::Key_F5}};
  mSnapActionGroup = new QActionGroup(this);
  mSnapActionGroup->setExclusive(false);
  for (int i = 0; i < 5; ++i) {
    QAction *snapAction = snapMenu->addAction(tr(snaps[i].text));
    snapAction->setCheckable(true);
    snapAction->setChecked(true);
    snapAction->setShortcut(QKeySequence(snaps[i].key));
    snapAction->setData(snaps[i].mode);
    mSnapActionGroup->addAction(snapAction);
    connect(snapAction, &QAction::toggled, this, [this](bool checked) {
      int modes = mViewport->snapModes();
      QAction *sender = qobject_cast<QAction *>(QObject::sender());
      const int mode = sender->data().toInt();
      modes = checked ? modes | mode : modes & ~mode;
      mViewport->setSnapModes(modes);
    });
  }
  snapMenu->addSeparator();
  snapMenu->addAction(tr("Enable &All"), this, &WbRobotCreatorWindow::toggleAllSnaps,
                      QKeySequence(Qt::Key_F6));
  snapMenu->addAction(tr("Disable All"), this, &WbRobotCreatorWindow::clearAllSnaps, QKeySequence(Qt::Key_F7));

  QMenu *viewMenu = menuBar()->addMenu(tr("&View"));
  viewMenu->addAction(tr("&Reset Camera"), mViewport, &WbMeshViewport::resetCamera, QKeySequence(Qt::Key_Home));
  QAction *gridAction = viewMenu->addAction(tr("Show &Grid"));
  gridAction->setCheckable(true);
  gridAction->setChecked(true);
  connect(gridAction, &QAction::toggled, mViewport, &WbMeshViewport::setShowGrid);
  QAction *featureAction = viewMenu->addAction(tr("Show Snap &Features"));
  featureAction->setCheckable(true);
  featureAction->setChecked(true);
  connect(featureAction, &QAction::toggled, mViewport, &WbMeshViewport::setShowFeatures);
}

void WbRobotCreatorWindow::showMessage(const QString &message, bool isError) {
  if (mStatusLabel) {
    mStatusLabel->setText(message);
    mStatusLabel->setStyleSheet(isError ? "color: #c04040" : "");
  }
  statusBar()->showMessage(message, isError ? 0 : 4000);
}

QString WbRobotCreatorWindow::uniquePartName(const QString &base) const {
  QString name = base;
  int suffix = 1;
  while (mAssembly.indexOfPart(name.toStdString()) >= 0)
    name = base + "_" + QString::number(suffix++);
  return name;
}

void WbRobotCreatorWindow::importMesh() {
  const QString path =
    QFileDialog::getOpenFileName(this, tr("Import Mesh"), QString(), tr("Mesh files (*.stl *.obj);;All files (*)"));
  if (path.isEmpty())
    return;
  wbmesh::Mesh mesh;
  std::string error;
  if (!wbmesh::loadMeshFile(path.toStdString(), mesh, error)) {
    showMessage(tr("Mesh import failed: %1").arg(error.c_str()), true);
    return;
  }
  wbrobotproto::Part part;
  const QFileInfo info(path);
  part.name = uniquePartName(info.completeBaseName().toUpper()).toStdString();
  part.meshUrl = info.fileName().toStdString();
  part.mesh = mesh;
  // spread the imports on the grid so they are all visible
  part.translation = wbmesh::Vec3((double)(mAssembly.parts.size() % 4) * 2.0, (double)(mAssembly.parts.size() / 4) * 2.0, 0);
  mAssembly.parts.push_back(part);
  markChanged();
  mViewport->resetCamera();
  showMessage(tr("Imported %1 (%2 triangles) as link \"%3\".")
                .arg(info.fileName())
                .arg(mesh.triangleCount())
                .arg(part.name.c_str()));
}

void WbRobotCreatorWindow::addEmptyLink() {
  wbrobotproto::Part part;
  part.name = uniquePartName("LINK").toStdString();
  part.translation = wbmesh::Vec3((double)(mAssembly.parts.size() % 4) * 2.0, (double)(mAssembly.parts.size() / 4) * 2.0, 0);
  mAssembly.parts.push_back(part);
  markChanged();
  showMessage(tr("Empty link \"%1\" created.").arg(part.name.c_str()));
}

void WbRobotCreatorWindow::removeSelected() {
  QTreeWidgetItem *item = mTree->currentItem();
  if (!item)
    return;
  if (item->type() == JOINT_ITEM_TYPE) {
    const int index = item->data(0, Qt::UserRole).toInt();
    if (index >= 0 && index < (int)mAssembly.joints.size())
      mAssembly.joints.erase(mAssembly.joints.begin() + index);
  } else if (item->type() == PART_ITEM_TYPE) {
    const int index = item->data(0, Qt::UserRole).toInt();
    if (index >= 0 && index < (int)mAssembly.parts.size()) {
      // also remove the joints attached to this link
      for (size_t j = mAssembly.joints.size(); j > 0; --j) {
        const wbrobotproto::JointDef &def = mAssembly.joints[j - 1];
        if (def.parentPart == mAssembly.parts[index].name || def.childPart == mAssembly.parts[index].name)
          mAssembly.joints.erase(mAssembly.joints.begin() + (j - 1));
      }
      mAssembly.parts.erase(mAssembly.parts.begin() + index);
    }
  }
  markChanged();
}

void WbRobotCreatorWindow::startJointPlacement() {
  QAction *action = qobject_cast<QAction *>(QObject::sender());
  if (!action)
    return;
  mPendingJointType = action->data().toInt();
  mViewport->setPlacementActive(true);
  showMessage(tr("Placing a %1 joint: click a snap point for the anchor (Tab cycles candidates, Esc cancels).")
                .arg(JOINT_NAMES[mPendingJointType]));
}

void WbRobotCreatorWindow::onPlacementCancelled() {
  mPendingJointType = -1;
  mViewport->setPlacementActive(false);
  showMessage(tr("Joint placement cancelled."));
}

void WbRobotCreatorWindow::onSnapPicked(const wbsnap::SnapCandidate &candidate, int partIndex) {
  if (mPendingJointType < 0)
    return;
  wbrobotproto::JointDef def;
  def.name = "J" + std::to_string(mJointCounter++);
  def.type = jointTypeOf(mPendingJointType);
  def.anchor = candidate.position;
  def.axis = candidate.hasAxis ? candidate.axis : wbmesh::Vec3(0, 0, 1);
  def.parentPart = partIndex >= 0 ? mAssembly.parts[partIndex].name : std::string();
  // default child: the next link that is not the parent (or empty, set later)
  for (size_t i = 0; i < mAssembly.parts.size(); ++i)
    if ((int)i != partIndex) {
      def.childPart = mAssembly.parts[i].name;
      break;
    }
  mAssembly.joints.push_back(def);
  mViewport->setPlacementActive(false);
  mPendingJointType = -1;
  markChanged();
  showMessage(tr("%1 joint \"%2\" created on snap \"%3\"%4 — set its child link in the properties panel.")
                .arg(JOINT_NAMES[jointDofIndex(def.type)])
                .arg(def.name.c_str())
                .arg(candidate.label.c_str())
                .arg(candidate.hasAxis ? tr(" (axis from the snap)") : QString()));
}

void WbRobotCreatorWindow::onTreeSelectionChanged() {
  rebuildProperties();
  QTreeWidgetItem *item = mTree->currentItem();
  if (item && item->type() == PART_ITEM_TYPE)
    mViewport->setSelectedPart(item->data(0, Qt::UserRole).toInt());
  else
    mViewport->setSelectedPart(-1);
}

wbrobotproto::Part *WbRobotCreatorWindow::selectedPart() {
  QTreeWidgetItem *item = mTree->currentItem();
  if (!item || item->type() != PART_ITEM_TYPE)
    return NULL;
  const int index = item->data(0, Qt::UserRole).toInt();
  if (index < 0 || index >= (int)mAssembly.parts.size())
    return NULL;
  return &mAssembly.parts[index];
}

wbrobotproto::JointDef *WbRobotCreatorWindow::selectedJoint() {
  QTreeWidgetItem *item = mTree->currentItem();
  if (!item || item->type() != JOINT_ITEM_TYPE)
    return NULL;
  const int index = item->data(0, Qt::UserRole).toInt();
  if (index < 0 || index >= (int)mAssembly.joints.size())
    return NULL;
  return &mAssembly.joints[index];
}

void WbRobotCreatorWindow::rebuildProperties() {
  wbrobotproto::JointDef *joint = selectedJoint();
  wbrobotproto::Part *part = selectedPart();
  mJointProperties->setVisible(joint != NULL);
  mPartProperties->setVisible(part != NULL && joint == NULL);
  if (joint) {
    mJointNameEdit->setText(joint->name.c_str());
    mJointTypeCombo->setCurrentIndex(jointDofIndex(joint->type));
    mJointParentCombo->clear();
    mJointChildCombo->clear();
    for (size_t i = 0; i < mAssembly.parts.size(); ++i) {
      mJointParentCombo->addItem(mAssembly.parts[i].name.c_str());
      mJointChildCombo->addItem(mAssembly.parts[i].name.c_str());
    }
    mJointChildCombo->addItem(tr("«none (set later)»"));
    const int parentIndex = mAssembly.indexOfPart(joint->parentPart);
    const int childIndex = mAssembly.indexOfPart(joint->childPart);
    mJointParentCombo->setCurrentIndex(std::max(0, parentIndex));
    mJointChildCombo->setCurrentIndex(childIndex >= 0 ? childIndex : (int)mAssembly.parts.size());
    for (int i = 0; i < 3; ++i) {
      mAnchorSpin[i]->setValue((&joint->anchor.x)[i]);
      mAxisSpin[i]->setValue((&joint->axis.x)[i]);
    }
  } else if (part) {
    mPartNameEdit->setText(part->name.c_str());
    for (int i = 0; i < 3; ++i)
      mPartTranslationSpin[i]->setValue((&part->translation.x)[i]);
  }
}

void WbRobotCreatorWindow::applyProperties() {
  wbrobotproto::JointDef *joint = selectedJoint();
  if (joint) {
    const std::string newName = mJointNameEdit->text().toStdString();
    if (!newName.empty())
      joint->name = newName;
    joint->type = jointTypeOf(mJointTypeCombo->currentIndex());
    if (mJointParentCombo->currentIndex() >= 0)
      joint->parentPart = mJointParentCombo->currentText().toStdString();
    const int childIndex = mJointChildCombo->currentIndex();
    if (childIndex >= 0 && childIndex < (int)mAssembly.parts.size())
      joint->childPart = mAssembly.parts[childIndex].name;
    else
      joint->childPart.clear();
    for (int i = 0; i < 3; ++i) {
      (&joint->anchor.x)[i] = mAnchorSpin[i]->value();
      (&joint->axis.x)[i] = mAxisSpin[i]->value();
    }
    markChanged();
    showMessage(tr("Joint \"%1\" updated.").arg(joint->name.c_str()));
    return;
  }
  wbrobotproto::Part *part = selectedPart();
  if (part) {
    const std::string newName = mPartNameEdit->text().toStdString();
    if (!newName.empty() && mAssembly.indexOfPart(newName) < 0) {
      // keep the joints in sync with the renamed link
      const std::string oldName = part->name;
      for (size_t j = 0; j < mAssembly.joints.size(); ++j) {
        if (mAssembly.joints[j].parentPart == oldName)
          mAssembly.joints[j].parentPart = newName;
        if (mAssembly.joints[j].childPart == oldName)
          mAssembly.joints[j].childPart = newName;
      }
      part->name = newName;
    }
    for (int i = 0; i < 3; ++i)
      (&part->translation.x)[i] = mPartTranslationSpin[i]->value();
    markChanged();
    showMessage(tr("Link \"%1\" updated.").arg(part->name.c_str()));
  }
}

void WbRobotCreatorWindow::toggleAllSnaps() {
  foreach (QAction *action, mSnapActionGroup->actions())
    action->setChecked(true);
  mViewport->setSnapModes(wbsnap::SNAP_ENDPOINT | wbsnap::SNAP_MIDPOINT | wbsnap::SNAP_CIRCLE_CENTER |
                          wbsnap::SNAP_CENTROID | wbsnap::SNAP_GRID);
}

void WbRobotCreatorWindow::clearAllSnaps() {
  foreach (QAction *action, mSnapActionGroup->actions())
    action->setChecked(false);
  mViewport->setSnapModes(wbsnap::SNAP_NONE);
}

void WbRobotCreatorWindow::markChanged() {
  mViewport->rebuild();
  rebuildTree();
  updateValidation();
}

void WbRobotCreatorWindow::rebuildTree() {
  const QSignalBlocker blocker(mTree);
  mTree->clear();
  QTreeWidgetItem *partsRoot = new QTreeWidgetItem(mTree, QStringList() << tr("Links"));
  for (size_t i = 0; i < mAssembly.parts.size(); ++i) {
    const wbrobotproto::Part &part = mAssembly.parts[i];
    QTreeWidgetItem *item = new QTreeWidgetItem(partsRoot, PART_ITEM_TYPE);
    item->setText(0, part.name.c_str());
    item->setText(1, part.meshUrl.empty() ? tr("empty link") : QString("%1 tris").arg(part.mesh.triangleCount()));
    item->setData(0, Qt::UserRole, (int)i);
  }
  QTreeWidgetItem *jointsRoot = new QTreeWidgetItem(mTree, QStringList() << tr("Joints"));
  for (size_t j = 0; j < mAssembly.joints.size(); ++j) {
    const wbrobotproto::JointDef &def = mAssembly.joints[j];
    QTreeWidgetItem *item = new QTreeWidgetItem(jointsRoot, JOINT_ITEM_TYPE);
    item->setText(0, def.name.c_str());
    item->setText(1, QString("%1: %2 -> %3")
                       .arg(JOINT_NAMES[jointDofIndex(def.type)])
                       .arg(def.parentPart.c_str())
                       .arg(def.childPart.empty() ? tr("«none»") : def.childPart.c_str()));
    item->setData(0, Qt::UserRole, (int)j);
  }
  partsRoot->setExpanded(true);
  jointsRoot->setExpanded(true);
}

void WbRobotCreatorWindow::updateValidation() {
  std::vector<size_t> dangling;
  for (size_t j = 0; j < mAssembly.joints.size(); ++j) {
    const wbrobotproto::JointDef &def = mAssembly.joints[j];
    if (def.parentPart.empty() || def.childPart.empty() || def.parentPart == def.childPart ||
        mAssembly.indexOfPart(def.parentPart) < 0 || mAssembly.indexOfPart(def.childPart) < 0)
      dangling.push_back(j);
  }
  const wbmechanism::Analysis analysis = wbrobotproto::validate(mAssembly, mPlanarCheck->isChecked());
  QString text = analysis.summary().c_str();
  if (!dangling.empty())
    text += tr(" — %1 joint(s) have no child link yet").arg(dangling.size());
  const std::vector<bool> flags = wbrobotproto::spanningFlags(mAssembly);
  int loopCount = 0;
  for (size_t i = 0; i < flags.size(); ++i)
    if (!flags[i])
      ++loopCount;
  if (loopCount > 0)
    text += tr(" — %1 loop-closing joint(s) (SolidReference, closed kinematic chain)").arg(loopCount);
  mValidationLabel->setText(text);
}

void WbRobotCreatorWindow::exportRobot() {
  bool ok = false;
  const QString defaultName = tr("my_robot");
  const QString robotName =
    QInputDialog::getText(this, tr("Export Robot"), tr("Robot name:"), QLineEdit::Normal, defaultName, &ok);
  if (!ok || robotName.isEmpty())
    return;
  const QString path = QFileDialog::getSaveFileName(this, tr("Export Robot"), robotName + ".wbt", tr("Webots worlds (*.wbt)"));
  if (path.isEmpty())
    return;
  const std::string text = wbrobotproto::generateWbt(mAssembly, robotName.toStdString());
  std::ofstream file(path.toStdString().c_str());
  if (!file.is_open()) {
    showMessage(tr("Cannot write %1").arg(path), true);
    return;
  }
  file << text;
  file.close();
  showMessage(tr("Robot exported to %1").arg(path));
}

void WbRobotCreatorWindow::copyToClipboard() {
  const std::string text = wbrobotproto::generateWbt(mAssembly, "robot");
  QGuiApplication::clipboard()->setText(QString::fromStdString(text));
  showMessage(tr("Robot text copied to the clipboard (paste it into a world or a PROTO body)."));
}
