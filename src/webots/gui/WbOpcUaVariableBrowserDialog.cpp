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

#include "WbOpcUaVariableBrowserDialog.hpp"

#include <QtCore/QSortFilterProxyModel>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>

#include "WbOpcUaBrowserModel.hpp"
#include "WbOpcUaManager.hpp"
#include "WbOpcUaMapping.hpp"

WbOpcUaVariableBrowserDialog::WbOpcUaVariableBrowserDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("OPC-UA Variables"));
  resize(860, 520);
  QVBoxLayout *layout = new QVBoxLayout(this);

  mSearchEdit = new QLineEdit(this);
  mSearchEdit->setPlaceholderText(tr("Search variables by name..."));
  connect(mSearchEdit, &QLineEdit::textChanged, this, [this](const QString &text) {
    mProxy->setFilterFixedString(text);
  });
  layout->addWidget(mSearchEdit);

  mModel = new WbOpcUaBrowserModel(this);
  mProxy = new QSortFilterProxyModel(this);
  mProxy->setSourceModel(mModel);
  mProxy->setFilterKeyColumn(WbOpcUaBrowserModel::NAME);
  mProxy->setFilterCaseSensitivity(Qt::CaseInsensitive);
  // variables only: hide pure objects when searching
  mTreeView = new QTreeView(this);
  mTreeView->setModel(mProxy);
  mTreeView->setSelectionMode(QAbstractItemView::ExtendedSelection);
  mTreeView->setUniformRowHeights(true);
  mTreeView->setAlternatingRowColors(true);
  mTreeView->header()->setSectionResizeMode(QHeaderView::ResizeToContents);
  connect(mTreeView->selectionModel(), &QItemSelectionModel::selectionChanged, this,
          &WbOpcUaVariableBrowserDialog::onSelectionChanged);
  layout->addWidget(mTreeView, 1);

  QFormLayout *form = new QFormLayout();
  mTargetEdit = new QLineEdit(this);
  mTargetEdit->setPlaceholderText(tr("device:MyRobot/motor1/targetPosition"));
  form->addRow(tr("Bind to:"), mTargetEdit);
  mDirectionCombo = new QComboBox(this);
  mDirectionCombo->addItem(tr("OPC -> Webots (read)"), 0);
  mDirectionCombo->addItem(tr("Webots -> OPC (write)"), 1);
  mDirectionCombo->addItem(tr("Both (readWrite)"), 2);
  form->addRow(tr("Direction:"), mDirectionCombo);
  QHBoxLayout *scaling = new QHBoxLayout();
  mScaleSpin = new QDoubleSpinBox(this);
  mScaleSpin->setRange(-1e6, 1e6);
  mScaleSpin->setValue(1.0);
  scaling->addWidget(new QLabel(tr("scale")));
  scaling->addWidget(mScaleSpin);
  mOffsetSpin = new QDoubleSpinBox(this);
  mOffsetSpin->setRange(-1e6, 1e6);
  scaling->addWidget(new QLabel(tr("offset")));
  scaling->addWidget(mOffsetSpin);
  mDeadbandSpin = new QDoubleSpinBox(this);
  mDeadbandSpin->setRange(0.0, 1e6);
  scaling->addWidget(new QLabel(tr("deadband")));
  scaling->addWidget(mDeadbandSpin);
  form->addRow(tr("Conversion:"), scaling);
  layout->addLayout(form);

  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  QPushButton *refreshButton = new QPushButton(tr("Refresh"), this);
  connect(refreshButton, &QPushButton::clicked, this, &WbOpcUaVariableBrowserDialog::refresh);
  buttonsLayout->addWidget(refreshButton);
  QPushButton *bindButton = new QPushButton(tr("Bind selected variables"), this);
  connect(bindButton, &QPushButton::clicked, this, &WbOpcUaVariableBrowserDialog::bindSelected);
  buttonsLayout->addWidget(bindButton);
  buttonsLayout->addStretch();
  QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  buttonsLayout->addWidget(buttons);
  layout->addLayout(buttonsLayout);

  if (mModel->hasError())
    layout->addWidget(new QLabel(tr("Error: %1").arg(mModel->errorString()), this));
}

void WbOpcUaVariableBrowserDialog::setDefaultTarget(const QString &target) {
  mTargetEdit->setText(target);
}

QString WbOpcUaVariableBrowserDialog::currentTarget() const {
  return mTargetEdit->text().trimmed();
}

void WbOpcUaVariableBrowserDialog::onSelectionChanged() {
  // convenience: suggest one binding per selected variable, using its browse name
  const QModelIndexList rows = mTreeView->selectionModel()->selectedRows();
  if (rows.size() == 1 && mTargetEdit->text().isEmpty())
    mTargetEdit->setPlaceholderText(tr("device:MyRobot/%1/position")
                                     .arg(mModel->browseName(mProxy->mapToSource(rows.first()))));
}

void WbOpcUaVariableBrowserDialog::refresh() {
  mModel->reload();
}

void WbOpcUaVariableBrowserDialog::bindSelected() {
  const QModelIndexList rows = mTreeView->selectionModel()->selectedRows();
  if (rows.isEmpty())
    return;
  const QString target = currentTarget();
  const wbopcua::Direction direction = (wbopcua::Direction)mDirectionCombo->currentData().toInt();
  WbOpcUaManager *manager = WbOpcUaManager::instance();
  foreach (const QModelIndex &proxyIndex, rows) {
    const QModelIndex index = mProxy->mapToSource(proxyIndex);
    wbopcua::Value::Type type = mModel->dataType(index);
    if (type == wbopcua::Value::NULL_VALUE)
      type = wbopcua::Value::DOUBLE;
    // with multiple selections each variable keeps its own target placeholder
    QString bindTarget = target;
    if (rows.size() > 1)
      bindTarget = target.isEmpty() ? mModel->browseName(index) : target + "/" + mModel->browseName(index);
    const QString id = manager->addMapping(bindTarget, mModel->nodeId(index), mModel->browseName(index), direction, type);
    wbopcua::Mapping *mapping = manager->mappingStore().find(id.toStdString());
    if (mapping) {
      mapping->scale = mScaleSpin->value();
      mapping->offset = mOffsetSpin->value();
      mapping->deadband = mDeadbandSpin->value();
    }
  }
  accept();
}
