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

#include "WbOpcUaDock.hpp"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>

#include "WbOpcUaConnectionDialog.hpp"
#include "WbOpcUaManager.hpp"
#include "WbOpcUaVariableBrowserDialog.hpp"

static const int ID_ROLE = Qt::UserRole;

WbOpcUaDock::WbOpcUaDock(QWidget *parent) : WbDockWidget(parent) {
  setWindowTitle(tr("OPC-UA I/O"));
  setObjectName("OpcUaWidget");  // for dock perspective

  QWidget *container = new QWidget(this);
  QVBoxLayout *layout = new QVBoxLayout(container);
  layout->setContentsMargins(4, 4, 4, 4);

  mStatusLabel = new QLabel(tr("Disconnected."), container);
  mStatusLabel->setWordWrap(true);
  layout->addWidget(mStatusLabel);

  QHBoxLayout *buttons = new QHBoxLayout();
  mConnectButton = new QPushButton(tr("Connect..."), container);
  connect(mConnectButton, &QPushButton::clicked, this, &WbOpcUaDock::openConnectionDialog);
  buttons->addWidget(mConnectButton);
  mDisconnectButton = new QPushButton(tr("Disconnect"), container);
  connect(mDisconnectButton, &QPushButton::clicked, this, &WbOpcUaDock::disconnectFromServer);
  buttons->addWidget(mDisconnectButton);
  mBrowseButton = new QPushButton(tr("Browse variables..."), container);
  connect(mBrowseButton, &QPushButton::clicked, this, &WbOpcUaDock::openVariableBrowser);
  buttons->addWidget(mBrowseButton);
  layout->addLayout(buttons);

  mTable = new QTableWidget(container);
  mTable->setColumnCount(7);
  QStringList headers;
  headers << tr("id") << tr("Webots target") << tr("OPC UA node id") << tr("direction") << tr("value") << tr("quality")
          << tr("enabled");
  mTable->setHorizontalHeaderLabels(headers);
  mTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
  mTable->horizontalHeader()->setStretchLastSection(true);
  mTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  mTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  layout->addWidget(mTable, 1);

  QHBoxLayout *bottomButtons = new QHBoxLayout();
  mRemoveButton = new QPushButton(tr("Remove mapping"), container);
  connect(mRemoveButton, &QPushButton::clicked, this, &WbOpcUaDock::removeSelectedMapping);
  bottomButtons->addWidget(mRemoveButton);
  QPushButton *loadButton = new QPushButton(tr("Load"), container);
  connect(loadButton, &QPushButton::clicked, this, &WbOpcUaDock::loadMappings);
  bottomButtons->addWidget(loadButton);
  QPushButton *saveButton = new QPushButton(tr("Save"), container);
  connect(saveButton, &QPushButton::clicked, this, &WbOpcUaDock::saveMappings);
  bottomButtons->addWidget(saveButton);
  bottomButtons->addStretch();
  layout->addLayout(bottomButtons);
  setWidget(container);

  WbOpcUaManager *manager = WbOpcUaManager::instance();
  connect(manager, &WbOpcUaManager::connectionStateChanged, this, &WbOpcUaDock::updateConnectionState);
  connect(manager, &WbOpcUaManager::mappingStoreChanged, this, &WbOpcUaDock::onMappingStoreChanged);
  connect(manager, &WbOpcUaManager::mappingValueChanged, this, &WbOpcUaDock::onMappingValueChanged);
  connect(manager, &WbOpcUaManager::errorOccurred, this, &WbOpcUaDock::showError);

  refresh();
}

WbOpcUaDock::~WbOpcUaDock() {
}

void WbOpcUaDock::refresh() {
  updateConnectionState();
  onMappingStoreChanged();
}

void WbOpcUaDock::openConnectionDialog() {
  WbOpcUaConnectionDialog dialog(this);
  if (dialog.exec() != QDialog::Accepted)
    return;
  WbOpcUaManager *manager = WbOpcUaManager::instance();
  manager->setConnectionConfig(dialog.connectionConfig());
  QString error;
  if (!manager->connectToServer(dialog.exposeServer(), error))
    showError(tr("OPC-UA connection failed: %1").arg(error));
  else
    manager->loadMappings(error);  // auto-load the world's mapping file when present
}

void WbOpcUaDock::openVariableBrowser() {
  WbOpcUaManager *manager = WbOpcUaManager::instance();
  if (!manager->isConnected()) {
    showError(tr("Connect to an OPC-UA endpoint first."));
    return;
  }
  WbOpcUaVariableBrowserDialog dialog(this);
  dialog.setDefaultTarget(mTable->currentRow() >= 0 ? mTable->item(mTable->currentRow(), 1)->text() : QString());
  dialog.exec();
}

void WbOpcUaDock::disconnectFromServer() {
  WbOpcUaManager::instance()->disconnectFromServer();
}

void WbOpcUaDock::removeSelectedMapping() {
  const int row = mTable->currentRow();
  if (row < 0)
    return;
  WbOpcUaManager::instance()->removeMapping(mTable->item(row, 0)->data(ID_ROLE).toString());
}

void WbOpcUaDock::loadMappings() {
  QString error;
  if (!WbOpcUaManager::instance()->loadMappings(error))
    showError(error);
}

void WbOpcUaDock::saveMappings() {
  QString error;
  if (!WbOpcUaManager::instance()->saveMappings(error))
    showError(error);
  else
    mStatusLabel->setText(tr("Mappings saved to %1").arg(WbOpcUaManager::instance()->mappingFilePath()));
}

void WbOpcUaDock::updateConnectionState() {
  WbOpcUaManager *manager = WbOpcUaManager::instance();
  if (manager->isConnected()) {
    QString text = tr("Connected to %1 (%2 backend)")
                     .arg(manager->endpointDescription(), manager->backendName());
    if (manager->isServerRunning())
      text += tr(" — embedded server running");
    mStatusLabel->setText(text);
  } else
    mStatusLabel->setText(tr("Disconnected."));
  mConnectButton->setEnabled(!manager->isConnected());
  mDisconnectButton->setEnabled(manager->isConnected());
  mBrowseButton->setEnabled(manager->isConnected());
}

void WbOpcUaDock::onMappingStoreChanged() {
  const std::vector<wbopcua::Mapping> &mappings = WbOpcUaManager::instance()->mappingStore().mappings();
  mTable->setRowCount((int)mappings.size());
  for (int i = 0; i < (int)mappings.size(); ++i) {
    const wbopcua::Mapping &m = mappings[i];
    const QString id = QString::fromStdString(m.id);
    QTableWidgetItem *idItem = new QTableWidgetItem(id);
    idItem->setData(ID_ROLE, id);
    mTable->setItem(i, 0, idItem);
    mTable->setItem(i, 1, new QTableWidgetItem(QString::fromStdString(m.webotsTarget)));
    mTable->setItem(i, 2, new QTableWidgetItem(QString::fromStdString(m.nodeId)));
    mTable->setItem(i, 3, new QTableWidgetItem(QString::fromStdString(wbopcua::directionName(m.direction))));
    mTable->setItem(i, 4, new QTableWidgetItem(QString::fromStdString(
                        WbOpcUaManager::instance()->mappingValue(id).toString())));
    mTable->setItem(i, 5, new QTableWidgetItem(WbOpcUaManager::instance()->mappingQuality(id)));
    QTableWidgetItem *enabledItem = new QTableWidgetItem(m.enabled ? tr("yes") : tr("no"));
    mTable->setItem(i, 6, enabledItem);
  }
}

void WbOpcUaDock::onMappingValueChanged(const QString &id, const QString &displayValue) {
  for (int i = 0; i < mTable->rowCount(); ++i) {
    if (mTable->item(i, 0)->data(ID_ROLE).toString() == id) {
      mTable->item(i, 4)->setText(displayValue);
      mTable->item(i, 5)->setText(WbOpcUaManager::instance()->mappingQuality(id));
      break;
    }
  }
}

void WbOpcUaDock::showError(const QString &message) {
  mStatusLabel->setText("<span style='color:#b00000'>" + message + "</span>");
}
