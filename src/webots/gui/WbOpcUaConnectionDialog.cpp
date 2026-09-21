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

#include "WbOpcUaConnectionDialog.hpp"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

#include "WbOpcUaManager.hpp"

WbOpcUaConnectionDialog::WbOpcUaConnectionDialog(QWidget *parent) : QDialog(parent) {
  setWindowTitle(tr("OPC-UA Connection"));
  QVBoxLayout *layout = new QVBoxLayout(this);
  QFormLayout *form = new QFormLayout();
  const wbopcua::ConnectionConfig current = WbOpcUaManager::instance()->connectionConfig();

  mEndpointEdit = new QLineEdit(QString::fromStdString(current.endpoint), this);
  mEndpointEdit->setPlaceholderText("opc.tcp://plc.example.com:4840");
  form->addRow(tr("Endpoint URL:"), mEndpointEdit);

  mSecurityCombo = new QComboBox(this);
  mSecurityCombo->addItem("None");
  mSecurityCombo->addItem("Basic256Sha256");
  mSecurityCombo->addItem("Basic128Rsa15");
  mSecurityCombo->addItem("Aes128_Sha256_RsaOaep");
  const int securityIndex = mSecurityCombo->findText(QString::fromStdString(current.securityPolicy));
  mSecurityCombo->setCurrentIndex(securityIndex < 0 ? 0 : securityIndex);
  form->addRow(tr("Security policy:"), mSecurityCombo);

  mAnonymousCheckBox = new QCheckBox(tr("anonymous"), this);
  mAnonymousCheckBox->setChecked(current.username.empty());
  connect(mAnonymousCheckBox, &QCheckBox::toggled, this, &WbOpcUaConnectionDialog::toggleAnonymous);
  form->addRow(tr("Authentication:"), mAnonymousCheckBox);

  mUserEdit = new QLineEdit(QString::fromStdString(current.username), this);
  form->addRow(tr("User:"), mUserEdit);
  mPasswordEdit = new QLineEdit(QString::fromStdString(current.password), this);
  mPasswordEdit->setEchoMode(QLineEdit::Password);
  form->addRow(tr("Password:"), mPasswordEdit);
  toggleAnonymous(mAnonymousCheckBox->isChecked());

  mExposeServerCheckBox = new QCheckBox(tr("run the embedded Webots OPC-UA server"), this);
  form->addRow(tr("Server:"), mExposeServerCheckBox);
  mPortSpinBox = new QSpinBox(this);
  mPortSpinBox->setRange(1, 65535);
  mPortSpinBox->setValue(current.serverPort);
  form->addRow(tr("Server port:"), mPortSpinBox);

  layout->addLayout(form);
  mStatusLabel = new QLabel(this);
  mStatusLabel->setWordWrap(true);
  layout->addWidget(mStatusLabel);

  QHBoxLayout *buttonsLayout = new QHBoxLayout();
  QPushButton *testButton = new QPushButton(tr("Test connection"), this);
  connect(testButton, &QPushButton::clicked, this, &WbOpcUaConnectionDialog::testConnection);
  buttonsLayout->addWidget(testButton);
  buttonsLayout->addStretch();
  QDialogButtonBox *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
  connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
  buttonsLayout->addWidget(buttons);
  layout->addLayout(buttonsLayout);

  if (!WbOpcUaManager::hasNativeStack())
    mStatusLabel->setText(tr("Note: built without open62541, the in-memory mock backend will be used."));
}

void WbOpcUaConnectionDialog::toggleAnonymous(bool anonymous) {
  mUserEdit->setEnabled(!anonymous);
  mPasswordEdit->setEnabled(!anonymous);
}

wbopcua::ConnectionConfig WbOpcUaConnectionDialog::connectionConfig() const {
  wbopcua::ConnectionConfig config;
  config.endpoint = mEndpointEdit->text().toStdString();
  config.securityPolicy = mSecurityCombo->currentText().toStdString();
  if (!mAnonymousCheckBox->isChecked()) {
    config.username = mUserEdit->text().toStdString();
    config.password = mPasswordEdit->text().toStdString();
  }
  config.serverPort = mPortSpinBox->value();
  return config;
}

bool WbOpcUaConnectionDialog::exposeServer() const {
  return mExposeServerCheckBox->isChecked();
}

void WbOpcUaConnectionDialog::testConnection() {
  WbOpcUaManager *manager = WbOpcUaManager::instance();
  const bool wasConnected = manager->isConnected();
  manager->setConnectionConfig(connectionConfig());
  QString error;
  if (manager->connectToServer(false, error)) {
    mStatusLabel->setText(tr("Connection OK (%1 backend).").arg(manager->backendName()));
    if (!wasConnected)
      manager->disconnectFromServer();
  } else
    mStatusLabel->setText(tr("Connection failed: %1").arg(error));
}
