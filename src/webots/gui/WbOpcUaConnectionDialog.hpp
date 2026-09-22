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

// Description: OPC-UA connection dialog (endpoint URL, security policy, user
//              credentials, embedded server options).

#ifndef WB_OPC_UA_CONNECTION_DIALOG_HPP
#define WB_OPC_UA_CONNECTION_DIALOG_HPP

#include <QtWidgets/QDialog>

#include "WbOpcUaBackend.hpp"

class QCheckBox;
class QComboBox;
class QLabel;
class QLineEdit;
class QSpinBox;

class WbOpcUaConnectionDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbOpcUaConnectionDialog(QWidget *parent = NULL);
  virtual ~WbOpcUaConnectionDialog() {}

  wbopcua::ConnectionConfig connectionConfig() const;
  bool exposeServer() const;

private slots:
  void testConnection();
  void toggleAnonymous(bool anonymous);

private:
  QLineEdit *mEndpointEdit;
  QComboBox *mSecurityCombo;
  QCheckBox *mAnonymousCheckBox;
  QLineEdit *mUserEdit;
  QLineEdit *mPasswordEdit;
  QCheckBox *mExposeServerCheckBox;
  QSpinBox *mPortSpinBox;
  QLabel *mStatusLabel;
};

#endif
