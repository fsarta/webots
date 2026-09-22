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

// Description: OPC-UA I/O dock: connection status, mapping table with live
//              values and access to the variable chooser.

#ifndef WB_OPC_UA_WINDOW_HPP
#define WB_OPC_UA_WINDOW_HPP

#include "WbDockWidget.hpp"

class QComboBox;
class QLabel;
class QPushButton;
class QTableWidget;

class WbOpcUaWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit WbOpcUaWindow(QWidget *parent = NULL);
  virtual ~WbOpcUaWindow();

  void refresh();

private slots:
  void openConnectionDialog();
  void openVariableBrowser();
  void disconnectFromServer();
  void removeSelectedMapping();
  void loadMappings();
  void saveMappings();
  void updateConnectionState();
  void onMappingStoreChanged();
  void onMappingValueChanged(const QString &id, const QString &displayValue);

private:
  void showError(const QString &message);

  QLabel *mStatusLabel;
  QTableWidget *mTable;
  QPushButton *mConnectButton;
  QPushButton *mDisconnectButton;
  QPushButton *mBrowseButton;
  QPushButton *mRemoveButton;
};

#endif
