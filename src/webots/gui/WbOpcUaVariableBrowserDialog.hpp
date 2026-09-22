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

// Description: OPC-UA variable chooser dialog: browse the remote address space,
//              search variables and bind them to Webots targets.

#ifndef WB_OPC_UA_VARIABLE_BROWSER_DIALOG_HPP
#define WB_OPC_UA_VARIABLE_BROWSER_DIALOG_HPP

#include <QtWidgets/QDialog>

class WbOpcUaBrowserModel;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QSortFilterProxyModel;
class QTreeView;

class WbOpcUaVariableBrowserDialog : public QDialog {
  Q_OBJECT

public:
  explicit WbOpcUaVariableBrowserDialog(QWidget *parent = NULL);
  virtual ~WbOpcUaVariableBrowserDialog() {}

  // default Webots target suggested for new mappings (e.g. from the current selection)
  void setDefaultTarget(const QString &target);

private slots:
  void onSelectionChanged();
  void bindSelected();
  void refresh();

private:
  QString currentTarget() const;

  WbOpcUaBrowserModel *mModel;
  QSortFilterProxyModel *mProxy;
  QTreeView *mTreeView;
  QLineEdit *mSearchEdit;
  QLineEdit *mTargetEdit;
  QComboBox *mDirectionCombo;
  QDoubleSpinBox *mScaleSpin;
  QDoubleSpinBox *mOffsetSpin;
  QDoubleSpinBox *mDeadbandSpin;
};

#endif
