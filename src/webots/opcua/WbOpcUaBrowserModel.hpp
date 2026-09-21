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

// Description: lazy Qt tree model over the remote OPC UA address space, used by
//              the variable chooser dialog.

#ifndef WB_OPC_UA_BROWSER_MODEL_HPP
#define WB_OPC_UA_BROWSER_MODEL_HPP

#include <QtCore/QAbstractItemModel>
#include <QtCore/QVector>

#include "WbOpcUaBackend.hpp"

class WbOpcUaBrowserModel : public QAbstractItemModel {
  Q_OBJECT

public:
  enum Columns { NAME = 0, NODE_ID, DATA_TYPE, VALUE, DESCRIPTION, COLUMN_COUNT };

  explicit WbOpcUaBrowserModel(QObject *parent = NULL);
  virtual ~WbOpcUaBrowserModel();

  // reload the tree from the root node
  void reload();
  bool hasError() const { return !mError.isEmpty(); }
  const QString &errorString() const { return mError; }

  // QModelIndex API
  QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
  QModelIndex parent(const QModelIndex &child) const override;
  int rowCount(const QModelIndex &parent = QModelIndex()) const override;
  int columnCount(const QModelIndex &parent = QModelIndex()) const override;
  QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

  // node id of an index ("" for the invisible root)
  QString nodeId(const QModelIndex &index) const;
  wbopcua::Value::Type dataType(const QModelIndex &index) const;
  QString browseName(const QModelIndex &index) const;

private:
  struct Item {
    wbopcua::BrowseEntry entry;
    Item *parent;
    QVector<Item *> children;
    bool childrenLoaded;
    explicit Item(Item *parentItem = NULL) : parent(parentItem), childrenLoaded(false) {}
  };

  Item *itemFromIndex(const QModelIndex &index) const;
  void loadChildren(Item *item) const;
  void clearItems(Item *item) const;

  mutable Item *mRoot;  // invisible root
  QString mError;
};

#endif
