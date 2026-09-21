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

#include "WbOpcUaBrowserModel.hpp"

#include "WbOpcUaManager.hpp"

using namespace wbopcua;

WbOpcUaBrowserModel::WbOpcUaBrowserModel(QObject *parent) : QAbstractItemModel(parent), mRoot(new Item()) {
  reload();
}

WbOpcUaBrowserModel::~WbOpcUaBrowserModel() {
  clearItems(mRoot);
  delete mRoot;
}

void WbOpcUaBrowserModel::clearItems(Item *item) const {
  for (int i = 0; i < item->children.size(); ++i)
    clearItems(item->children[i]);
  qDeleteAll(item->children);
  item->children.clear();
  item->childrenLoaded = false;
}

void WbOpcUaBrowserModel::reload() {
  beginResetModel();
  clearItems(mRoot);
  mError.clear();
  WbOpcUaManager *manager = WbOpcUaManager::instance();
  if (manager->isConnected())
    loadChildren(mRoot);
  else
    mError = tr("Not connected to any OPC UA endpoint.");
  endResetModel();
}

void WbOpcUaBrowserModel::loadChildren(Item *item) const {
  if (item->childrenLoaded)
    return;
  item->childrenLoaded = true;
  QString error;
  const std::vector<BrowseEntry> entries =
    WbOpcUaManager::instance()->browse(QString::fromStdString(item->entry.nodeId), error);
  if (!error.isEmpty()) {
    const_cast<WbOpcUaBrowserModel *>(this)->mError = error;
    return;
  }
  for (size_t i = 0; i < entries.size(); ++i) {
    Item *child = new Item(item);
    child->entry = entries[i];
    // the embedded Webots server keeps its variables at a deterministic root
    item->children.append(child);
  }
}

WbOpcUaBrowserModel::Item *WbOpcUaBrowserModel::itemFromIndex(const QModelIndex &index) const {
  return index.isValid() ? static_cast<Item *>(index.internalPointer()) : mRoot;
}

QModelIndex WbOpcUaBrowserModel::index(int row, int column, const QModelIndex &parent) const {
  if (!hasIndex(row, column, parent))
    return QModelIndex();
  Item *parentItem = itemFromIndex(parent);
  if (!parentItem->childrenLoaded)
    loadChildren(parentItem);
  if (row < 0 || row >= parentItem->children.size())
    return QModelIndex();
  return createIndex(row, column, parentItem->children[row]);
}

QModelIndex WbOpcUaBrowserModel::parent(const QModelIndex &child) const {
  if (!child.isValid())
    return QModelIndex();
  Item *item = itemFromIndex(child);
  Item *parentItem = item->parent;
  if (!parentItem || parentItem == mRoot)
    return QModelIndex();
  Item *grandParent = parentItem->parent;
  const int row = grandParent ? grandParent->children.indexOf(parentItem) : 0;
  return createIndex(row, 0, parentItem);
}

int WbOpcUaBrowserModel::rowCount(const QModelIndex &parent) const {
  Item *item = itemFromIndex(parent);
  if (!item->childrenLoaded)
    loadChildren(item);
  return item->children.size();
}

int WbOpcUaBrowserModel::columnCount(const QModelIndex &parent) const {
  Q_UNUSED(parent);
  return COLUMN_COUNT;
}

QVariant WbOpcUaBrowserModel::data(const QModelIndex &index, int role) const {
  if (!index.isValid() || (role != Qt::DisplayRole && role != Qt::ToolTipRole))
    return QVariant();
  Item *item = itemFromIndex(index);
  switch (index.column()) {
    case NAME:
      return QString::fromStdString(item->entry.displayName);
    case NODE_ID:
      return QString::fromStdString(item->entry.nodeId);
    case DATA_TYPE:
      return QString::fromStdString(item->entry.dataType);
    case VALUE:
      return QString::fromStdString(item->entry.value.toString());
    case DESCRIPTION:
      return QString::fromStdString(item->entry.description);
  }
  return QVariant();
}

QVariant WbOpcUaBrowserModel::headerData(int section, Qt::Orientation orientation, int role) const {
  if (orientation != Qt::Horizontal || role != Qt::DisplayRole)
    return QVariant();
  switch (section) {
    case NAME:
      return tr("Name");
    case NODE_ID:
      return tr("Node id");
    case DATA_TYPE:
      return tr("Type");
    case VALUE:
      return tr("Value");
    case DESCRIPTION:
      return tr("Description");
  }
  return QVariant();
}

QString WbOpcUaBrowserModel::nodeId(const QModelIndex &index) const {
  return index.isValid() ? QString::fromStdString(itemFromIndex(index)->entry.nodeId) : QString();
}

Value::Type WbOpcUaBrowserModel::dataType(const QModelIndex &index) const {
  return index.isValid() ? Value::typeFromName(itemFromIndex(index)->entry.dataType) : Value::NULL_VALUE;
}

QString WbOpcUaBrowserModel::browseName(const QModelIndex &index) const {
  return index.isValid() ? QString::fromStdString(itemFromIndex(index)->entry.browseName) : QString();
}
