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

// Description: native OPC-UA manager: owns the client/server backend, the
//              variable mapping store and the synchronization loop between the
//              simulation and OPC UA variables.

#ifndef WB_OPC_UA_MANAGER_HPP
#define WB_OPC_UA_MANAGER_HPP

#include <QtCore/QHash>
#include <QtCore/QObject>
#include <QtCore/QString>

#include "WbOpcUaBackend.hpp"
#include "WbOpcUaMapping.hpp"

class WbOpcUaTarget;
class QTimer;

class WbOpcUaManager : public QObject {
  Q_OBJECT

public:
  static WbOpcUaManager *instance();
  static void cleanup();

  // ---- connection
  void setConnectionConfig(const wbopcua::ConnectionConfig &config);
  const wbopcua::ConnectionConfig &connectionConfig() const { return mConfig; }
  // opens the remote client session and, when `exposeServer`, the embedded server
  bool connectToServer(bool exposeServer, QString &error);
  void disconnectFromServer();
  bool isConnected() const;
  bool isServerRunning() const;
  QString backendName() const;
  QString endpointDescription() const;
  // true when Webots was built with the open62541 stack (false: in-memory mock)
  static bool hasNativeStack();

  // ---- remote address space (variable chooser)
  std::vector<wbopcua::BrowseEntry> browse(const QString &nodeId, QString &error);
  bool readVariable(const QString &nodeId, wbopcua::Value &value, QString &error);

  // ---- mappings
  wbopcua::MappingStore &mappingStore() { return mMappingStore; }
  const wbopcua::MappingStore &mappingStore() const { return mMappingStore; }
  QString mappingFilePath() const;
  bool loadMappings(QString &error);
  bool saveMappings(QString &error) const;
  // bind a Webots target to an OPC UA variable and expose it on the embedded server
  QString addMapping(const QString &webotsTarget, const QString &nodeId, const QString &browseName,
                     wbopcua::Direction direction, wbopcua::Value::Type dataType);
  bool removeMapping(const QString &id);
  // live cached values of the mappings (OPC UA units)
  wbopcua::Value mappingValue(const QString &id) const;
  QString mappingQuality(const QString &id) const;  // "good", "bad: <error>", "-" when idle

signals:
  void connectionStateChanged();
  void mappingValueChanged(const QString &id, const QString &displayValue);
  void mappingStoreChanged();
  void errorOccurred(const QString &message);

private slots:
  void synchronize();

private:
  explicit WbOpcUaManager(QObject *parent = NULL);
  virtual ~WbOpcUaManager();
  static WbOpcUaManager *cInstance;

  void ensureBackend();
  void refreshMappingValue(const QString &id, const wbopcua::Value &value, const QString &quality);
  void updateServerExposure();

  wbopcua::BackendPtr mBackend;
  wbopcua::ConnectionConfig mConfig;
  wbopcua::MappingStore mMappingStore;
  QHash<QString, wbopcua::Value> mLastOpcValues;    // last OPC value per mapping id
  QHash<QString, double> mLastWebotsValues;         // last Webots value per mapping id
  QHash<QString, QString> mQuality;                 // quality string per mapping id
  QTimer *mSyncTimer;
  bool mExposeServer;
};

#endif
