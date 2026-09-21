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

#include "WbOpcUaManager.hpp"

#include <cmath>

#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QTextStream>
#include <QtCore/QTimer>

#include "WbOpcUaMockBackend.hpp"
#include "WbOpcUaTarget.hpp"
#include "WbWorld.hpp"

#ifdef WB_USE_OPEN62541
#  include "WbOpcUaOpen62541Backend.hpp"
#endif

using namespace wbopcua;

WbOpcUaManager *WbOpcUaManager::cInstance = NULL;

WbOpcUaManager *WbOpcUaManager::instance() {
  if (!cInstance)
    cInstance = new WbOpcUaManager();
  return cInstance;
}

void WbOpcUaManager::cleanup() {
  delete cInstance;
  cInstance = NULL;
}

WbOpcUaManager::WbOpcUaManager(QObject *parent) :
  QObject(parent), mBackend(NULL), mSyncTimer(new QTimer(this)), mExposeServer(false) {
  mConfig.endpoint = "opc.tcp://localhost:4840";
  connect(mSyncTimer, &QTimer::timeout, this, &WbOpcUaManager::synchronize);
}

WbOpcUaManager::~WbOpcUaManager() {
  if (mBackend)
    mBackend->disconnect();
}

void WbOpcUaManager::ensureBackend() {
  if (mBackend)
    return;
#ifdef WB_USE_OPEN62541
  mBackend = std::make_shared<WbOpcUaOpen62541Backend>();
#else
  mBackend = std::make_shared<WbOpcUaMockBackend>();
#endif
}

bool WbOpcUaManager::hasNativeStack() {
#ifdef WB_USE_OPEN62541
  return true;
#else
  return false;
#endif
}

void WbOpcUaManager::setConnectionConfig(const ConnectionConfig &config) {
  mConfig = config;
}

bool WbOpcUaManager::connectToServer(bool exposeServer, QString &error) {
  ensureBackend();
  std::string stdError;
  mExposeServer = exposeServer;
  if (!mBackend->connect(mConfig, exposeServer, stdError)) {
    error = QString::fromStdString(stdError);
    emit errorOccurred(error);
    emit connectionStateChanged();
    return false;
  }
  updateServerExposure();
  mSyncTimer->start(50);  // 20 Hz synchronization loop
  emit connectionStateChanged();
  return true;
}

void WbOpcUaManager::disconnectFromServer() {
  mSyncTimer->stop();
  if (mBackend)
    mBackend->disconnect();
  mLastOpcValues.clear();
  mLastWebotsValues.clear();
  mQuality.clear();
  emit connectionStateChanged();
}

bool WbOpcUaManager::isConnected() const {
  return mBackend && mBackend->isConnected();
}

bool WbOpcUaManager::isServerRunning() const {
  return mBackend && mBackend->isServerRunning();
}

QString WbOpcUaManager::backendName() const {
  return mBackend ? QString::fromStdString(mBackend->backendName()) : QString("-");
}

QString WbOpcUaManager::endpointDescription() const {
  return mBackend ? QString::fromStdString(mBackend->endpointDescription()) : QString();
}

std::vector<BrowseEntry> WbOpcUaManager::browse(const QString &nodeId, QString &error) {
  std::vector<BrowseEntry> result;
  if (!isConnected()) {
    error = tr("Not connected to any OPC UA endpoint.");
    return result;
  }
  std::string stdError;
  result = mBackend->browse(nodeId.toStdString(), stdError);
  error = QString::fromStdString(stdError);
  return result;
}

bool WbOpcUaManager::readVariable(const QString &nodeId, Value &value, QString &error) {
  if (!isConnected()) {
    error = tr("Not connected to any OPC UA endpoint.");
    return false;
  }
  std::string stdError;
  const bool ok = mBackend->read(nodeId.toStdString(), value, stdError);
  error = QString::fromStdString(stdError);
  return ok;
}

QString WbOpcUaManager::mappingFilePath() const {
  WbWorld *world = WbWorld::instance();
  if (!world || world->fileName().isEmpty())
    return QString();
  const QFileInfo info(world->fileName());
  return info.absolutePath() + "/" + info.completeBaseName() + ".opcua.json";
}

bool WbOpcUaManager::loadMappings(QString &error) {
  const QString path = mappingFilePath();
  if (path.isEmpty()) {
    error = tr("Save the world first: the mapping file is stored next to it.");
    return false;
  }
  QFile file(path);
  if (!file.exists()) {
    mMappingStore.clear();
    emit mappingStoreChanged();
    return true;
  }
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    error = tr("Cannot read '%1'").arg(path);
    return false;
  }
  const QByteArray content = file.readAll();
  std::string stdError;
  if (!mMappingStore.fromJsonString(std::string(content.constData(), content.size()), stdError)) {
    error = QString::fromStdString(stdError);
    return false;
  }
  emit mappingStoreChanged();
  return true;
}

bool WbOpcUaManager::saveMappings(QString &error) const {
  const QString path = mappingFilePath();
  if (path.isEmpty()) {
    error = tr("Save the world first: the mapping file is stored next to it.");
    return false;
  }
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = tr("Cannot write '%1'").arg(path);
    return false;
  }
  const std::string content = mMappingStore.toJsonString();
  file.write(content.data(), (qint64)content.size());
  return true;
}

QString WbOpcUaManager::addMapping(const QString &webotsTarget, const QString &nodeId, const QString &browseName,
                                   Direction direction, Value::Type dataType) {
  Mapping mapping;
  mapping.webotsTarget = webotsTarget.toStdString();
  mapping.nodeId = nodeId.toStdString();
  mapping.browseName = browseName.toStdString();
  mapping.direction = direction;
  mapping.dataType = dataType;
  const std::string id = mMappingStore.add(mapping);
  mLastWebotsValues[QString::fromStdString(id)] = qQNaN();
  mLastOpcValues[QString::fromStdString(id)] = Value();
  mQuality[QString::fromStdString(id)] = "-";
  updateServerExposure();
  emit mappingStoreChanged();
  return QString::fromStdString(id);
}

bool WbOpcUaManager::removeMapping(const QString &id) {
  const bool ok = mMappingStore.remove(id.toStdString());
  if (ok) {
    mLastOpcValues.remove(id);
    mLastWebotsValues.remove(id);
    mQuality.remove(id);
    updateServerExposure();
    emit mappingStoreChanged();
  }
  return ok;
}

Value WbOpcUaManager::mappingValue(const QString &id) const {
  return mLastOpcValues.value(id);
}

QString WbOpcUaManager::mappingQuality(const QString &id) const {
  return mQuality.value(id, "-");
}

void WbOpcUaManager::updateServerExposure() {
  if (!mBackend || !mBackend->isServerRunning())
    return;
  std::string error;
  const std::vector<Mapping> &mappings = mMappingStore.mappings();
  for (size_t i = 0; i < mappings.size(); ++i) {
    const Mapping &m = mappings[i];
    // every mapping is exposed on the embedded server under a deterministic node id
    std::string nodeId = "ns=2;s=Webots." + m.id;
    mBackend->exposeVariable(nodeId, m.browseName.empty() ? m.webotsTarget : m.browseName, m.dataType, Value(0.0),
                             m.direction == WRITE, error);
  }
}

void WbOpcUaManager::refreshMappingValue(const QString &id, const Value &value, const QString &quality) {
  const Value previous = mLastOpcValues.value(id);
  mLastOpcValues[id] = value;
  mQuality[id] = quality;
  if (!previous.approximatelyEquals(value, 1e-9))
    emit mappingValueChanged(id, QString::fromStdString(value.toString()));
}

void WbOpcUaManager::synchronize() {
  if (!isConnected())
    return;
  const std::vector<Mapping> &mappings = mMappingStore.mappings();
  for (size_t i = 0; i < mappings.size(); ++i) {
    const Mapping &m = mappings[i];
    if (!m.enabled)
      continue;
    const QString id = QString::fromStdString(m.id);
    std::string stdError;
    QString error;

    if (m.direction == READ || m.direction == READ_WRITE) {
      // OPC UA -> Webots: fetch the variable (remote client or external server write)
      Value raw;
      bool ok;
      if (mBackend->isServerRunning()) {
        ok = mBackend->readExposed("ns=2;s=Webots." + m.id, raw, stdError);
        // fall back to the remote session for pure client mappings
        if (!ok && mBackend->isConnected())
          ok = mBackend->read(m.nodeId, raw, stdError);
      } else
        ok = mBackend->read(m.nodeId, raw, stdError);
      if (!ok) {
        refreshMappingValue(id, Value(), QString("bad: %1").arg(QString::fromStdString(stdError)));
        continue;
      }
      const double opcValue = raw.toDouble();
      double previous = qQNaN();
      if (mLastOpcValues.contains(id))
        previous = mLastOpcValues.value(id).toDouble();
      if (!qIsFinite(previous) || fabs(opcValue - previous) > m.deadband) {
        const double webotsValue = MappingStore::toWebots(m, opcValue);
        WbOpcUaTarget target;
        if (WbOpcUaTarget::resolve(QString::fromStdString(m.webotsTarget), target, error)) {
          if (!target.write(Value(webotsValue), error))
            refreshMappingValue(id, raw, QString("bad: %1").arg(error));
          else
            refreshMappingValue(id, raw, "good");
        } else
          refreshMappingValue(id, raw, QString("bad: %1").arg(error));
      } else
        refreshMappingValue(id, raw, mQuality.value(id, "good"));
    }

    if (m.direction == WRITE || m.direction == READ_WRITE) {
      // Webots -> OPC UA: publish the target value
      WbOpcUaTarget target;
      Value webotsValue;
      if (!WbOpcUaTarget::resolve(QString::fromStdString(m.webotsTarget), target, error)) {
        refreshMappingValue(id, mLastOpcValues.value(id), QString("bad: %1").arg(error));
        continue;
      }
      if (!target.read(webotsValue)) {
        refreshMappingValue(id, mLastOpcValues.value(id),
                            QString("bad: cannot read target '%1'").arg(QString::fromStdString(m.webotsTarget)));
        continue;
      }
      const double webotsDouble = webotsValue.toDouble();
      const double previous = mLastWebotsValues.value(id, qQNaN());
      const double opcDouble = MappingStore::toOpc(m, webotsDouble);
      if (!qIsFinite(previous) || fabs(webotsDouble - previous) > m.deadband) {
        bool ok;
        if (mBackend->isServerRunning())
          ok = mBackend->writeExposed("ns=2;s=Webots." + m.id, Value(opcDouble), stdError);
        else
          ok = true;  // server-only exposure
        if (!mBackend->isServerRunning() || !ok) {
          stdError.clear();
          ok = mBackend->write(m.nodeId, Value(opcDouble), stdError);
        }
        mLastWebotsValues[id] = webotsDouble;
        if (!ok)
          refreshMappingValue(id, Value(opcDouble), QString("bad: %1").arg(QString::fromStdString(stdError)));
        else if (m.direction == WRITE)
          refreshMappingValue(id, Value(opcDouble), "good");
      }
    }
  }
}
