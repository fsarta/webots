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

#include "WbOpcUaMockBackend.hpp"

namespace wbopcua {

MockBackend::MockBackend() : mConnected(false), mServerRunning(false), mBrowseCount(0) {
  // simulated remote PLC
  Node objects;
  objects.nodeId = "i=85";
  objects.displayName = "Objects";
  objects.browseName = "Objects";
  mRemote[objects.nodeId] = objects;
  mChildren[""] = std::vector<std::string>(1, objects.nodeId);

  Node plc;
  plc.nodeId = "ns=1;s=PLC";
  plc.parent = "i=85";
  plc.displayName = "PLC";
  plc.browseName = "PLC";
  plc.description = "Demo PLC used by the mock OPC UA backend";
  addNode(plc);

  Node setpoint;
  setpoint.nodeId = "ns=1;s=PLC.Setpoint";
  setpoint.parent = plc.nodeId;
  setpoint.displayName = "Setpoint";
  setpoint.browseName = "Setpoint";
  setpoint.description = "Axis setpoint (rad)";
  setpoint.dataType = "Double";
  setpoint.isVariable = true;
  setpoint.readOnly = false;
  setpoint.value = Value(0.0);
  addNode(setpoint);

  Node speed;
  speed.nodeId = "ns=1;s=PLC.Speed";
  speed.parent = plc.nodeId;
  speed.displayName = "Speed";
  speed.browseName = "Speed";
  speed.description = "Axis speed (rad/s)";
  speed.dataType = "Double";
  speed.isVariable = true;
  speed.readOnly = false;
  speed.value = Value(0.0);
  addNode(speed);

  Node running;
  running.nodeId = "ns=1;s=PLC.Running";
  running.parent = plc.nodeId;
  running.displayName = "Running";
  running.browseName = "Running";
  running.description = "Axis running flag";
  running.dataType = "Boolean";
  running.isVariable = true;
  running.readOnly = false;
  running.value = Value(false);
  addNode(running);

  Node line;
  line.nodeId = "ns=1;s=PLC.Line";
  line.parent = plc.nodeId;
  line.displayName = "Line";
  line.browseName = "Line";
  addNode(line);

  Node counter;
  counter.nodeId = "ns=1;s=PLC.Line.Counter";
  counter.parent = line.nodeId;
  counter.displayName = "Counter";
  counter.browseName = "Counter";
  counter.description = "Production counter";
  counter.dataType = "Int32";
  counter.isVariable = true;
  counter.readOnly = true;
  counter.value = Value((int32_t)0);
  addNode(counter);
}

void MockBackend::addNode(const Node &node) {
  mRemote[node.nodeId] = node;
  mChildren[node.parent].push_back(node.nodeId);
}

bool MockBackend::connect(const ConnectionConfig &config, bool exposeServer, std::string &error) {
  error.clear();
  if (config.endpoint.empty()) {
    error = "empty endpoint URL";
    return false;
  }
  // the mock simulates any opc.tcp endpoint, including "mock://" test endpoints
  mEndpoint = config.endpoint;
  mConnected = true;
  mServerRunning = exposeServer;
  return true;
}

void MockBackend::disconnect() {
  mConnected = false;
  mServerRunning = false;
}

std::vector<BrowseEntry> MockBackend::browse(const std::string &nodeId, std::string &error) {
  ++mBrowseCount;
  std::vector<BrowseEntry> result;
  error.clear();
  if (!mConnected) {
    error = "not connected";
    return result;
  }
  std::string key = nodeId.empty() ? std::string("i=85") : nodeId;
  std::map<std::string, std::vector<std::string> >::const_iterator it = mChildren.find(key);
  if (it == mChildren.end()) {
    // variable nodes have no children but browsing them is not an error
    if (mRemote.count(key))
      return result;
    error = "unknown node '" + key + "'";
    return result;
  }
  for (size_t i = 0; i < it->second.size(); ++i) {
    const Node &n = mRemote[it->second[i]];
    BrowseEntry e;
    e.nodeId = n.nodeId;
    e.displayName = n.displayName;
    e.browseName = n.browseName;
    e.description = n.description;
    e.dataType = n.dataType;
    e.isVariable = n.isVariable;
    e.hasChildren = mChildren.count(n.nodeId) && !mChildren[n.nodeId].empty();
    e.value = n.value;
    result.push_back(e);
  }
  return result;
}

bool MockBackend::read(const std::string &nodeId, Value &value, std::string &error) {
  error.clear();
  if (!mConnected) {
    error = "not connected";
    return false;
  }
  std::map<std::string, Node>::const_iterator it = mRemote.find(nodeId);
  if (it == mRemote.end() || !it->second.isVariable) {
    error = "unknown variable '" + nodeId + "'";
    return false;
  }
  value = it->second.value;
  return true;
}

bool MockBackend::write(const std::string &nodeId, const Value &value, std::string &error) {
  error.clear();
  if (!mConnected) {
    error = "not connected";
    return false;
  }
  std::map<std::string, Node>::iterator it = mRemote.find(nodeId);
  if (it == mRemote.end() || !it->second.isVariable) {
    error = "unknown variable '" + nodeId + "'";
    return false;
  }
  if (it->second.readOnly) {
    error = "variable '" + nodeId + "' is read-only";
    return false;
  }
  it->second.value = value.convert(Value::typeFromName(it->second.dataType));
  return true;
}

bool MockBackend::exposeVariable(const std::string &nodeId, const std::string &browseName, Value::Type type,
                                const Value &initialValue, bool readOnly, std::string &error) {
  error.clear();
  if (nodeId.empty()) {
    error = "empty node id";
    return false;
  }
  Node &n = mExposed[nodeId];
  n.nodeId = nodeId;
  n.parent = "ns=0;i=85";
  n.displayName = browseName;
  n.browseName = browseName;
  n.dataType = Value::typeName(type);
  n.isVariable = true;
  n.readOnly = readOnly;
  n.value = initialValue.convert(type);
  return true;
}

bool MockBackend::unexposeVariable(const std::string &nodeId) {
  return mExposed.erase(nodeId) > 0;
}

std::vector<BrowseEntry> MockBackend::listExposed(std::string &error) {
  std::vector<BrowseEntry> result;
  error.clear();
  if (!mServerRunning) {
    error = "embedded server is not running";
    return result;
  }
  for (std::map<std::string, Node>::const_iterator it = mExposed.begin(); it != mExposed.end(); ++it) {
    BrowseEntry e;
    e.nodeId = it->second.nodeId;
    e.displayName = it->second.displayName;
    e.browseName = it->second.browseName;
    e.dataType = it->second.dataType;
    e.isVariable = true;
    e.value = it->second.value;
    result.push_back(e);
  }
  return result;
}

bool MockBackend::readExposed(const std::string &nodeId, Value &value, std::string &error) {
  error.clear();
  std::map<std::string, Node>::const_iterator it = mExposed.find(nodeId);
  if (it == mExposed.end()) {
    error = "unknown exposed variable '" + nodeId + "'";
    return false;
  }
  value = it->second.value;
  return true;
}

bool MockBackend::writeExposed(const std::string &nodeId, const Value &value, std::string &error) {
  error.clear();
  std::map<std::string, Node>::iterator it = mExposed.find(nodeId);
  if (it == mExposed.end()) {
    error = "unknown exposed variable '" + nodeId + "'";
    return false;
  }
  it->second.value = value.convert(Value::typeFromName(it->second.dataType));
  return true;
}

bool MockBackend::setRemoteValue(const std::string &nodeId, const Value &value) {
  std::map<std::string, Node>::iterator it = mRemote.find(nodeId);
  if (it == mRemote.end() || !it->second.isVariable)
    return false;
  it->second.value = value.convert(Value::typeFromName(it->second.dataType));
  return true;
}

Value MockBackend::remoteValue(const std::string &nodeId, bool *ok) const {
  std::map<std::string, Node>::const_iterator it = mRemote.find(nodeId);
  const bool found = it != mRemote.end() && it->second.isVariable;
  if (ok)
    *ok = found;
  return found ? it->second.value : Value();
}

}  // namespace wbopcua
