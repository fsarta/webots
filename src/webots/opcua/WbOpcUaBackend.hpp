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

// Description: transport-neutral OPC UA endpoint interface used by Webots.
//
// One backend instance plays both roles used by the Webots OPC-UA module:
//   - a *client* connected to a remote OPC UA server (read/browse/choose variables),
//   - an embedded *server* exposing the mapped simulation variables.
// Implementations: WbOpcUaMockBackend (in-memory, tests and fallback) and
// WbOpcUaOpen62541Backend (real stack, compiled when open62541 is available).

#ifndef WB_OPC_UA_BACKEND_HPP
#define WB_OPC_UA_BACKEND_HPP

#include <memory>
#include <string>
#include <vector>

#include "WbOpcUaValue.hpp"

namespace wbopcua {

struct BrowseEntry {
  std::string nodeId;
  std::string displayName;
  std::string browseName;
  std::string description;
  std::string dataType;  // "Boolean", "Double", ... for variables, empty otherwise
  bool isVariable;
  bool hasChildren;
  Value value;  // last known value (variables only)

  BrowseEntry() : isVariable(false), hasChildren(false) {}
};

struct ConnectionConfig {
  std::string endpoint;        // e.g. "opc.tcp://localhost:4840"
  std::string securityPolicy;  // "None", "Basic256Sha256", ...
  std::string username;
  std::string password;
  int serverPort;  // port of the embedded Webots server (4840 by default)
  ConnectionConfig() : securityPolicy("None"), serverPort(4840) {}
};

class Backend {
public:
  virtual ~Backend() {}

  // ---- shared
  // opens the remote client session (and the embedded server when `exposeServer` is set)
  virtual bool connect(const ConnectionConfig &config, bool exposeServer, std::string &error) = 0;
  virtual void disconnect() = 0;
  virtual bool isConnected() const = 0;
  virtual std::string backendName() const = 0;
  virtual std::string endpointDescription() const = 0;

  // ---- client role (remote address space)
  virtual std::string remoteRootNodeId() const { return "i=85"; }  // OPC UA Objects folder
  virtual std::vector<BrowseEntry> browse(const std::string &nodeId, std::string &error) = 0;
  virtual bool read(const std::string &nodeId, Value &value, std::string &error) = 0;
  virtual bool write(const std::string &nodeId, const Value &value, std::string &error) = 0;

  // ---- server role (variables exposed by Webots)
  // creates or updates an exposed variable; `readOnly` variables reject external writes
  virtual bool exposeVariable(const std::string &nodeId, const std::string &browseName, Value::Type type,
                              const Value &initialValue, bool readOnly, std::string &error) = 0;
  virtual bool unexposeVariable(const std::string &nodeId) = 0;
  virtual std::vector<BrowseEntry> listExposed(std::string &error) = 0;
  // value of an exposed variable as currently seen by the embedded server
  // (including writes performed by external OPC UA clients)
  virtual bool readExposed(const std::string &nodeId, Value &value, std::string &error) = 0;
  // publishes a simulation value into an exposed variable
  virtual bool writeExposed(const std::string &nodeId, const Value &value, std::string &error) = 0;
  virtual bool isServerRunning() const = 0;
};

typedef std::shared_ptr<Backend> BackendPtr;

}  // namespace wbopcua

#endif
