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

// Description: native OPC-UA backend implemented on top of the open62541 stack.
//              Client role browses/reads/writes a remote server; server role
//              exposes Webots variables through an embedded OPC-UA server.
//
// Only compiled when the open62541 amalgamation is available (WB_USE_OPEN62541,
// installed by scripts/install/open62541_installer.sh).

#ifndef WB_OPC_UA_OPEN62541_BACKEND_HPP
#define WB_OPC_UA_OPEN62541_BACKEND_HPP

#ifdef WB_USE_OPEN62541

#  include <map>
#  include <string>

#  include <open62541.h>

#  include "WbOpcUaBackend.hpp"

namespace wbopcua {

class WbOpcUaOpen62541Backend : public Backend {
public:
  WbOpcUaOpen62541Backend();
  ~WbOpcUaOpen62541Backend() override;

  bool connect(const ConnectionConfig &config, bool exposeServer, std::string &error) override;
  void disconnect() override;
  bool isConnected() const override { return mClientConnected; }
  std::string backendName() const override { return "open62541"; }
  std::string endpointDescription() const override { return mEndpoint; }

  std::vector<BrowseEntry> browse(const std::string &nodeId, std::string &error) override;
  bool read(const std::string &nodeId, Value &value, std::string &error) override;
  bool write(const std::string &nodeId, const Value &value, std::string &error) override;

  bool exposeVariable(const std::string &nodeId, const std::string &browseName, Value::Type type,
                      const Value &initialValue, bool readOnly, std::string &error) override;
  bool unexposeVariable(const std::string &nodeId) override;
  std::vector<BrowseEntry> listExposed(std::string &error) override;
  bool readExposed(const std::string &nodeId, Value &value, std::string &error) override;
  bool writeExposed(const std::string &nodeId, const Value &value, std::string &error) override;
  bool isServerRunning() const override { return mServer != NULL; }

private:
  struct ExposedNode {
    Value::Type type;
    std::string browseName;
    bool readOnly;
  };

  static UA_NodeId toNodeId(const std::string &id);
  static std::string fromNodeId(const UA_NodeId &id);
  static bool toVariant(const Value &value, Value::Type type, UA_Variant &variant);
  static bool fromVariant(const UA_Variant &variant, Value &value);
  static const UA_DataType *uaType(Value::Type type);
  void pumpServer();  // one server iteration (called on each I/O operation)

  UA_Client *mClient;
  UA_Server *mServer;
  UA_UInt16 mNamespaceIndex;
  bool mClientConnected;
  std::string mEndpoint;
  std::map<std::string, ExposedNode> mExposed;
};

}  // namespace wbopcua

#endif  // WB_USE_OPEN62541

#endif
