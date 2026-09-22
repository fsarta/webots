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

// Description: in-memory OPC UA backend used for unit tests, UI development and
//              as a fallback when Webots is built without the open62541 stack.
//
// It simulates a small remote PLC address space:
//   Objects (i=85)
//   └── PLC (ns=1;s=PLC)
//       ├── Setpoint  (ns=1;s=PLC.Setpoint, Double)
//       ├── Speed     (ns=1;s=PLC.Speed, Double)
//       ├── Running   (ns=1;s=PLC.Running, Boolean)
//       └── Line (ns=1;s=PLC.Line)
//           └── Counter (ns=1;s=PLC.Line.Counter, Int32)
// plus the embedded-server side where Webots exposes its own variables.

#ifndef WB_OPC_UA_MOCK_BACKEND_HPP
#define WB_OPC_UA_MOCK_BACKEND_HPP

#include <map>
#include <string>
#include <vector>

#include "WbOpcUaBackend.hpp"

namespace wbopcua {

class MockBackend : public Backend {
public:
  MockBackend();

  // Backend interface
  bool connect(const ConnectionConfig &config, bool exposeServer, std::string &error) override;
  void disconnect() override;
  bool isConnected() const override { return mConnected; }
  std::string backendName() const override { return "Mock"; }
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
  bool isServerRunning() const override { return mServerRunning; }

  // test helpers (simulate a remote PLC)
  bool setRemoteValue(const std::string &nodeId, const Value &value);
  Value remoteValue(const std::string &nodeId, bool *ok = NULL) const;
  int browseCount() const { return mBrowseCount; }

private:
  struct Node {
    std::string nodeId;
    std::string parent;
    std::string displayName;
    std::string browseName;
    std::string description;
    std::string dataType;
    bool isVariable;
    bool readOnly;
    Value value;
    Node() : isVariable(false), readOnly(true) {}
  };

  void addNode(const Node &node);

  bool mConnected;
  bool mServerRunning;
  std::string mEndpoint;
  int mBrowseCount;
  std::map<std::string, Node> mRemote;   // simulated external server
  std::map<std::string, Node> mExposed;  // variables exposed by the embedded server
  std::map<std::string, std::vector<std::string> > mChildren;
};

}  // namespace wbopcua

#endif
