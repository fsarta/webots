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

// Description: OPC-UA variable mappings: binding between Webots targets
//              (joint motors, sensors, node fields) and OPC UA node ids.
//
// Pure C++ (no Qt) so the store can be unit-tested standalone.

#ifndef WB_OPC_UA_MAPPING_HPP
#define WB_OPC_UA_MAPPING_HPP

#include <string>
#include <vector>

#include "WbOpcUaValue.hpp"

namespace wbopcua {

// Direction of a mapping:
//   READ       OPC UA -> Webots  (e.g. PLC setpoint drives a motor)
//   WRITE      Webots -> OPC UA  (e.g. a sensor value is published)
//   READ_WRITE both directions (an external writable value is synchronized)
enum Direction {
  READ = 0,
  WRITE,
  READ_WRITE,
};

const char *directionName(Direction d);
Direction directionFromName(const std::string &name, bool *ok = NULL);

struct Mapping {
  std::string id;           // unique identifier ("m1", "m2", ...)
  std::string webotsTarget; // e.g. "MyRobot/shoulder_motor/targetPosition" or field "SHOULDER/HingeJointParameters/minStop"
  std::string nodeId;       // OPC UA node id, e.g. "ns=2;s=PLC1.DB1.Setpoint"
  std::string browseName;   // display hint filled by the variable browser
  Direction direction;
  Value::Type dataType;
  double scale;     // webotsValue = scale * opcValue + offset (WRITE: inverse)
  double offset;
  double deadband;  // skip updates smaller than this (in OPC units)
  bool enabled;

  Mapping() :
    direction(READ), dataType(Value::DOUBLE), scale(1.0), offset(0.0), deadband(0.0), enabled(true) {}
};

class MappingStore {
public:
  MappingStore() : mNextId(1) {}

  // returns the id of the inserted mapping
  std::string add(const Mapping &m);
  bool remove(const std::string &id);
  void clear();
  int count() const { return (int)mMappings.size(); }
  bool empty() const { return mMappings.empty(); }
  const std::vector<Mapping> &mappings() const { return mMappings; }
  Mapping *find(const std::string &id);
  const Mapping *find(const std::string &id) const;
  // first mapping bound to a given Webots target or OPC node id
  const Mapping *findByTarget(const std::string &webotsTarget) const;
  const Mapping *findByNodeId(const std::string &nodeId) const;

  // validation report (empty when the store is consistent)
  std::vector<std::string> validate() const;

  // apply scale/offset: OPC units -> Webots units and back
  static double toWebots(const Mapping &m, double opcValue);
  static double toOpc(const Mapping &m, double webotsValue);

  // JSON (de)serialization of the whole store
  std::string toJsonString(bool pretty = true) const;
  // replaces the current content; on parse error the store is left unchanged and `error` is filled
  bool fromJsonString(const std::string &text, std::string &error);

private:
  std::vector<Mapping> mMappings;
  int mNextId;
};

}  // namespace wbopcua

#endif
