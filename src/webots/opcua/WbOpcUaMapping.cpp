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

#include "WbOpcUaMapping.hpp"

#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>

#include "WbOpcUaJson.hpp"

namespace wbopcua {

const char *directionName(Direction d) {
  switch (d) {
    case READ:
      return "read";
    case WRITE:
      return "write";
    case READ_WRITE:
      return "readWrite";
  }
  return "read";
}

Direction directionFromName(const std::string &name, bool *ok) {
  if (name == "read") {
    if (ok)
      *ok = true;
    return READ;
  }
  if (name == "write") {
    if (ok)
      *ok = true;
    return WRITE;
  }
  if (name == "readWrite") {
    if (ok)
      *ok = true;
    return READ_WRITE;
  }
  if (ok)
    *ok = false;
  return READ;
}

std::string MappingStore::add(const Mapping &m) {
  Mapping copy = m;
  if (copy.id.empty()) {
    char buf[32];
    snprintf(buf, sizeof(buf), "m%d", mNextId++);
    copy.id = buf;
  } else {
    // keep the id counter ahead of any explicit id of the form m<number>
    if (copy.id.size() > 1 && copy.id[0] == 'm') {
      const int n = atoi(copy.id.c_str() + 1);
      if (n >= mNextId)
        mNextId = n + 1;
    }
  }
  mMappings.push_back(copy);
  return copy.id;
}

bool MappingStore::remove(const std::string &id) {
  for (size_t i = 0; i < mMappings.size(); ++i)
    if (mMappings[i].id == id) {
      mMappings.erase(mMappings.begin() + i);
      return true;
    }
  return false;
}

void MappingStore::clear() {
  mMappings.clear();
}

Mapping *MappingStore::find(const std::string &id) {
  for (size_t i = 0; i < mMappings.size(); ++i)
    if (mMappings[i].id == id)
      return &mMappings[i];
  return NULL;
}

const Mapping *MappingStore::find(const std::string &id) const {
  for (size_t i = 0; i < mMappings.size(); ++i)
    if (mMappings[i].id == id)
      return &mMappings[i];
  return NULL;
}

const Mapping *MappingStore::findByTarget(const std::string &webotsTarget) const {
  for (size_t i = 0; i < mMappings.size(); ++i)
    if (mMappings[i].webotsTarget == webotsTarget)
      return &mMappings[i];
  return NULL;
}

const Mapping *MappingStore::findByNodeId(const std::string &nodeId) const {
  for (size_t i = 0; i < mMappings.size(); ++i)
    if (mMappings[i].nodeId == nodeId)
      return &mMappings[i];
  return NULL;
}

std::vector<std::string> MappingStore::validate() const {
  std::vector<std::string> problems;
  std::map<std::string, int> targets;
  std::map<std::string, int> nodes;
  for (size_t i = 0; i < mMappings.size(); ++i) {
    const Mapping &m = mMappings[i];
    if (m.webotsTarget.empty())
      problems.push_back("mapping '" + m.id + "' has no Webots target");
    if (m.nodeId.empty())
      problems.push_back("mapping '" + m.id + "' has no OPC UA node id");
    if (!m.webotsTarget.empty() && ++targets[m.webotsTarget] > 1 && m.direction != WRITE)
      problems.push_back("multiple read-capable mappings target '" + m.webotsTarget + "'");
    if (!m.nodeId.empty() && ++nodes[m.nodeId] > 1 && m.direction != READ)
      problems.push_back("multiple write-capable mappings use OPC node '" + m.nodeId + "'");
    if (m.scale == 0.0)
      problems.push_back("mapping '" + m.id + "' has a null scale and will produce constant values");
    if (m.deadband < 0.0)
      problems.push_back("mapping '" + m.id + "' has a negative deadband");
    if (m.direction == WRITE && m.dataType == Value::NULL_VALUE)
      problems.push_back("mapping '" + m.id + "' has no data type");
  }
  return problems;
}

double MappingStore::toWebots(const Mapping &m, double opcValue) {
  return m.scale * opcValue + m.offset;
}

double MappingStore::toOpc(const Mapping &m, double webotsValue) {
  if (m.scale == 0.0)
    return 0.0;
  return (webotsValue - m.offset) / m.scale;
}

std::string MappingStore::toJsonString(bool pretty) const {
  json::Value root = json::Value::makeObject();
  root.set("version", json::Value::makeNumber(1));
  json::Value list = json::Value::makeArray();
  for (size_t i = 0; i < mMappings.size(); ++i) {
    const Mapping &m = mMappings[i];
    json::Value item = json::Value::makeObject();
    item.set("id", json::Value::makeString(m.id));
    item.set("target", json::Value::makeString(m.webotsTarget));
    item.set("nodeId", json::Value::makeString(m.nodeId));
    item.set("browseName", json::Value::makeString(m.browseName));
    item.set("direction", json::Value::makeString(directionName(m.direction)));
    item.set("dataType", json::Value::makeString(Value::typeName(m.dataType)));
    item.set("scale", json::Value::makeNumber(m.scale));
    item.set("offset", json::Value::makeNumber(m.offset));
    item.set("deadband", json::Value::makeNumber(m.deadband));
    item.set("enabled", json::Value::makeBool(m.enabled));
    list.array.push_back(item);
  }
  root.set("mappings", list);
  return pretty ? json::serializePretty(root) : json::serialize(root);
}

bool MappingStore::fromJsonString(const std::string &text, std::string &error) {
  const json::Value root = json::parse(text, error);
  if (!error.empty())
    return false;
  if (root.kind != json::OBJECT_KIND) {
    error = "mapping file must contain a JSON object";
    return false;
  }
  const json::Value *list = root.find("mappings");
  if (!list || list->kind != json::ARRAY_KIND) {
    error = "mapping file must contain a 'mappings' array";
    return false;
  }
  std::vector<Mapping> loaded;
  for (size_t i = 0; i < list->array.size(); ++i) {
    const json::Value &item = list->array[i];
    if (item.kind != json::OBJECT_KIND) {
      error = "mapping entry must be a JSON object";
      return false;
    }
    Mapping m;
    m.id = item.getString("id");
    m.webotsTarget = item.getString("target");
    m.nodeId = item.getString("nodeId");
    m.browseName = item.getString("browseName");
    bool ok = false;
    m.direction = directionFromName(item.getString("direction", "read"), &ok);
    if (!ok) {
      error = "invalid direction in mapping '" + m.id + "'";
      return false;
    }
    m.dataType = Value::typeFromName(item.getString("dataType", "Double"));
    if (m.dataType == Value::NULL_VALUE && item.getString("dataType") != "Null") {
      error = "invalid dataType in mapping '" + m.id + "'";
      return false;
    }
    m.scale = item.getNumber("scale", 1.0);
    m.offset = item.getNumber("offset", 0.0);
    m.deadband = item.getNumber("deadband", 0.0);
    m.enabled = item.getBool("enabled", true);
    loaded.push_back(m);
  }
  // commit only when the whole document is valid
  mMappings = loaded;
  mNextId = 1;
  for (size_t i = 0; i < mMappings.size(); ++i) {
    const std::string &id = mMappings[i].id;
    if (id.size() > 1 && id[0] == 'm') {
      const int n = atoi(id.c_str() + 1);
      if (n >= mNextId)
        mNextId = n + 1;
    }
  }
  return true;
}

}  // namespace wbopcua
