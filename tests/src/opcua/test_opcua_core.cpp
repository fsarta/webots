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

// Standalone unit tests for the Webots OPC-UA core (values, JSON, mappings, mock backend).

#include <cstdio>
#include <cstdlib>
#include <string>

#include "WbOpcUaJson.hpp"
#include "WbOpcUaMapping.hpp"
#include "WbOpcUaMockBackend.hpp"
#include "WbOpcUaValue.hpp"

static int gFailures = 0;
static int gChecks = 0;

#define CHECK(cond) \
  do { \
    ++gChecks; \
    if (!(cond)) { \
      ++gFailures; \
      fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    } \
  } while (0)

#define CHECK_EQ(a, b) \
  do { \
    ++gChecks; \
    if (!((a) == (b))) { \
      ++gFailures; \
      fprintf(stderr, "FAIL %s:%d: %s == %s\n", __FILE__, __LINE__, #a, #b); \
    } \
  } while (0)

using namespace wbopcua;

static void testValue() {
  Value i((int32_t)42);
  CHECK(i.type() == Value::INT32);
  CHECK_EQ(i.toString(), std::string("42"));
  CHECK(i.toBool());
  CHECK(i.toDouble() == 42.0);

  Value d(3.25);
  CHECK(d.convert(Value::STRING).toString() == "3.25");
  CHECK(d.convert(Value::INT32).int32Value() == 3);
  CHECK(d.convert(Value::BOOLEAN).toBool());

  Value s("hello");
  CHECK(s.type() == Value::STRING);
  CHECK(s.convert(Value::DOUBLE).isNull() == false);

  Value parsed;
  CHECK(Value::parse(Value::DOUBLE, "-1.5", parsed));
  CHECK(parsed.doubleValue() == -1.5);
  CHECK(Value::parse(Value::BOOLEAN, "true", parsed));
  CHECK(parsed.toBool());
  CHECK(!Value::parse(Value::BOOLEAN, "maybe", parsed));

  CHECK(Value::typeFromName("Double") == Value::DOUBLE);
  CHECK(Value::typeFromName("Nope") == Value::NULL_VALUE);
  CHECK(Value(1.0).approximatelyEquals(Value(1.0 + 1e-15)));

  Value t(true);
  CHECK(t.approximatelyEquals(Value(1.0)));  // numeric comparison across types
}

static void testJson() {
  std::string error;
  json::Value v = json::parse("{\"a\": 1, \"b\": [true, \"x\"], \"c\": {\"d\": null}}", error);
  CHECK(error.empty());
  CHECK(v.kind == json::OBJECT_KIND);
  CHECK(v.getNumber("a") == 1.0);
  const json::Value *b = v.find("b");
  CHECK(b && b->kind == json::ARRAY_KIND && b->array.size() == 2);
  CHECK(b->array[0].kind == json::BOOL_KIND && b->array[0].boolean);
  CHECK(b->array[1].string == "x");
  const json::Value *c = v.find("c");
  CHECK(c && c->find("d") && c->find("d")->isNull());

  // roundtrip
  const std::string text = json::serialize(v);
  json::Value v2 = json::parse(text, error);
  CHECK(error.empty());
  CHECK(json::serialize(v2) == text);

  // escapes
  json::Value e = json::parse("\"a\\n\\\"b\\u0041\"", error);
  CHECK(error.empty());
  CHECK(e.string == "a\n\"bA");

  // pretty print parses back
  json::Value p = json::parse(json::serializePretty(v), error);
  CHECK(error.empty());

  // errors
  json::parse("{", error);
  CHECK(!error.empty());
  json::parse("[1,]", error);
  CHECK(!error.empty());
  json::parse("tru", error);
  CHECK(!error.empty());
}

static void testMappingStore() {
  MappingStore store;
  Mapping m;
  m.webotsTarget = "MyRobot/shoulder_motor/targetPosition";
  m.nodeId = "ns=1;s=PLC.Setpoint";
  m.direction = READ;
  m.scale = 2.0;
  m.offset = 1.0;
  const std::string id1 = store.add(m);
  CHECK(!id1.empty());

  Mapping m2;
  m2.webotsTarget = "MyRobot/shoulder_sensor/position";
  m2.nodeId = "ns=1;s=PLC.Speed";
  m2.direction = WRITE;
  const std::string id2 = store.add(m2);
  CHECK(id1 != id2);
  CHECK_EQ(store.count(), 2);
  CHECK(store.findByTarget("MyRobot/shoulder_motor/targetPosition") != NULL);
  CHECK(store.findByNodeId("ns=1;s=PLC.Setpoint") != NULL);

  CHECK(MappingStore::toWebots(m, 3.0) == 7.0);  // 2*3+1
  CHECK(MappingStore::toOpc(m, 7.0) == 3.0);

  CHECK(store.validate().empty());

  // duplicate read-capable target is a conflict
  Mapping m3 = m;
  m3.nodeId = "ns=1;s=PLC.Other";
  store.add(m3);
  CHECK(!store.validate().empty());
  store.remove(store.mappings().back().id);
  CHECK(store.validate().empty());

  // serialization roundtrip
  const std::string json = store.toJsonString();
  MappingStore loaded;
  std::string error;
  CHECK(loaded.fromJsonString(json, error));
  CHECK(error.empty());
  CHECK_EQ(loaded.count(), 2);
  const Mapping *lm = loaded.find(id1);
  CHECK(lm && lm->scale == 2.0 && lm->offset == 1.0 && lm->direction == READ);
  CHECK(loaded.find(id2)->direction == WRITE);
  CHECK(loaded.toJsonString() == json);  // stable output

  // invalid documents leave the store unchanged
  CHECK(!loaded.fromJsonString("{ \"mappings\": 3 }", error));
  CHECK(!error.empty());
  CHECK_EQ(loaded.count(), 2);
  CHECK(!loaded.fromJsonString("{ \"mappings\": [ { \"direction\": \"sideways\" } ] }", error));
  CHECK_EQ(loaded.count(), 2);

  // id counter stays unique after load
  Mapping m4;
  const std::string id4 = loaded.add(m4);
  CHECK(loaded.find(id4) != NULL);
  CHECK(id4 != id1 && id4 != id2);
}

static void testMockBackend() {
  MockBackend backend;
  std::string error;
  CHECK(!backend.connect(ConnectionConfig(), true, error));  // empty endpoint rejected

  ConnectionConfig config;
  config.endpoint = "opc.tcp://localhost:4840";
  CHECK(backend.connect(config, true, error));
  CHECK(backend.isConnected());
  CHECK(backend.isServerRunning());

  std::vector<BrowseEntry> root = backend.browse("i=85", error);
  CHECK(error.empty());
  CHECK_EQ(root.size(), (size_t)1);
  CHECK(root[0].nodeId == "ns=1;s=PLC");
  CHECK(root[0].hasChildren);

  std::vector<BrowseEntry> plc = backend.browse("ns=1;s=PLC", error);
  CHECK_EQ(plc.size(), (size_t)4);

  // read/write
  Value v;
  CHECK(backend.read("ns=1;s=PLC.Setpoint", v, error));
  CHECK(v.doubleValue() == 0.0);
  CHECK(backend.write("ns=1;s=PLC.Setpoint", Value(1.5), error));
  CHECK(backend.read("ns=1;s=PLC.Setpoint", v, error));
  CHECK(v.doubleValue() == 1.5);

  // read-only variable rejects writes
  CHECK(!backend.write("ns=1;s=PLC.Line.Counter", Value((int32_t)5), error));
  CHECK(!error.empty());

  // unknown node
  CHECK(!backend.read("ns=1;s=Nope", v, error));

  // embedded server side
  CHECK(backend.exposeVariable("ns=2;s=Webots.Joint1.Position", "Joint1_Position", Value::DOUBLE, Value(0.25),
                               true, error));
  CHECK(backend.writeExposed("ns=2;s=Webots.Joint1.Position", Value(0.5), error));
  CHECK(backend.readExposed("ns=2;s=Webots.Joint1.Position", v, error));
  CHECK(v.doubleValue() == 0.5);
  std::vector<BrowseEntry> exposed = backend.listExposed(error);
  CHECK_EQ(exposed.size(), (size_t)1);
  CHECK(exposed[0].browseName == "Joint1_Position");

  // test helper: remote PLC value injection
  CHECK(backend.setRemoteValue("ns=1;s=PLC.Running", Value(true)));
  CHECK(backend.remoteValue("ns=1;s=PLC.Running").toBool());

  backend.disconnect();
  CHECK(!backend.isConnected());
  CHECK(backend.browse("i=85", error).empty());
}

// end-to-end: a mapping synchronized against the mock, like WbOpcUaManager does
static void testMappingSync() {
  MockBackend backend;
  std::string error;
  ConnectionConfig config;
  config.endpoint = "mock://plc";
  CHECK(backend.connect(config, true, error));

  MappingStore store;
  Mapping m;
  m.id = "m1";
  m.webotsTarget = "robot/motor/targetPosition";
  m.nodeId = "ns=1;s=PLC.Setpoint";
  m.direction = READ;  // OPC -> Webots
  m.scale = 0.001;     // millirad -> rad
  m.deadband = 0.0;
  store.add(m);

  // PLC writes 1500 mrad -> Webots target must become 1.5 rad
  CHECK(backend.setRemoteValue(m.nodeId, Value(1500.0)));
  Value opc;
  CHECK(backend.read(m.nodeId, opc, error));
  const double webots = MappingStore::toWebots(m, opc.doubleValue());
  CHECK(webots == 1.5);

  // Webots sensor -> OPC (WRITE direction)
  Mapping s;
  s.id = "m2";
  s.webotsTarget = "robot/sensor/position";
  s.nodeId = "ns=1;s=PLC.Speed";
  s.direction = WRITE;
  store.add(s);
  const double opcValue = MappingStore::toOpc(s, 2.5);
  CHECK(backend.write(s.nodeId, Value(opcValue), error));
  CHECK(backend.remoteValue(s.nodeId).doubleValue() == 2.5);
}

int main() {
  testValue();
  testJson();
  testMappingStore();
  testMockBackend();
  testMappingSync();

  if (gFailures) {
    fprintf(stderr, "%d/%d checks failed\n", gFailures, gChecks);
    return 1;
  }
  printf("test_opcua_core: all %d checks passed\n", gChecks);
  return 0;
}
