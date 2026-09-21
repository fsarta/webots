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

// Description: transport-neutral OPC UA data value used by the Webots OPC-UA module.
//
// This module is intentionally free of Qt/open62541 dependencies so that the
// value coercion logic can be unit-tested standalone (see tests/src/opcua).

#ifndef WB_OPC_UA_VALUE_HPP
#define WB_OPC_UA_VALUE_HPP

#include <cstdint>
#include <string>

namespace wbopcua {

class Value {
public:
  enum Type {
    NULL_VALUE = 0,
    BOOLEAN,
    INT32,
    INT64,
    FLOAT,
    DOUBLE,
    STRING,
  };

  Value();
  explicit Value(bool v);
  explicit Value(int32_t v);
  explicit Value(int64_t v);
  explicit Value(float v);
  explicit Value(double v);
  explicit Value(const std::string &v);
  explicit Value(const char *v);

  Type type() const { return mType; }
  static const char *typeName(Type type);
  // parse "Boolean", "Int32", ... (OPC UA built-in type names); NULL_VALUE on error
  static Type typeFromName(const std::string &name);
  bool isNull() const { return mType == NULL_VALUE; }
  bool isNumeric() const { return mType >= BOOLEAN && mType <= DOUBLE; }

  // raw access
  bool boolValue() const { return mBool; }
  int32_t int32Value() const;
  int64_t int64Value() const;
  float floatValue() const;
  double doubleValue() const;
  const std::string &stringValue() const { return mString; }

  // coercion
  bool toBool() const;
  double toDouble() const;
  std::string toString() const;
  // returns a copy of this value converted to the requested type (best effort)
  Value convert(Type type) const;

  bool equals(const Value &other) const;
  // ordering used for change detection
  bool approximatelyEquals(const Value &other, double epsilon = 1e-12) const;

  // parse text into a value of the given type ("true", "3.14", "hello", ...)
  static bool parse(Type type, const std::string &text, Value &result);

private:
  Type mType;
  bool mBool;
  int64_t mInt;
  double mDouble;
  std::string mString;
};

}  // namespace wbopcua

#endif
