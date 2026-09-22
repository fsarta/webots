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

// Description: minimal JSON reader/writer used to persist OPC-UA mapping files.
//
// Supports the JSON subset needed by the mapping store (objects, arrays,
// strings, numbers, booleans and null) with no external dependency.

#ifndef WB_OPC_UA_JSON_HPP
#define WB_OPC_UA_JSON_HPP

#include <map>
#include <string>
#include <vector>

namespace wbopcua {
namespace json {

  enum Kind { NULL_KIND, BOOL_KIND, NUMBER_KIND, STRING_KIND, ARRAY_KIND, OBJECT_KIND };

  struct Value {
    Kind kind;
    bool boolean;
    double number;
    std::string string;
    std::vector<Value> array;
    std::vector<std::pair<std::string, Value> > object;  // preserves insertion order

    Value() : kind(NULL_KIND), boolean(false), number(0.0) {}

    static Value makeBool(bool v);
    static Value makeNumber(double v);
    static Value makeString(const std::string &v);
    static Value makeArray();
    static Value makeObject();

    bool isNull() const { return kind == NULL_KIND; }
    bool has(const std::string &key) const;
    const Value *find(const std::string &key) const;
    void set(const std::string &key, const Value &v);

    // convenience typed getters with defaults
    std::string getString(const std::string &key, const std::string &def = std::string()) const;
    double getNumber(const std::string &key, double def = 0.0) const;
    bool getBool(const std::string &key, bool def = false) const;
  };

  // serialize without unnecessary whitespace
  std::string serialize(const Value &v);
  // pretty-print with two-space indentation
  std::string serializePretty(const Value &v);

  // parse `text`; on failure returns a NULL_KIND value and fills `error`
  Value parse(const std::string &text, std::string &error);

}  // namespace json
}  // namespace wbopcua

#endif
