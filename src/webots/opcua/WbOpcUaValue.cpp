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

#include "WbOpcUaValue.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace wbopcua {

Value::Value() : mType(NULL_VALUE), mBool(false), mInt(0), mDouble(0.0) {
}

Value::Value(bool v) : mType(BOOLEAN), mBool(v), mInt(v ? 1 : 0), mDouble(v ? 1.0 : 0.0) {
}

Value::Value(int32_t v) : mType(INT32), mBool(v != 0), mInt(v), mDouble((double)v) {
}

Value::Value(int64_t v) : mType(INT64), mBool(v != 0), mInt(v), mDouble((double)v) {
}

Value::Value(float v) : mType(FLOAT), mBool(v != 0.0f), mInt((int64_t)v), mDouble((double)v) {
}

Value::Value(double v) : mType(DOUBLE), mBool(v != 0.0), mInt((int64_t)v), mDouble(v) {
}

Value::Value(const std::string &v) : mType(STRING), mBool(!v.empty()), mInt(0), mDouble(0.0), mString(v) {
  mInt = (int64_t)strtod(v.c_str(), NULL);
  mDouble = strtod(v.c_str(), NULL);
}

Value::Value(const char *v) : mType(STRING), mBool(v && *v), mInt(0), mDouble(0.0), mString(v ? v : "") {
  mInt = (int64_t)strtod(mString.c_str(), NULL);
  mDouble = strtod(mString.c_str(), NULL);
}

const char *Value::typeName(Type type) {
  switch (type) {
    case NULL_VALUE:
      return "Null";
    case BOOLEAN:
      return "Boolean";
    case INT32:
      return "Int32";
    case INT64:
      return "Int64";
    case FLOAT:
      return "Float";
    case DOUBLE:
      return "Double";
    case STRING:
      return "String";
  }
  return "Null";
}

Value::Type Value::typeFromName(const std::string &name) {
  for (int t = NULL_VALUE; t <= STRING; ++t) {
    if (name == typeName((Type)t))
      return (Type)t;
  }
  return NULL_VALUE;
}

int32_t Value::int32Value() const {
  return (int32_t)mInt;
}

int64_t Value::int64Value() const {
  return mInt;
}

float Value::floatValue() const {
  return (float)mDouble;
}

double Value::doubleValue() const {
  return mDouble;
}

bool Value::toBool() const {
  switch (mType) {
    case NULL_VALUE:
      return false;
    case BOOLEAN:
      return mBool;
    case STRING:
      return mString == "true" || mString == "1" || mString == "TRUE" || mString == "True";
    default:
      return mDouble != 0.0;
  }
}

double Value::toDouble() const {
  switch (mType) {
    case NULL_VALUE:
      return 0.0;
    case BOOLEAN:
      return mBool ? 1.0 : 0.0;
    case STRING:
      return strtod(mString.c_str(), NULL);
    default:
      return mDouble;
  }
}

std::string Value::toString() const {
  std::ostringstream os;
  switch (mType) {
    case NULL_VALUE:
      return "";
    case BOOLEAN:
      return mBool ? "true" : "false";
    case INT32:
    case INT64:
      os << mInt;
      return os.str();
    case FLOAT:
    case DOUBLE:
      os << mDouble;
      return os.str();
    case STRING:
      return mString;
  }
  return "";
}

Value Value::convert(Type type) const {
  if (type == mType)
    return *this;
  switch (type) {
    case NULL_VALUE:
      return Value();
    case BOOLEAN:
      return Value(toBool());
    case INT32:
      return Value((int32_t)toDouble());
    case INT64:
      return Value((int64_t)toDouble());
    case FLOAT:
      return Value((float)toDouble());
    case DOUBLE:
      return Value(toDouble());
    case STRING:
      return Value(toString());
  }
  return Value();
}

bool Value::equals(const Value &other) const {
  if (mType != other.mType)
    return false;
  switch (mType) {
    case NULL_VALUE:
      return true;
    case BOOLEAN:
      return mBool == other.mBool;
    case INT32:
    case INT64:
      return mInt == other.mInt;
    case FLOAT:
    case DOUBLE:
      return mDouble == other.mDouble;
    case STRING:
      return mString == other.mString;
  }
  return false;
}

bool Value::approximatelyEquals(const Value &other, double epsilon) const {
  if (isNumeric() && other.isNumeric())
    return fabs(toDouble() - other.toDouble()) <= epsilon;
  return equals(other);
}

bool Value::parse(Type type, const std::string &text, Value &result) {
  std::istringstream is(text);
  switch (type) {
    case NULL_VALUE:
      result = Value();
      return true;
    case BOOLEAN: {
      if (text == "true" || text == "1" || text == "TRUE" || text == "True") {
        result = Value(true);
        return true;
      }
      if (text == "false" || text == "0" || text == "FALSE" || text == "False") {
        result = Value(false);
        return true;
      }
      return false;
    }
    case INT32: {
      int32_t v;
      is >> v;
      if (is.fail() || !is.eof()) {
        // allow trailing spaces
        is.clear();
        is >> v;
        if (is.fail())
          return false;
      }
      result = Value(v);
      return true;
    }
    case INT64: {
      int64_t v;
      is >> v;
      if (is.fail())
        return false;
      result = Value(v);
      return true;
    }
    case FLOAT: {
      float v;
      is >> v;
      if (is.fail())
        return false;
      result = Value(v);
      return true;
    }
    case DOUBLE: {
      double v;
      is >> v;
      if (is.fail())
        return false;
      result = Value(v);
      return true;
    }
    case STRING:
      result = Value(text);
      return true;
  }
  return false;
}

}  // namespace wbopcua
