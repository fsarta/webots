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

#include "WbOpcUaJson.hpp"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace wbopcua {
namespace json {

  Value Value::makeBool(bool v) {
    Value r;
    r.kind = BOOL_KIND;
    r.boolean = v;
    return r;
  }

  Value Value::makeNumber(double v) {
    Value r;
    r.kind = NUMBER_KIND;
    r.number = v;
    return r;
  }

  Value Value::makeString(const std::string &v) {
    Value r;
    r.kind = STRING_KIND;
    r.string = v;
    return r;
  }

  Value Value::makeArray() {
    Value r;
    r.kind = ARRAY_KIND;
    return r;
  }

  Value Value::makeObject() {
    Value r;
    r.kind = OBJECT_KIND;
    return r;
  }

  bool Value::has(const std::string &key) const {
    return find(key) != NULL;
  }

  const Value *Value::find(const std::string &key) const {
    for (size_t i = 0; i < object.size(); ++i)
      if (object[i].first == key)
        return &object[i].second;
    return NULL;
  }

  void Value::set(const std::string &key, const Value &v) {
    for (size_t i = 0; i < object.size(); ++i)
      if (object[i].first == key) {
        object[i].second = v;
        return;
      }
    object.push_back(std::make_pair(key, v));
  }

  std::string Value::getString(const std::string &key, const std::string &def) const {
    const Value *v = find(key);
    return (v && v->kind == STRING_KIND) ? v->string : def;
  }

  double Value::getNumber(const std::string &key, double def) const {
    const Value *v = find(key);
    return (v && v->kind == NUMBER_KIND) ? v->number : def;
  }

  bool Value::getBool(const std::string &key, bool def) const {
    const Value *v = find(key);
    return (v && v->kind == BOOL_KIND) ? v->boolean : def;
  }

  // ---------------------------------------------------------------- serialize

  static void escapeString(const std::string &s, std::ostringstream &os) {
    os << '"';
    for (size_t i = 0; i < s.size(); ++i) {
      const char c = s[i];
      switch (c) {
        case '"':
          os << "\\\"";
          break;
        case '\\':
          os << "\\\\";
          break;
        case '\n':
          os << "\\n";
          break;
        case '\r':
          os << "\\r";
          break;
        case '\t':
          os << "\\t";
          break;
        default:
          if ((unsigned char)c < 0x20) {
            char buf[8];
            snprintf(buf, sizeof(buf), "\\u%04x", (unsigned char)c);
            os << buf;
          } else
            os << c;
      }
    }
    os << '"';
  }

  static void serializeTo(const Value &v, std::ostringstream &os, int indent, int depth) {
    const bool pretty = indent > 0;
    const std::string pad = pretty ? std::string((size_t)(depth + 1) * indent, ' ') : std::string();
    const std::string padEnd = pretty ? std::string((size_t)depth * indent, ' ') : std::string();
    switch (v.kind) {
      case NULL_KIND:
        os << "null";
        break;
      case BOOL_KIND:
        os << (v.boolean ? "true" : "false");
        break;
      case NUMBER_KIND: {
        // integral numbers are printed without decimals
        if (v.number == (double)(long long)v.number) {
          char buf[32];
          snprintf(buf, sizeof(buf), "%lld", (long long)v.number);
          os << buf;
        } else {
          char buf[40];
          snprintf(buf, sizeof(buf), "%.17g", v.number);
          os << buf;
        }
        break;
      }
      case STRING_KIND:
        escapeString(v.string, os);
        break;
      case ARRAY_KIND: {
        os << '[';
        for (size_t i = 0; i < v.array.size(); ++i) {
          if (i)
            os << ',';
          if (pretty)
            os << '\n' << pad;
          serializeTo(v.array[i], os, indent, depth + 1);
        }
        if (pretty && !v.array.empty())
          os << '\n' << padEnd;
        os << ']';
        break;
      }
      case OBJECT_KIND: {
        os << '{';
        for (size_t i = 0; i < v.object.size(); ++i) {
          if (i)
            os << ',';
          if (pretty)
            os << '\n' << pad;
          escapeString(v.object[i].first, os);
          os << (pretty ? ": " : ":");
          serializeTo(v.object[i].second, os, indent, depth + 1);
        }
        if (pretty && !v.object.empty())
          os << '\n' << padEnd;
        os << '}';
        break;
      }
    }
  }

  std::string serialize(const Value &v) {
    std::ostringstream os;
    serializeTo(v, os, 0, 0);
    return os.str();
  }

  std::string serializePretty(const Value &v) {
    std::ostringstream os;
    serializeTo(v, os, 2, 0);
    os << '\n';
    return os.str();
  }

  // ---------------------------------------------------------------- parse

  namespace {
    class Parser {
    public:
      Parser(const std::string &text) : mText(text), mPos(0) {}

      Value run(std::string &error) {
        skipWhitespace();
        Value v = parseValue(error);
        if (!error.empty())
          return Value();
        skipWhitespace();
        if (mPos != mText.size()) {
          error = "trailing characters after JSON document";
          return Value();
        }
        return v;
      }

    private:
      const std::string &mText;
      size_t mPos;

      void skipWhitespace() {
        while (mPos < mText.size() && isspace((unsigned char)mText[mPos]))
          ++mPos;
      }

      bool consume(char c) {
        if (mPos < mText.size() && mText[mPos] == c) {
          ++mPos;
          return true;
        }
        return false;
      }

      Value parseValue(std::string &error) {
        skipWhitespace();
        if (mPos >= mText.size()) {
          error = "unexpected end of JSON document";
          return Value();
        }
        const char c = mText[mPos];
        if (c == '{')
          return parseObject(error);
        if (c == '[')
          return parseArray(error);
        if (c == '"') {
          std::string s;
          if (!parseString(s, error))
            return Value();
          return Value::makeString(s);
        }
        if (c == 't' || c == 'f')
          return parseBool(error);
        if (c == 'n')
          return parseNull(error);
        return parseNumber(error);
      }

      Value parseObject(std::string &error) {
        Value v = Value::makeObject();
        consume('{');
        skipWhitespace();
        if (consume('}'))
          return v;
        while (true) {
          skipWhitespace();
          std::string key;
          if (!parseString(key, error))
            return Value();
          skipWhitespace();
          if (!consume(':')) {
            error = "expected ':' in JSON object";
            return Value();
          }
          Value item = parseValue(error);
          if (!error.empty())
            return Value();
          v.set(key, item);
          skipWhitespace();
          if (consume(','))
            continue;
          if (consume('}'))
            return v;
          error = "expected ',' or '}' in JSON object";
          return Value();
        }
      }

      Value parseArray(std::string &error) {
        Value v = Value::makeArray();
        consume('[');
        skipWhitespace();
        if (consume(']'))
          return v;
        while (true) {
          Value item = parseValue(error);
          if (!error.empty())
            return Value();
          v.array.push_back(item);
          skipWhitespace();
          if (consume(','))
            continue;
          if (consume(']'))
            return v;
          error = "expected ',' or ']' in JSON array";
          return Value();
        }
      }

      bool parseString(std::string &out, std::string &error) {
        if (!consume('"')) {
          error = "expected JSON string";
          return false;
        }
        out.clear();
        while (mPos < mText.size()) {
          const char c = mText[mPos++];
          if (c == '"')
            return true;
          if (c == '\\') {
            if (mPos >= mText.size())
              break;
            const char e = mText[mPos++];
            switch (e) {
              case '"':
                out += '"';
                break;
              case '\\':
                out += '\\';
                break;
              case '/':
                out += '/';
                break;
              case 'b':
                out += '\b';
                break;
              case 'f':
                out += '\f';
                break;
              case 'n':
                out += '\n';
                break;
              case 'r':
                out += '\r';
                break;
              case 't':
                out += '\t';
                break;
              case 'u': {
                if (mPos + 4 > mText.size()) {
                  error = "truncated \\u escape in JSON string";
                  return false;
                }
                const std::string hex = mText.substr(mPos, 4);
                mPos += 4;
                const long code = strtol(hex.c_str(), NULL, 16);
                if (code < 0x80)
                  out += (char)code;
                else if (code < 0x800) {  // 2-byte UTF-8
                  out += (char)(0xC0 | (code >> 6));
                  out += (char)(0x80 | (code & 0x3F));
                } else {  // 3-byte UTF-8 (surrogate pairs not needed for mapping files)
                  out += (char)(0xE0 | (code >> 12));
                  out += (char)(0x80 | ((code >> 6) & 0x3F));
                  out += (char)(0x80 | (code & 0x3F));
                }
                break;
              }
              default:
                error = "invalid escape in JSON string";
                return false;
            }
          } else
            out += c;
        }
        error = "unterminated JSON string";
        return false;
      }

      Value parseBool(std::string &error) {
        if (mText.compare(mPos, 4, "true") == 0) {
          mPos += 4;
          return Value::makeBool(true);
        }
        if (mText.compare(mPos, 5, "false") == 0) {
          mPos += 5;
          return Value::makeBool(false);
        }
        error = "invalid JSON boolean";
        return Value();
      }

      Value parseNull(std::string &error) {
        if (mText.compare(mPos, 4, "null") == 0) {
          mPos += 4;
          return Value();
        }
        error = "invalid JSON null";
        return Value();
      }

      Value parseNumber(std::string &error) {
        const char *start = mText.c_str() + mPos;
        char *end = NULL;
        const double v = strtod(start, &end);
        if (end == start) {
          error = "invalid JSON number";
          return Value();
        }
        mPos += (size_t)(end - start);
        return Value::makeNumber(v);
      }
    };
  }  // namespace

  Value parse(const std::string &text, std::string &error) {
    error.clear();
    Parser parser(text);
    return parser.run(error);
  }

}  // namespace json
}  // namespace wbopcua
