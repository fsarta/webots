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

#include "WbOpcUaOpen62541Backend.hpp"

#ifdef WB_USE_OPEN62541

#  include <cstring>
#  include <vector>

namespace wbopcua {

WbOpcUaOpen62541Backend::WbOpcUaOpen62541Backend() :
  mClient(NULL), mServer(NULL), mNamespaceIndex(2), mClientConnected(false) {
}

WbOpcUaOpen62541Backend::~WbOpcUaOpen62541Backend() {
  disconnect();
}

const UA_DataType *WbOpcUaOpen62541Backend::uaType(Value::Type type) {
  switch (type) {
    case Value::BOOLEAN:
      return &UA_TYPES[UA_TYPES_BOOLEAN];
    case Value::INT32:
      return &UA_TYPES[UA_TYPES_INT32];
    case Value::INT64:
      return &UA_TYPES[UA_TYPES_INT64];
    case Value::FLOAT:
      return &UA_TYPES[UA_TYPES_FLOAT];
    case Value::DOUBLE:
      return &UA_TYPES[UA_TYPES_DOUBLE];
    case Value::STRING:
      return &UA_TYPES[UA_TYPES_STRING];
    default:
      return &UA_TYPES[UA_TYPES_DOUBLE];
  }
}

UA_NodeId WbOpcUaOpen62541Backend::toNodeId(const std::string &id) {
  // accepts "i=...", "ns=N;i=...", "ns=N;s=..." and plain node ids
  UA_NodeId result = UA_NODEID_STRING_ALLOC(2, id.c_str());
  unsigned ns = 0;
  char kind = 0;
  char text[512];
  if (sscanf(id.c_str(), "ns=%u;%c=%511s", &ns, &kind, text) == 3) {
    UA_NodeId_clear(&result);
    if (kind == 'i')
      result = UA_NODEID_NUMERIC(ns, (UA_UInt32)strtoul(text, NULL, 10));
    else
      result = UA_NODEID_STRING_ALLOC(ns, text);
  } else if (sscanf(id.c_str(), "i=%511s", text) == 1) {
    UA_NodeId_clear(&result);
    result = UA_NODEID_NUMERIC(0, (UA_UInt32)strtoul(text, NULL, 10));
  }
  return result;
}

std::string WbOpcUaOpen62541Backend::fromNodeId(const UA_NodeId &id) {
  char buffer[600];
  if (id.identifierType == UA_NODEIDTYPE_NUMERIC)
    snprintf(buffer, sizeof(buffer), "ns=%u;i=%u", id.namespaceIndex, id.identifier.numeric);
  else if (id.identifierType == UA_NODEIDTYPE_STRING)
    snprintf(buffer, sizeof(buffer), "ns=%u;s=%.*s", id.namespaceIndex, (int)id.identifier.string.length,
             (const char *)id.identifier.string.data);
  else
    snprintf(buffer, sizeof(buffer), "ns=%u;(opaque)", id.namespaceIndex);
  return std::string(buffer);
}

bool WbOpcUaOpen62541Backend::toVariant(const Value &value, Value::Type type, UA_Variant &variant) {
  UA_Variant_init(&variant);
  switch (type) {
    case Value::BOOLEAN: {
      UA_Boolean v = value.toBool();
      UA_Variant_setScalarCopy(&variant, &v, &UA_TYPES[UA_TYPES_BOOLEAN]);
      return true;
    }
    case Value::INT32: {
      UA_Int32 v = value.int32Value();
      UA_Variant_setScalarCopy(&variant, &v, &UA_TYPES[UA_TYPES_INT32]);
      return true;
    }
    case Value::INT64: {
      UA_Int64 v = value.int64Value();
      UA_Variant_setScalarCopy(&variant, &v, &UA_TYPES[UA_TYPES_INT64]);
      return true;
    }
    case Value::FLOAT: {
      UA_Float v = value.floatValue();
      UA_Variant_setScalarCopy(&variant, &v, &UA_TYPES[UA_TYPES_FLOAT]);
      return true;
    }
    case Value::DOUBLE: {
      UA_Double v = value.doubleValue();
      UA_Variant_setScalarCopy(&variant, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
      return true;
    }
    case Value::STRING: {
      const std::string s = value.toString();
      UA_String v = UA_STRING_ALLOC(s.c_str());
      UA_Variant_setScalarCopy(&variant, &v, &UA_TYPES[UA_TYPES_STRING]);
      UA_String_clear(&v);
      return true;
    }
    default:
      return false;
  }
}

bool WbOpcUaOpen62541Backend::fromVariant(const UA_Variant &variant, Value &value) {
  if (!variant.type || !variant.data)
    return false;
  if (variant.type == &UA_TYPES[UA_TYPES_BOOLEAN])
    value = Value(*(UA_Boolean *)variant.data != 0);
  else if (variant.type == &UA_TYPES[UA_TYPES_INT32])
    value = Value((int32_t)*(UA_Int32 *)variant.data);
  else if (variant.type == &UA_TYPES[UA_TYPES_INT64])
    value = Value((int64_t)*(UA_Int64 *)variant.data);
  else if (variant.type == &UA_TYPES[UA_TYPES_FLOAT])
    value = Value((float)*(UA_Float *)variant.data);
  else if (variant.type == &UA_TYPES[UA_TYPES_DOUBLE])
    value = Value((double)*(UA_Double *)variant.data);
  else if (variant.type == &UA_TYPES[UA_TYPES_STRING]) {
    UA_String *s = (UA_String *)variant.data;
    value = Value(std::string((const char *)s->data, s->length));
  } else
    return false;
  return true;
}

bool WbOpcUaOpen62541Backend::connect(const ConnectionConfig &config, bool exposeServer, std::string &error) {
  disconnect();
  error.clear();
  mEndpoint = config.endpoint;
  if (config.endpoint.empty()) {
    error = "empty endpoint URL";
    return false;
  }

  // ---- client role
  mClient = UA_Client_new();
  UA_ClientConfig *clientConfig = UA_Client_getConfig(mClient);
  UA_ClientConfig_setDefault(clientConfig);
  if (config.securityPolicy != "None" && !config.securityPolicy.empty()) {
    // full security policy negotiation (certificates) is handled by open62541
    // when a policy URI is provided through the application options
    clientConfig->securityPolicyUri = UA_STRING_ALLOC(
      (std::string("http://opcfoundation.org/UA/SecurityPolicy#") + config.securityPolicy).c_str());
  }
  UA_StatusCode status;
  if (config.username.empty())
    status = UA_Client_connect(mClient, config.endpoint.c_str());
  else
    status = UA_Client_connectUsername(mClient, config.endpoint.c_str(), config.username.c_str(),
                                       config.password.c_str());
  mClientConnected = (status == UA_STATUSCODE_GOOD);
  if (!mClientConnected) {
    error = std::string("client connection failed: ") + UA_StatusCode_name(status);
    UA_Client_delete(mClient);
    mClient = NULL;
  }

  // ---- server role
  if (exposeServer) {
    mServer = UA_Server_new();
    UA_ServerConfig *serverConfig = UA_Server_getConfig(mServer);
    UA_ServerConfig_setMinimal(serverConfig, (UA_UInt16)config.serverPort, NULL);
    mNamespaceIndex = UA_Server_addNamespace(mServer, "urn:webots:opcua");
  }

  if (!mClientConnected && !mServer) {
    disconnect();
    return false;
  }
  if (mServer)
    pumpServer();
  return true;
}

void WbOpcUaOpen62541Backend::disconnect() {
  if (mClient) {
    if (mClientConnected)
      UA_Client_disconnect(mClient);
    UA_Client_delete(mClient);
    mClient = NULL;
  }
  if (mServer) {
    UA_Server_delete(mServer);
    mServer = NULL;
  }
  mClientConnected = false;
  mExposed.clear();
}

void WbOpcUaOpen62541Backend::pumpServer() {
  if (mServer)
    UA_Server_run_iterate(mServer, false);
}

std::vector<BrowseEntry> WbOpcUaOpen62541Backend::browse(const std::string &nodeId, std::string &error) {
  std::vector<BrowseEntry> result;
  error.clear();
  pumpServer();
  if (!mClient || !mClientConnected) {
    error = "not connected";
    return result;
  }
  UA_BrowseRequest request;
  UA_BrowseRequest_init(&request);
  request.requestedMaxReferencesPerNode = 1000;
  request.nodesToBrowse = UA_BrowseDescription_new();
  request.nodesToBrowseSize = 1;
  UA_NodeId id = toNodeId(nodeId.empty() ? "i=85" : nodeId);
  UA_NodeId_copy(&id, &request.nodesToBrowse[0].nodeId);
  UA_NodeId_clear(&id);  // the copy lives in the request
  request.nodesToBrowse[0].browseDirection = UA_BROWSEDIRECTION_FORWARD;
  request.nodesToBrowse[0].resultMask = UA_BROWSERESULTMASK_ALL;

  UA_BrowseResponse response = UA_Client_Service_browse(mClient, request);
  UA_BrowseRequest_clear(&request);
  if (response.responseHeader.serviceResult != UA_STATUSCODE_GOOD) {
    error = std::string("browse failed: ") + UA_StatusCode_name(response.responseHeader.serviceResult);
    UA_BrowseResponse_clear(&response);
    return result;
  }
  for (size_t i = 0; i < response.resultsSize; ++i) {
    const UA_BrowseResult &browseResult = response.results[i];
    for (size_t j = 0; j < browseResult.referencesSize; ++j) {
      const UA_ReferenceDescription &ref = browseResult.references[j];
      BrowseEntry entry;
      entry.nodeId = fromNodeId(ref.nodeId.nodeId);
      entry.displayName = ref.displayName.text.length ?
                            std::string((const char *)ref.displayName.text.data, ref.displayName.text.length) :
                            entry.nodeId;
      entry.browseName =
        ref.browseName.name.length ? std::string((const char *)ref.browseName.name.data, ref.browseName.name.length) :
                                     entry.displayName;
      entry.hasChildren = ref.nodeClass == UA_NODECLASS_OBJECT || ref.nodeClass == UA_NODECLASS_VARIABLE;
      entry.isVariable = ref.nodeClass == UA_NODECLASS_VARIABLE;
      if (entry.isVariable) {
        Value value;
        if (read(entry.nodeId, value, error)) {
          entry.value = value;
          entry.dataType = Value::typeName(value.type());
        }
        error.clear();
      }
      result.push_back(entry);
    }
  }
  UA_BrowseResponse_clear(&response);
  return result;
}

bool WbOpcUaOpen62541Backend::read(const std::string &nodeId, Value &value, std::string &error) {
  error.clear();
  pumpServer();
  if (!mClient || !mClientConnected) {
    error = "not connected";
    return false;
  }
  UA_NodeId id = toNodeId(nodeId);
  UA_Variant variant;
  UA_Variant_init(&variant);
  const UA_StatusCode status = UA_Client_readValueAttribute(mClient, id, &variant);
  UA_NodeId_clear(&id);
  if (status != UA_STATUSCODE_GOOD) {
    error = std::string("read failed: ") + UA_StatusCode_name(status);
    return false;
  }
  const bool ok = fromVariant(variant, value);
  UA_Variant_clear(&variant);
  if (!ok)
    error = "unsupported value type";
  return ok;
}

bool WbOpcUaOpen62541Backend::write(const std::string &nodeId, const Value &value, std::string &error) {
  error.clear();
  pumpServer();
  if (!mClient || !mClientConnected) {
    error = "not connected";
    return false;
  }
  UA_Variant variant;
  if (!toVariant(value, value.type(), variant)) {
    error = "unsupported value type";
    return false;
  }
  UA_NodeId id = toNodeId(nodeId);
  const UA_StatusCode status = UA_Client_writeValueAttribute(mClient, id, &variant);
  UA_NodeId_clear(&id);
  UA_Variant_clear(&variant);
  if (status != UA_STATUSCODE_GOOD) {
    error = std::string("write failed: ") + UA_StatusCode_name(status);
    return false;
  }
  return true;
}

bool WbOpcUaOpen62541Backend::exposeVariable(const std::string &nodeId, const std::string &browseName, Value::Type type,
                                             const Value &initialValue, bool readOnly, std::string &error) {
  error.clear();
  if (!mServer) {
    error = "embedded server is not running";
    return false;
  }
  ExposedNode &exposed = mExposed[nodeId];
  const bool isNew = exposed.browseName.empty();
  exposed.type = type;
  exposed.browseName = browseName;
  exposed.readOnly = readOnly;
  if (!isNew) {
    // only the value is refreshed on re-exposure
    writeExposed(nodeId, initialValue, error);
    return error.empty();
  }

  UA_VariableAttributes attributes = UA_VariableAttributes_default;
  UA_Variant variant;
  if (!toVariant(initialValue, type, variant)) {
    error = "unsupported value type";
    return false;
  }
  attributes.value = variant;
  attributes.displayName = UA_LOCALIZEDTEXT_ALLOC("en", browseName.c_str());
  attributes.accessLevel = readOnly ? (UA_ACCESSLEVELMASK_READ) : (UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE);

  UA_NodeId id = toNodeId("ns=" + std::to_string((int)mNamespaceIndex) + ";s=" + nodeId);
  UA_QualifiedName browse = UA_QUALIFIEDNAME_ALLOC(mNamespaceIndex, browseName.c_str());
  const UA_StatusCode status =
    UA_Server_addVariableNode(mServer, id, UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), browse, UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                              attributes, NULL, NULL);
  UA_NodeId_clear(&id);
  UA_QualifiedName_clear(&browse);
  UA_NodeId_clear(&attributes.dataType);
  UA_LocalizedText_clear(&attributes.displayName);
  UA_Variant_clear(&variant);
  if (status != UA_STATUSCODE_GOOD) {
    error = std::string("cannot expose variable: ") + UA_StatusCode_name(status);
    return false;
  }
  pumpServer();
  return true;
}

bool WbOpcUaOpen62541Backend::unexposeVariable(const std::string &nodeId) {
  if (!mServer)
    return false;
  mExposed.erase(nodeId);
  UA_NodeId id = toNodeId("ns=" + std::to_string((int)mNamespaceIndex) + ";s=" + nodeId);
  const UA_StatusCode status = UA_Server_deleteNode(mServer, id, true);
  UA_NodeId_clear(&id);
  return status == UA_STATUSCODE_GOOD;
}

std::vector<BrowseEntry> WbOpcUaOpen62541Backend::listExposed(std::string &error) {
  std::vector<BrowseEntry> result;
  error.clear();
  if (!mServer) {
    error = "embedded server is not running";
    return result;
  }
  for (std::map<std::string, ExposedNode>::const_iterator it = mExposed.begin(); it != mExposed.end(); ++it) {
    BrowseEntry entry;
    entry.nodeId = it->first;
    entry.browseName = it->second.browseName;
    entry.displayName = it->second.browseName;
    entry.dataType = Value::typeName(it->second.type);
    entry.isVariable = true;
    readExposed(it->first, entry.value, error);
    error.clear();
    result.push_back(entry);
  }
  return result;
}

bool WbOpcUaOpen62541Backend::readExposed(const std::string &nodeId, Value &value, std::string &error) {
  error.clear();
  if (!mServer) {
    error = "embedded server is not running";
    return false;
  }
  pumpServer();
  UA_NodeId id = toNodeId("ns=" + std::to_string((int)mNamespaceIndex) + ";s=" + nodeId);
  UA_Variant variant;
  UA_Variant_init(&variant);
  const UA_StatusCode status = UA_Server_readValue(mServer, id, &variant);
  UA_NodeId_clear(&id);
  if (status != UA_STATUSCODE_GOOD) {
    error = std::string("read failed: ") + UA_StatusCode_name(status);
    return false;
  }
  const bool ok = fromVariant(variant, value);
  UA_Variant_clear(&variant);
  if (!ok)
    error = "unsupported value type";
  return ok;
}

bool WbOpcUaOpen62541Backend::writeExposed(const std::string &nodeId, const Value &value, std::string &error) {
  error.clear();
  if (!mServer) {
    error = "embedded server is not running";
    return false;
  }
  std::map<std::string, ExposedNode>::const_iterator it = mExposed.find(nodeId);
  const Value::Type type = it != mExposed.end() ? it->second.type : value.type();
  UA_Variant variant;
  if (!toVariant(value, type, variant)) {
    error = "unsupported value type";
    return false;
  }
  UA_NodeId id = toNodeId("ns=" + std::to_string((int)mNamespaceIndex) + ";s=" + nodeId);
  const UA_StatusCode status = UA_Server_writeValue(mServer, id, variant);
  UA_NodeId_clear(&id);
  UA_Variant_clear(&variant);
  pumpServer();
  if (status != UA_STATUSCODE_GOOD) {
    error = std::string("write failed: ") + UA_StatusCode_name(status);
    return false;
  }
  return true;
}

}  // namespace wbopcua

#endif  // WB_USE_OPEN62541
