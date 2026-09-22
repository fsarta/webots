# Standalone unit tests for the pure-logic Webots modules (no Qt required):
#   - mechanism kinematic graph analysis (closed kinematic chains)
#   - OPC-UA core (values, JSON mapping files, mock backend)
#   - Robot Creator cores (mesh loading, CAD snaps, robot export)
#
# Usage:
#   make        # build and run all tests
#   make clean

WEBOTS_SRC = ../../src/webots
CXX ?= g++
CXXFLAGS += -std=c++17 -Wall -Wextra -g -I$(WEBOTS_SRC)/mechanism -I$(WEBOTS_SRC)/opcua -I$(WEBOTS_SRC)/robotcreator

MECHANISM_SOURCES = $(WEBOTS_SRC)/mechanism/WbKinematicGraphCore.cpp
OPCUA_SOURCES = $(WEBOTS_SRC)/opcua/WbOpcUaValue.cpp $(WEBOTS_SRC)/opcua/WbOpcUaJson.cpp \
                $(WEBOTS_SRC)/opcua/WbOpcUaMapping.cpp $(WEBOTS_SRC)/opcua/WbOpcUaMockBackend.cpp
ROBOTCREATOR_SOURCES = $(MECHANISM_SOURCES) $(WEBOTS_SRC)/robotcreator/WbMeshCore.cpp \
                $(WEBOTS_SRC)/robotcreator/WbSnapCore.cpp $(WEBOTS_SRC)/robotcreator/WbRobotProtoCore.cpp

BUILD_DIR = build

.PHONY: all test clean

all: test

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/test_kinematic_graph: mechanism/test_kinematic_graph.cpp $(MECHANISM_SOURCES) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD_DIR)/test_opcua_core: opcua/test_opcua_core.cpp $(OPCUA_SOURCES) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD_DIR)/test_mesh_snap: robotcreator/test_mesh_snap.cpp $(ROBOTCREATOR_SOURCES) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

test: $(BUILD_DIR)/test_kinematic_graph $(BUILD_DIR)/test_opcua_core $(BUILD_DIR)/test_mesh_snap
	$(BUILD_DIR)/test_kinematic_graph
	$(BUILD_DIR)/test_opcua_core
	$(BUILD_DIR)/test_mesh_snap

clean:
	rm -rf $(BUILD_DIR)
