# Standalone unit tests for the pure-logic Webots modules (no Qt required):
#   - mechanism kinematic graph analysis (closed kinematic chains)
#   - OPC-UA core (values, JSON mapping files, mock backend)
#
# Usage:
#   make        # build and run all tests
#   make clean

WEBOTS_SRC = ../../src/webots
CXX ?= g++
CXXFLAGS += -std=c++17 -Wall -Wextra -g -I$(WEBOTS_SRC)/mechanism -I$(WEBOTS_SRC)/opcua

MECHANISM_SOURCES = $(WEBOTS_SRC)/mechanism/WbKinematicGraphCore.cpp
OPCUA_SOURCES = $(WEBOTS_SRC)/opcua/WbOpcUaValue.cpp $(WEBOTS_SRC)/opcua/WbOpcUaJson.cpp \
                $(WEBOTS_SRC)/opcua/WbOpcUaMapping.cpp $(WEBOTS_SRC)/opcua/WbOpcUaMockBackend.cpp

BUILD_DIR = build

.PHONY: all test clean

all: test

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/test_kinematic_graph: mechanism/test_kinematic_graph.cpp $(MECHANISM_SOURCES) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

$(BUILD_DIR)/test_opcua_core: opcua/test_opcua_core.cpp $(OPCUA_SOURCES) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $^

test: $(BUILD_DIR)/test_kinematic_graph $(BUILD_DIR)/test_opcua_core
	$(BUILD_DIR)/test_kinematic_graph
	$(BUILD_DIR)/test_opcua_core

clean:
	rm -rf $(BUILD_DIR)
