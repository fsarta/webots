#!/usr/bin/env python3
# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimal OPC-UA demo server to try the Webots native OPC-UA module without a PLC.

Usage (Windows, macOS or Linux):
    pip install opcua
    python scripts/opcua_demo_server.py

It exposes a small 'PLC' address space with string node ids, matching the one
simulated by the Webots mock backend:
    ns=2;s=PLC.Setpoint   Double   writable  (drive it into a motor targetPosition)
    ns=2;s=PLC.Speed      Double   writable  (updated with a sine wave)
    ns=2;s=PLC.Running    Boolean  writable
    ns=2;s=PLC.Line.Counter  Int32 read-only (incremented every second)

Connect from Webots: Tools > OPC-UA I/O > Connect... with endpoint
    opc.tcp://localhost:4840
"""

import math
import sys
import time

try:
    from opcua import Server, ua
except ImportError:
    sys.exit("The 'opcua' package is required: pip install opcua")

ENDPOINT = "opc.tcp://0.0.0.0:4840"
NAMESPACE_URI = "urn:webots:opcua:demo"


def main():
    server = Server()
    server.set_endpoint(ENDPOINT)
    server.set_server_name("Webots OPC-UA demo PLC")

    index = server.register_namespace(NAMESPACE_URI)
    objects = server.get_objects_node()
    plc = objects.add_object(index, "PLC")

    def add(name, initial, writable=True):
        node = plc.add_variable(ua.NodeId(name, index), name, initial)
        node.set_writable() if writable else None
        print(f"  {node}  ({name})")
        return node

    print("Exposed variables (copy the 'ns=...;s=...' node ids into the Webots variable browser):")
    setpoint = add("PLC.Setpoint", 0.0)
    speed = add("PLC.Speed", 0.0)
    running = add("PLC.Running", False)
    counter = add("PLC.Line.Counter", 0, writable=False)

    server.start()
    print(f"OPC-UA demo server running on {ENDPOINT} - press Ctrl+C to stop")

    tick = 0
    try:
        while True:
            # simple simulated PLC behaviour
            speed.set_value(0.5 * math.sin(tick * 0.05))
            if running.get_value():
                counter.set_value(counter.get_value() + 1)
            tick += 1
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        print("stopped")


if __name__ == "__main__":
    main()
