# Interfacing Webots with OPC-UA

Webots embeds a native OPC-UA module to connect simulations with industrial automation systems (PLC, SCADA, digital twins) with no external glue code.
Both directions are supported:

- **Client**: Webots connects to a remote OPC-UA server (e.g. a PLC) and lets you **choose variables directly in the remote address space** and bind them to simulation targets (joint setpoints, sensor values, node fields).
- **Server**: Webots exposes an embedded OPC-UA server publishing the mapped simulation variables to the factory network.

The module is built on the [open62541](https://open62541.org) stack when available (see below); otherwise an in-memory mock backend is used (useful for testing the workflow without a PLC).

## Building with the native stack

Run the installer before building Webots:

```bash
scripts/install/open62541_installer.sh
```

It builds `open62541` and installs the amalgamated header into `include/open62541/` and the static library into `lib/webots/`.
The `src/webots/Makefile` detects them automatically (`WB_USE_OPEN62541`) and links the native stack.
On Linux and macOS the dependency is also part of the `dependencies` Makefiles (`make open62541`).

## The OPC-UA I/O Window

Open it from the **Tools** menu: `Tools > OPC-UA I/O...`. The OPC-UA section lives in its own window, so the mapping table and the variable chooser get all the room they need.

1. **Connect...** opens the connection dialog: endpoint URL (`opc.tcp://plc:4840`), security policy, anonymous or user/password authentication, and the embedded server options (port, activation). The **Test connection** button validates the endpoint before committing.
2. **Browse variables...** opens the variable chooser: the remote address space is presented as a lazy tree with name, node id, type, live value and description columns. Search variables by name, multi-select them and press **Bind selected variables** to create mappings.
3. The **mapping table** shows all bindings with their direction, live value and quality (`good` or `bad: <reason>`). Mappings can be removed here.

## Mapping Targets

Each mapping binds a Webots target to an OPC UA node id (`ns=2;s=PLC1.DB1.Setpoint`).
Supported target syntaxes:

| Syntax | Description | Example |
| ------ | ----------- | ------- |
| `device:<robot>/<device>/<tag>` | motor/sensor runtime values | `device:MyRobot/motor1/targetPosition` |
| `field:<DEF>/<field>[/<field>][.x\|.y\|.z]` | node field values | `field:SHOULDER/HingeJointParameters/minStop` |

Device tags:

| Tag | Access | Device |
| --- | ------ | ------ |
| `targetPosition` | read/write | Motor |
| `position`, `value` | read | PositionSensor |
| `velocity` | read | Motor |
| `torque` | read | RotationalMotor |
| `force` | read | LinearMotor |

Mapping directions:

- **read** (`OPC -> Webots`): the OPC variable drives the Webots target (e.g. a PLC setpoint drives a motor).
- **write** (`Webots -> OPC`): the Webots target is published on the OPC side (e.g. a sensor value).
- **readWrite**: both directions, last change wins.

Each mapping supports engineering conversions: `webotsValue = scale * opcValue + offset`, and a `deadband` to suppress noisy updates.

## Mapping Files

Mappings are stored next to the world file as `<world>.opcua.json` and can be loaded/saved from the window:

```json
{
  "version": 1,
  "mappings": [
    {
      "id": "m1",
      "target": "device:MyRobot/motor1/targetPosition",
      "nodeId": "ns=2;s=PLC1.DB1.Setpoint",
      "browseName": "Setpoint",
      "direction": "read",
      "dataType": "Double",
      "scale": 0.001,
      "offset": 0.0,
      "deadband": 0.0,
      "enabled": true
    }
  ]
}
```

The complete format is documented in the [OPC-UA reference](../reference/opcua.md).

## Embedded Server

When the embedded server is enabled at connection time, every mapping is also exposed under the `urn:webots:opcua` namespace as `ns=2;s=Webots.<mapping id>` (readable/writable according to the mapping direction), so external OPC UA clients can browse and consume the simulation variables natively.

## Security

The connection dialog supports the `None`, `Basic256Sha256`, `Basic128Rsa15` and `Aes128_Sha256_RsaOaep` security policies and username/password authentication.
Certificate management for encrypted endpoints uses the open62541 application options (see the open62541 documentation for PKI setup).

## Trying the Workflow without a PLC

Two zero-hardware options are available:

1. **Mock backend** (no installation): when Webots is built *without* the open62541 stack, the OPC-UA window transparently uses an in-memory backend which accepts any endpoint and simulates a demo PLC (`ns=1;s=PLC.Setpoint`, `ns=1;s=PLC.Speed`, `ns=1;s=PLC.Running`, `ns=1;s=PLC.Line.Counter`). Ideal to explore the connection dialog and the variable chooser.
2. **Demo server + native stack**: run the bundled demo PLC on any machine with Python:

   ```
   pip install opcua
   python scripts/opcua_demo_server.py
   ```

   It exposes `ns=2;s=PLC.Setpoint` (writable), `ns=2;s=PLC.Speed`, `ns=2;s=PLC.Running` and `ns=2;s=PLC.Line.Counter` on `opc.tcp://localhost:4840`, printing the exact node ids at startup. Connect from Webots and bind, for example:

   | Webots target | OPC UA node id | direction | scale |
   | ------------- | -------------- | --------- | ----- |
   | `device:four_bar/crank_motor/targetPosition` | `ns=2;s=PLC.Setpoint` | read | 1 |
   | `device:four_bar/crank_sensor/position` | `ns=2;s=PLC.Speed` | write | 1 |

   together with the four-bar demo world (`tests/manual_tests/worlds/mechanism_four_bar_linkage.wbt`, see the [Robot Creator](robot-creator.md#demo-world)) this gives the complete digital-twin loop: drive the motor from the PLC side and publish the sensor value back.

## Building on Windows

On Windows the whole toolchain runs in the **MSYS2 MinGW64** shell (see the build instructions of the wiki). For the native OPC-UA stack additionally install:

```
pacman -S git mingw-w64-x86_64-cmake
```

before running `make release` (the `open62541` dependency step is skipped automatically — with a warning — when `git`/`cmake` are missing, and the mock backend is used instead).
