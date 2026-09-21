# OPC-UA

The OPC-UA module maps Webots simulation values to OPC UA variables in both directions.
It is configured through mappings, edited with the **OPC-UA I/O** dock (`Tools > OPC-UA I/O`) and stored in `<world>.opcua.json` next to the world file.
See the [OPC-UA guide](../guide/opcua.md) for a walkthrough.

## Mapping File Format

The mapping file is a JSON document:

```
{
  "version": 1,
  "mappings": [ mapping, ... ]
}
```

Each *mapping* object supports the following keys:

| Key | Type | Default | Description |
| --- | ---- | ------- | ----------- |
| `id` | string | auto (`m1`, `m2`, ...) | unique mapping identifier, also used for the embedded server node id |
| `target` | string | | Webots target path (see below) |
| `nodeId` | string | | OPC UA node id, e.g. `ns=2;s=PLC1.DB1.Setpoint` |
| `browseName` | string | | display hint filled by the variable chooser |
| `direction` | `read` \| `write` \| `readWrite` | `read` | data flow: `read` = OPC -> Webots, `write` = Webots -> OPC |
| `dataType` | `Boolean` \| `Int32` \| `Int64` \| `Float` \| `Double` \| `String` | `Double` | OPC UA built-in type |
| `scale` | number | `1.0` | engineering conversion: `webotsValue = scale * opcValue + offset` |
| `offset` | number | `0.0` | conversion offset |
| `deadband` | number | `0.0` | updates smaller than this (in source units) are suppressed |
| `enabled` | bool | `true` | inactive mappings are not synchronized |

## Target Paths

| Syntax | Description |
| ------ | ----------- |
| `device:<robot>/<device>/<tag>` | device runtime values; `<robot>` matches the robot `name` or `DEF`; `<device>` matches the `deviceName`; tags: `targetPosition` (rw, Motor), `position`/`value` (r, PositionSensor), `velocity` (r, Motor), `torque` (r, RotationalMotor), `force` (r, LinearMotor) |
| `field:<DEF>/<field>[/<field>][.x\|.y\|.z]` | node field values; the first element matches the `DEF` name of a node; single-value fields (`SFBool`, `SFInt32`, `SFDouble`, `SFString`) are mapped as-is; `SFVector3` fields need a `.x`, `.y` or `.z` suffix |

## Embedded Server Address Space

When the embedded server runs (enabled in the connection dialog), every enabled mapping is exposed as an OPC UA variable under the `urn:webots:opcua` namespace:

| Property | Value |
| -------- | ----- |
| Node id | `ns=2;s=Webots.<mapping id>` |
| Browse name | the mapping `browseName` (or the target path) |
| Data type | the mapping `dataType` |
| Access level | read-only for `write` mappings, read/write otherwise |

External OPC UA clients may write `read`/`readWrite` variables to drive the simulation; values written by Webots are visible to all connected clients.

## Runtime

- Synchronization runs at 20 Hz from the GUI process.
- Every mapping reports a quality string in the dock: `good`, `bad: <reason>` (unresolved target, type mismatch, connection error) or `-` (idle).
- Mapping edits are applied immediately; use **Save** in the dock to persist them.

## API Summary (C++)

| Class | Role |
| ----- | ---- |
| `WbOpcUaManager` | singleton: connection lifecycle, mapping store, synchronization loop |
| `WbOpcUaTarget` | resolves `device:`/`field:` target paths against the live node tree |
| `wbopcua::MappingStore` | mapping collection with JSON serialization and validation |
| `wbopcua::Backend` | transport-neutral endpoint interface |
| `wbopcua::WbOpcUaOpen62541Backend` | open62541 implementation (client + embedded server) |
| `wbopcua::MockBackend` | in-memory implementation used by tests and as fallback |
