# The Robot Creator

The Robot Creator is a dedicated window to build Webots robots starting from **mesh files (STL, OBJ)** and placing their joints on **CAD-style snap points** — like Fusion 360: endpoint, midpoint, circle center, median and grid snaps.

Open it from the **Tools** menu: `Tools > Robot Creator...`. It is a standalone window (not integrated in the main view, so nothing gets squeezed): it keeps the whole viewport for your geometry.

## Workflow

1. **Import the link meshes** with `File > Import Mesh...` (STL ascii/binary or OBJ). Each mesh becomes a link of the robot; use `File > Add Empty Link` for links without geometry. The parts are spread on the work plane so everything is visible.
2. **Insert the joints** with the `Joint` menu (`Shift+H` hinge, `Shift+S` slider, `Shift+2` hinge2, `Shift+B` ball, `Shift+F` fixed): the cursor becomes a CAD crosshair — every snap feature in range lights up; **click on the snap point** to set the joint anchor there. The joint axis is inferred from the snap (circle normal for `Circle Center`, face normal for surface picks) and can be refined in the properties panel.
3. **Wire the kinematics** in the properties panel: set each joint's *parent link* and *child link*. Joints with no child can be completed later.
4. **Validate**: the Assembly panel shows the live **Grübler–Kutzbach mobility** of the mechanism (planar or spatial) and reports **closed kinematic chains**.
5. **Export**: `File > Export Robot...` (or `Copy .wbt to Clipboard`) generates a Webots `Robot { ... }` text — tree joints are nested per link, and every joint that closes a loop is written with a `SolidReference` endpoint, i.e. a **closed kinematic chain** Webots can simulate.

The exported poses follow Webots' semantics: child link poses are relative to their parent link, joint anchors are in the parent link frame and joint axes in the child link frame. Copy the mesh files next to the world so the `Mesh { url "..." }` nodes resolve.

## CAD Snap Modes

Toggle the snap kinds in the `Snap` menu:

| Snap | Shortcut | Picks | Joint axis inference |
| ---- | -------- | ----- | -------------------- |
| **Endpoints** | `F1` | mesh vertices (point fine) | — |
| **Midpoints** | `F2` | edge midpoints (punto medio) | — |
| **Circle Centers** | `F3` | centers of circular edge loops — holes, rims (centro del cerchio) | circle normal (ideal for hinges) |
| **Centroids** | `F4` | triangle face centroids (mediana) | face normal on surface picks |
| **Grid** | `F5` | the z=0 work plane grid | Z axis |

`F6`/`F7` enable/disable all snaps at once.

Selection policy, as in Fusion 360: the **nearest distance band** around the cursor wins (14 px); inside the same 8 px band the snap **priority** wins — endpoint, then circle center, midpoint, centroid and finally grid. `Tab` cycles through every candidate in range (the status bar shows *candidate n of N*), `Esc` cancels the joint placement.

## Camera

- `Right-drag` — orbit
- `Middle-drag` — zoom (`Shift+Middle-drag` — pan)
- `Wheel` — zoom
- `Home` (or `View > Reset Camera`) — frame the assembly

`View` also toggles the grid and the snap feature markers.

## Closed Kinematic Chains

A joint whose child link is **already connected** to the mechanism closes a loop: the Robot Creator automatically classifies it as a loop-closing joint and the export writes

```
endPoint SolidReference { solidName "LINK_NAME" }
```

The mobility analysis reports the fundamental loops and warns when a loop has fewer joint DOF than its generic requirement (like a planar four-bar: it needs a special geometric alignment — parallel hinge axes — and is handled by the physics engine as a soft constraint).

## Demo World

A ready-to-use planar four-bar linkage with a closed kinematic chain ships with the tests:

```
tests/manual_tests/worlds/mechanism_four_bar_linkage.wbt
```

Open it with `File > Open World...` to see the expected result of the workflow (links `CRANK`/`COUPLER`/`ROCKER`, tree joints `J0`/`J1`/`J2` and the loop-closing joint `J3` with `endPoint SolidReference`). The world also contains a `crank_motor` / `crank_sensor` device pair, handy for the [OPC-UA tutorial](opcua.md).

## Tips

- Give every link a unique `name`: `SolidReference` endpoints resolve links by name.
- For hinges on holes, pick the **Circle Center** snap directly on the hole rim: the anchor lands on the true center and the axis follows the hole axis.
- Place multiple meshes (one per link) and use `translation` spin boxes in the properties panel for coarse positioning before snapping the joints.
