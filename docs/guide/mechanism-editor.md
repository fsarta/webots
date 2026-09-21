# The Mechanism Editor

The Mechanism Editor is a CAD-like tool (inspired by the joint workflows of Fusion 360) for creating and managing the joints and links of a robot or mechanism graphically, without editing the [Scene Tree](the-scene-tree.md) by hand.
It fully supports closed kinematic chains (parallel mechanisms, five-bar linkages, slider-cranks, etc.) through `SolidReference` endpoints.

## Opening the Editor

The Mechanism Editor is available from the **Tools** menu: `Tools > Mechanism Editor`.
It appears as a dockable widget with:

- A **2D schematic canvas** of the kinematic structure: links are blocks (grounded links in blue), joints are glyphs on the edges (red circle = hinge, blue circle = hinge2, green diamond = joint closing a kinematic loop).
- A **toolbar** with the editing tools.
- A **property panel** to edit the selected joint or link.
- A **mobility report** computed with the Grübler–Kutzbach criterion.

## Editing Tools

- **Select**: click links and joints to select them (the selection is mirrored in the Scene Tree and the 3D view); drag links to rearrange the diagram.
- **Joint**: click a parent link and then a child link to connect the two existing links with a new joint. When the two links are already connected through the tree, the new joint closes a **kinematic loop** and its `endPoint` is created as a `SolidReference`.
- **Close Loop**: same as the Joint tool, with explicit loop semantics: Webots warns you if the two links are not connected yet (the joint will then simply join two sub-mechanisms).
- **Delete**: click a joint to remove it.
- **Add Link + Joint**: creates a new link attached to the currently selected link with a brand new joint (hinge, hinge2, slider or ball, chosen in the toolbar combo box).

Topology changes (add/delete joints and links) require the simulation to be paused, because they restructure the live node tree.

## Joint Properties

When a joint is selected, the property panel edits the underlying `jointParameters` fields (undoable like the Scene Tree editors):

- **Axis**: the joint axis expressed in the parent link frame.
- **Anchor**: the anchor point (hinge and ball joints) expressed in the parent link frame.
- **Stops**: `minStop` and `maxStop`.
- **springConstant** and **dampingConstant**.

## 3D Manipulators

With **3D handles** enabled, selecting a joint shows Fusion-style manipulators directly in the 3D view:

- an orange **anchor cross** which can be dragged to reposition the anchor,
- an **axis arrow** whose tip can be dragged to reorient the axis.

The edits are undoable (Ctrl+Z) and mirrored in the property panel and in the Scene Tree.

## Closed Kinematic Chains

Webots represents closed loops with a `Joint` whose `endPoint` is a `SolidReference` to an already existing solid, instead of an inline `Solid`:

```
HingeJoint {
  jointParameters HingeJointParameters { axis 0 1 0 }
  endPoint SolidReference { solidName "LINK_4" }
}
```

The Mechanism Editor creates these connections for you: just use the **Joint** or **Close Loop** tool between two links that already belong to the mechanism.
The mobility report then shows the fundamental loops and validates them:

- **Mechanism with N DOF**: the expected result for a well designed mechanism.
- **Rigid structure (mobility 0)**: the mechanism cannot move.
- **Over-constrained**: redundant constraints (e.g. two rigid joints between the same links).
- Warnings are displayed for loops which have fewer joint DOF than the generic loop-closure requirement (spatial: 6, planar: 3): such loops (e.g. parallelograms) need a special geometric alignment and are simulated as soft constraints.

Use the **Planar** checkbox to switch the analysis between planar (3-DOF) and spatial (6-DOF) formulations.

## Tips

- Give every link a unique `name`: `SolidReference` endpoints resolve links by name.
- The **Auto Layout** and **Reset Layout** buttons recompute the diagram layout.
- The editor works on any mechanism found in the world, including non-robot mechanisms.
