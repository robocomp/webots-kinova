# webots-kinova

Webots scene + RoboComp bridge for a fixed **Kinova Gen3 7‑DoF arm with
Robotiq 2F‑85 gripper** mounted on a table. Pairs with the active-inference
arm controller at
[`robocomp/active_inference/kinova_controller`](https://github.com/robocomp/active_inference).

Spawned from [`SergioEslava-UEx/webots-p3bot`](https://github.com/SergioEslava-UEx/webots-p3bot)
as a single-arm Kinova fork; the original mobile-robot bridge is preserved.

## Layout

```
worlds/
  arm_table.wbt              # Kinova-on-table scene used by the controller
  …                          # other inherited p3bot scenes (legacy)
protos/
  KinovaGen3.proto           # the arm
  WaterBottleOpaque.proto    # opaque drop-in for Cyberbotics' WaterBottle
                             # so the bottle casts shadows in the renderer
  kinova_arm_meshes/         # STL meshes used both by Webots and by the
                             # active-inference viewer (ArmBeliefViewer3D
                             # in kinova_controller reads from here)
  …
components/
  kinova-bridge/             # RoboComp bridge: Webots ↔ Ice/CORTEX
    KinovaBridge.cdsl        # interfaces (KinovaArm, Camera360RGB, Lidar3D,
                             #             CameraRGBDSimple, OmniRobot,
                             #             Webots2Robocomp, …)
    src/specificworker.{h,cpp}
```

## Build the bridge

```bash
cd components/kinova-bridge
cmake -B build && make -C build -j12
```

Binary lands in `bin/KinovaBridge`. Run with:

```bash
bin/KinovaBridge etc/config
```

`etc/config` declares the Ice endpoints (`KinovaArm = "tcp -p 15005"`,
`Webots2Robocomp = "tcp -p 15010"`, …); the `kinova_controller` agent's
`Proxies.*` need to match these.

## Webots side

Open `worlds/arm_table.wbt`. The `P3Bot` `Robot` node is the supervisor that
hosts the `KinovaGen3` PROTO (controller `<extern>`, so the bridge process
above drives it). The bottle uses `WaterBottleOpaque` rather than upstream
`WaterBottle` because Webots' shadow pass skips transparent geometry — see
[Why opaque?](#why-opaque) below.

## Webots2Robocomp DEF resolution

`Webots2Robocomp_getObjectPose("name")` first tries `getFromDef(name)` and,
on a miss, falls back to a one-shot scan of the world root's top-level
`children` looking for a node whose `SFString name` field matches. This
means world authors don't have to remember to add a `DEF` to every scene
object they want supervised — setting the `name` field is enough.

Resolved nodes are cached (`Node*` pointers are stable in Webots).

## Why opaque?

The upstream `WaterBottle` PROTO uses a textured `IndexedFaceSet` whose
appearance pipeline makes the Webots shadow renderer skip it, so the bottle
floats over the desk with no shadow. `WaterBottleOpaque` is a tiny PROTO
(opaque `PBRAppearance` + `Cylinder` geometry, same interface fields:
`translation`, `rotation`, `name`, `mass`, plus `color`, `radius`, `height`)
that drops in cleanly and casts a real shadow.

## Active-inference workflow

In tandem with [`kinova_controller`](https://github.com/robocomp/active_inference):

1. Launch Webots and open `worlds/arm_table.wbt`.
2. Start `bin/KinovaBridge etc/config` from this repo.
3. Start `kinova_controller`'s agent; the side-grasp loop hits the bottle's
   live world pose via `Webots2Robocomp_getObjectPose("bottle")`.

## Status

Active development. Bridge interfaces declared but several (Lidar3D,
Camera360RGB, JoystickAdapter) are still scaffolds inherited from
`kinova-bridge`; the Kinova side (KinovaArm + Webots2Robocomp) is the path
currently exercised by `kinova_controller`.
