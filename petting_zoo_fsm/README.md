# petting_zoo_fsm

The state machine (`fsm_node`) that decides what the robot is doing. It does not move the robot itself. Instead, it watches the detector outputs and switches the motion nodes in `petting_zoo_navigation` on and off through their latched `<node>/enable` topics.

It is started by `petting_zoo_navigation/launch/petting_zoo.launch.xml`. You can also run it on its own:

```bash
ros2 run petting_zoo_fsm fsm_node
```


| State | Walker | Arm tracking | Gripper | What it waits for |
|---|---|---|---|---|
| `searching` | on | off (arm returns to start pose) | — | A human detection, once any cooldown has ended |
| `tracking` | off | on, follows the human | — | An apple in the human's hand, the human being lost, or `max_track_time` running out |
| `apple` | off | on, see below | open | The human or the apple being lost. Ignored while the liked apple is being dropped |

In the `apple` state, the arm target changes with what is seen in the human's hand:

- **Nothing steady yet:** the arm keeps following the human.
- **Steady liked apple:** the arm follows the liked apple (`arm_track_controller/apple_colour = "liked"`).
- **Steady disliked apple:** the arm holds still (`"hold"`). If both are steady, the liked apple wins. Meanwhile `gripper_controller` closes the gripper on its own disliked-apple detections.

Which colour is liked is set with the `liked_apple` launch argument (`red` by default). The launch file remaps `detections/apples/red` to the liked colour and `detections/apples/green` to the other one.

**Dropping the liked apple:** when the button closes the gripper on the liked apple, `gripper_controller` moves the arm aside and opens the gripper again (see `petting_zoo_navigation`). For the whole sequence it publishes `true` on `gripper_controller/dropping`. While that is true, the `apple` state does nothing: no timeouts, no state change and no arm target change, so the arm can't swing back before the apple is dropped. Once the gripper has opened it publishes `false`, and the normal checks resume. Usually the apple is gone by then, so the FSM moves to `tracking`.

When the robot leaves `apple`, the FSM closes the gripper and switches the arm back to following humans.

An apple only counts as being "in hand" if **all** of these are true:

1. A human was detected less than 0.5 s ago.
2. The apple's centre lies inside the largest human box. The box is widened ×1.3 horizontally, because hands stick out.
3. For the arm to react to the apple, the apple must also be **steady**: `apple_track_frames` detections in a row, each less than 0.5 s apart, with the last one less than 0.5 s ago.

## Topics

**Subscribes**
- `detections/humans` (`vision_msgs/BoundingBox2DArray`)
- `detections/apples/red`, `detections/apples/green` (`vision_msgs/BoundingBox2DArray`): in the code these are the liked and disliked apple. The launch file remaps them to the real colours.
- `gripper_controller/dropping` (`std_msgs/Bool`, `transient_local`): `true` while the liked apple is being dropped

**Publishes** (all `transient_local`, so a node that starts late still gets the latest value)
- `walking_around/enable` (`std_msgs/Bool`)
- `arm_track_controller/enable` (`std_msgs/Bool`)
- `arm_track_controller/apple_colour` (`std_msgs/String`): `""` = human, `"liked"`, or `"hold"`
- `gripper_controller/enable` (`std_msgs/Bool`)

## Parameters

There are **no ROS parameters**. All timings are hard-coded members in [include/petting_zoo_fsm/fsm_node.hpp](include/petting_zoo_fsm/fsm_node.hpp). Change them there and rebuild.

| Constant | Default | Meaning |
|---|---|---|
| `lost_timeout_` | `10.0` s | Time without any human detection before `tracking`/`apple` gives up and returns to `searching` |
| `max_track_time_` | `30.0` s | Maximum time in `tracking` (without an apple) before moving on to look for someone else |
| `cooldown_` | `10.0` s | After `max_track_time_` runs out, human detections are ignored for this long, so the robot can walk away |
| `apple_lost_timeout_` | `3.0` s | Time without an in-hand apple before `apple` drops back to `tracking` |
| `apple_track_frames_` | `3` | In-hand detections in a row before the arm switches to the liked apple or holds still (disliked) |

Values in the source files:

| Where | Value | Meaning |
|---|---|---|
| `fsm_node.cpp` timer | 100 ms | FSM tick rate |
| `appleCallback` | 0.5 s | A human must have been seen this recently for an apple to count |
| `appleCallback` / `AppleState` | 0.5 s | Maximum gap between apple detections before the streak resets (the detector runs at about 5 Hz) |
| `insideHuman` | ×1.3 | Horizontal widening of the human box when checking whether an apple is in hand |

## Tuning

- **Robot gives up on a person too quickly**, e.g. when they turn sideways and YOLO drops them for a moment: raise `lost_timeout_`.
- **Robot stands still too long after someone has left:** lower `lost_timeout_`.
- **Robot stays with one person too long:** lower `max_track_time_`.
- **Robot immediately re-locks onto the same person after moving on:** raise `cooldown_`. It must be long enough for the walker to turn and drive away.
- **Robot leaves the apple state while the apple is still being held:** the apple detector is losing it between frames. Raise `apple_lost_timeout_`, or tune the apple detector first (see `mirte_apple_detector`).
- **Arm jumps to objects of the liked colour that aren't apples:** raise `apple_track_frames_`. Each extra frame adds about 0.2 s of delay at 5 Hz.
- **Apples held out at arm's length aren't counted as in hand:** increase the 1.3 widening in `insideHuman`.
- **Apples are never counted as in hand:** check that the human detector and the apple detector both see the scene at the same time. An apple is ignored unless a human was detected in the last 0.5 s.
- **Robot stays in the apple state forever:** `gripper_controller/dropping` is stuck at `true`, usually because `gripper_controller` crashed during a drop. Check with `ros2 topic echo --qos-durability transient_local /gripper_controller/dropping` and restart the launch.
- If you lower the detectors' `max_rate`, also raise the 0.5 s gap thresholds. Otherwise streaks will never build up.

Watch the transitions live with `ros2 run rqt_console rqt_console`, or just read the `fsm_node` output. Every transition is logged with its reason.
