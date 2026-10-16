# petting_zoo_navigation

Motion nodes for the MIRTE Master petting zoo: random-walk with obstacle avoidance, arm tracking of a human/apple, and gripper control. Every node is idle until the FSM (`petting_zoo_fsm`) enables it on its `<node>/enable` topic.

| Node | What it does | Enable topic |
|---|---|---|
| `walking_around` | Random walk, avoids obstacles **ahead** | `walking_around/enable` |
| `walking_around_field` | Random walk, avoids obstacles **all around** (repulsive field) | `walking_around/enable` |
| `arm_track_controller` | Pans/tilts the arm so the camera keeps the target centred | `arm_track_controller/enable` |
| `gripper_controller` | Opens the gripper, closes it on its own for the disliked apple, or with the button for the liked one and then drops it to the side | `gripper_controller/enable` |

Only one of the two walkers runs at a time (picked with the `walker` launch argument). Both listen on the same enable topic.

## Launching

```bash
ros2 launch petting_zoo_navigation petting_zoo.launch.xml
```

### Launch arguments

`petting_zoo.launch.xml`

| Argument | Default | Meaning |
|---|---|---|
| `use_sim_time` | `false` | Use `/clock` instead of wall time |
| `cmd_vel_topic` | `/mirte_base_controller/cmd_vel` | Base velocity command topic |
| `human_detector` | `yolo` | `yolo` or `hog` |
| `camera_topic` | `/gripper_camera/image_raw` | Raw image topic, only used by the HOG detector |
| `front_offset` | `1.5708` | Lidar angle (rad) that points to the robot's front. Real robot: `1.5708` |
| `flip_scan` | `true` | Real lidar is mirrored (angles run clockwise) |
| `walker` | `front` | `front` = `walking_around`, `field` = `walking_around_field` |
| `counter_rotate` | `true` | Turn the arm against the base rotation, so the camera stays on target while the base turns |
| `liked_apple` | `red` | Which apple the robot likes: `red` or `green`. The other one is disliked. The nodes' `detections/apples/red` and `detections/apples/green` subscriptions are remapped to the liked and disliked colour, so inside the code "red" means liked and "green" means disliked |
| `navigation` | `true` | `false` skips the walker, so the base never drives (for testing detection, the arm and the gripper) |

---

## Parameters

### Shared lidar / robot-shape parameters (both walkers)

These describe how the lidar is mounted and how big the robot is. Get them right **before** tuning anything else.

| Parameter | Default | Meaning |
|---|---|---|
| `front_offset` | `π/2` | Scan angle (rad) that points to the robot's front. Subtracted from every scan angle. |
| `flip_scan` | `false` | Set `true` if the lidar's angles run clockwise. Negates the angle so +angle = left. |
| `min_range` | `0.15` m | Lidar returns closer than this are ignored (robot's own body, cables, the arm). |
| `robot_half_width` | `0.19` m | Half the robot width plus a margin. Points with `|y|` below this, in front of the robot, count as "in the path". |
| `arm_reach` | `0.35` m | How far the lowered arm sticks out ahead of the lidar. Added to the stop distance so the **arm tip**, not the lidar, keeps clearance. |
| `stop_margin` | `0.15` m | Free space to keep between the arm tip and an obstacle. Stop distance = `arm_reach + stop_margin`. |
| `slow_margin` | `0.30` m | Start slowing down this far before the stop distance. Speed scales linearly from 100% to 30% across this band. |

How to tune:

1. **`front_offset` / `flip_scan`**: put an object directly in front of the robot and echo the scan (or view it in RViz). Pick the offset that makes that object appear at angle 0. Then put the object on the robot's **left**. If the robot turns towards it when avoiding, flip `flip_scan`.
2. **`min_range`**: if the robot stops or turns for no reason, it is probably seeing itself. Raise this until the phantom readings disappear. Keep it below the smallest real distance you care about.
3. **`robot_half_width`**: measure half the base width and add 2–5 cm. If it clips table legs or door frames when driving past them, increase it. If it refuses narrow gaps it could fit through, decrease it.
4. **`arm_reach`**: measure it on the real robot with the arm in its driving pose. This is the most important one to get right, because if it is too small the arm will hit things.
5. **`stop_margin` / `slow_margin`**: if the robot stops too close, increase `stop_margin`. If it brakes hard or jerks near obstacles, increase `slow_margin`. If it is too hesitant in cluttered spaces, reduce both.

### `walking_around` only

| Parameter | Default | Meaning |
|---|---|---|
| `side_margin` | `0.20` m | Minimum gap beside the robot/arm. The random turn is cancelled if it would steer towards a side closer than this. |

Behaviour: drives at 0.1 m/s. Every 5 s it picks a random turn rate in ±0.3 rad/s, which lasts 2 s before it straightens out again. If something is within the stop distance ahead, it turns in place at 1 rad/s towards the side with more room. It keeps turning until the front is clear by 10 cm past the stop distance. This hysteresis stops it from stuttering.

Tuning `side_margin`: raise it if the robot scrapes along walls while wandering. Lower it if it hardly ever turns in narrow corridors.

### `walking_around_field` only

| Parameter | Default | Meaning |
|---|---|---|
| `turn_speed` | `1.0` rad/s | Turn rate when spinning in place away from a blocked front |
| `influence_radius` | `0.6` m | Obstacles closer than this (in any direction) push the robot away |
| `rep_gain` | `1.5` | Strength of the push compared to "drive forward" (which has a weight of 1) |
| `max_ang` | `1.0` rad/s | Maximum steering rate produced by the field |

Behaviour: each lidar point inside `influence_radius` contributes a push away from itself, with weight `(influence_radius − r) / influence_radius`. The sum of the pushes sets the direction of the push. The closest point alone sets its strength, so a long wall doesn't count more than a single post. The robot steers towards `forward + rep_gain · push` with `ω = clamp(2 · heading, ±max_ang)`. Forward speed is multiplied by `cos(heading)`, so it slows down during sharp turns. The front safety stop from the shared parameters still applies on top of this.

How to tune:

- **Robot reacts too late or brushes past obstacles:** increase `influence_radius` first, then `rep_gain`.
- **Robot wobbles or zig-zags in corridors:** decrease `rep_gain`, or decrease `influence_radius` so both walls aren't pushing it all the time.
- **Robot hardly moves in clutter:** decrease `influence_radius`. Speed drops with `cos(heading)`, so a strong field can stall it.
- **Steering too jerky:** lower `max_ang`. **Steering too sluggish:** raise it.
- **`turn_speed`**: lower it if the robot overshoots and spins past open space. Raise it if escaping corners takes too long.

### `gripper_controller`

| Parameter | Default | Meaning |
|---|---|---|
| `open_position` | `-0.3` | Gripper command position when open |
| `closed_position` | `0.1` | Gripper command position when closed |
| `disliked_frames` | `3` | Disliked-apple detections in a row (each less than 0.5 s apart) needed before closing |
| `disliked_lost_timeout` | `1.0` s | Time without the disliked apple before the gripper opens again |
| `liked_memory` | `20.0` s | The button only closes the gripper if the liked apple was seen within this time |
| `button_pressed_value` | `false` | Button reading that counts as pressed |
| `drop_delay` | `2.0` s | After a button close, time before the arm moves aside |
| `aside_time` | `2.0` s | Time the arm gets to move aside before the gripper opens and drops the apple |

The gripper opens when the node is enabled (the FSM enters `apple`) and closes when it is disabled (the FSM leaves `apple`). While enabled:

- **Disliked apple:** it closes on its own once the apple has been seen for `disliked_frames` detections in a row, and opens again after `disliked_lost_timeout` without it.
- **Liked apple:** it only closes when the button is pressed within `liked_memory` of last seeing the apple. Then it drops the apple:
  1. waits `drop_delay`
  2. publishes `arm_track_controller/move_aside`, and the arm turns by `aside_pan`
  3. waits `aside_time`
  4. opens the gripper

  From the button press until the gripper opens, it publishes `true` on `gripper_controller/dropping`, so the FSM stays in the `apple` state. After the drop the button does nothing until the node is enabled again.

Topics:

- Subscribes: `gripper_controller/enable`, `detections/apples/red` (liked), `detections/apples/green` (disliked), `button` (`mirte_msgs/IntensityDigital`, remapped to `/io/intensity/emergency_button/digital` in the launch file)
- Publishes: `arm_track_controller/move_aside` (`std_msgs/Empty`), `gripper_controller/dropping` (`std_msgs/Bool`, `transient_local`)
- Action client: `/mirte_master_gripper_controller/gripper_cmd` (`control_msgs/GripperCommand`)

How to tune:

- **Gripper doesn't fully open or close:** send test goals to `/mirte_master_gripper_controller/gripper_cmd` by hand and find the real limits.
- **Closes on false detections:** raise `disliked_frames`. The detector runs at about 5 Hz, so each frame adds about 0.2 s of delay.
- **Opens and closes rapidly (flickers) while the apple is held:** raise `disliked_lost_timeout`.
- **Button does nothing:** the liked apple wasn't seen in the last `liked_memory` seconds (the log says so), or `button_pressed_value` is wrong for your button. Check with `ros2 topic echo /io/intensity/emergency_button/digital`.
- **Gripper opens before the arm has finished moving aside:** raise `aside_time`.

> These parameters are only read at startup, so `ros2 param set` has no effect. Restart the node after changing them.

### `arm_track_controller`

| Parameter | Default | Meaning |
|---|---|---|
| `counter_rotate` | `true` | Turn the arm against the base rotation (set from the launch argument) |
| `counter_rotate_ratio` | `1.0` | Fraction of the base rotation the arm undoes |
| `max_counter_pan` | `1.0` rad | How far the arm may lean against a turn while searching |
| `aside_pan` | `0.8` rad | Added to `shoulder_pan_joint` when moving aside to drop the liked apple. Negative = the other side |

Hard-coded constants. Change these values in [include/petting_zoo_navigation/arm_track_controller.hpp](include/petting_zoo_navigation/arm_track_controller.hpp) and rebuild.

| Constant | Value | Meaning |
|---|---|---|
| `start_pose_` | `{0, 0, -1.3, 0}` | Pan, lift, elbow, wrist pose the arm moves to before tracking starts |
| `image_width_` / `image_height_` | `640` / `480` | Camera resolution. Must match the detector's input. |
| `rad_per_pixel_` | `1.047 / 640` | Camera horizontal FOV (≈60°) divided by the image width |
| `gain_` | `0.7` | Fraction of the pixel error corrected in each step (0–1) |
| `deadband_px_` | `15` | Pixel error below this is ignored |
| `move_time_` | `0.2` s | `time_from_start` for each trajectory point |

The node tracks the **largest** box. That is the human, or the apple named on `arm_track_controller/apple_colour` (`""` = human, `"liked"`/`"disliked"` = that apple, anything else = hold still). Horizontal error drives `shoulder_pan_joint` and vertical error drives `wrist_joint`. Both are clamped to ±π/2. A message on `arm_track_controller/move_aside` (`std_msgs/Empty`) turns `shoulder_pan_joint` by the `aside_pan` parameter (default `0.8` rad, negative = other side) and pauses tracking until the next enable or `apple_colour` message. You can also send a pose directly on `arm_target` (a `JointTrajectoryPoint` with 4 positions).

How to tune:

- **Arm oscillates around the target:** lower `gain_` or raise `deadband_px_`.
- **Arm follows too slowly:** raise `gain_` (keep it ≤ 1), or lower `move_time_`.
- **Arm jitters when the target is still:** raise `deadband_px_`. Detection boxes jump by a few pixels between frames.
- If you change cameras, update `image_width_`, `image_height_` and `rad_per_pixel_` (= horizontal FOV in rad / width).
- **Arm hardly moves when moving aside:** the pan is clamped to ±π/2, so if the arm is already turned that way there's little room left. Use a negative `aside_pan`.

---

## Live tuning

The two walkers re-read their parameters on every scan, so you can tune them while they are running:

```bash
ros2 param set /walking_around_field rep_gain 1.0
ros2 param set /walking_around arm_reach 0.40
ros2 param dump /walking_around_field   # copy the final values into the launch file
```

To test a node without the FSM, enable it by hand:

```bash
ros2 topic pub --once --qos-durability transient_local /walking_around/enable std_msgs/msg/Bool "{data: true}"
```

The enable subscribers use `transient_local` durability. A publisher with the default (volatile) durability is incompatible with them, so the message is never delivered.

To make tuned values permanent, add `<param>` entries to the node in [launch/petting_zoo.launch.xml](launch/petting_zoo.launch.xml).
