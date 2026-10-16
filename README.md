# Mirte Petting Zoo

This repo contains all the code needed to run the petting zoo demo using the MIRTE Master robot.

During the demo the robot moves around randomly. When it detects a human, it stops and tracks them with the camera on the gripper. When the human holds up an apple, the robot opens the gripper:

- **Liked apple:** the arm follows the apple. Put the apple in the gripper and press the button. The gripper closes, and after a short delay the arm moves to the side and opens the gripper to drop the apple. Only then does the robot move on.
- **Disliked apple:** the arm holds still and the gripper closes on its own. It opens again when the apple is taken away.

Which colour the robot likes is set with the `liked_apple` launch argument (`red` by default, or `green`).

To launch the entire demo:

```bash
ros2 launch petting_zoo_navigation petting_zoo.launch.xml
ros2 launch petting_zoo_navigation petting_zoo.launch.xml liked_apple:=green   # the robot likes green apples
```

| Package | What it contains |
|---|---|
| [petting_zoo_fsm](petting_zoo_fsm/README.md) | The state machine that decides what the robot is doing |
| [petting_zoo_navigation](petting_zoo_navigation/README.md) | The motion nodes (walkers, arm tracking, gripper) and the main launch file |
| `human_detector_yolo` | YOLO human detector |
| `mirte_apple_detector` | Colour-based red/green apple detector |

Each package's README explains how it works and how to tune it.
