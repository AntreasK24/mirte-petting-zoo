# mirte_apple_detector

A ROS 2 **C++** node (`rclcpp`) that detects **red** and **green apples** in a compressed image stream using **classical colour + shape analysis**. There is no neural network and no model to download. Red and green apples are classified separately.

Humans are handled by the separate `human_detector_yolo` package.

## How it works

For each frame (after rate limiting):

1. Decode the JPEG, apply a Gaussian blur (`blur_ksize`) and convert to HSV and LAB.
2. For each colour, threshold a mask from its `hue_ranges` and `sat_min`/`val_min`. Red wraps around hue 0, so it uses two ranges. Then keep only pixels whose LAB `a*` lies in `a_min`–`a_max`. `a*` is a green↔red axis that does not change with brightness, so this removes skin, grey-green and teal that pass the HSV check.
3. Morphological **open** (removes specks), then **close** (fills highlight holes), with a `morph_ksize` ellipse.
4. Find the outer contours. Keep the ones that pass:
   - `min_area` ≤ area ≤ `max_area`
   - circularity `4π·area / perimeter²` ≥ `min_circularity` (1.0 = a perfect circle)
   - solidity `area / convex-hull area` ≥ `min_solidity` (1.0 = completely filled in)
   - aspect `short side / long side` of the box ≥ `min_aspect` (1.0 = square)
   - fill `area / enclosing-circle area` ≥ `min_fill` (1.0 = a perfect circle, 0.64 = a square)
5. Where a red and a green box overlap by more than `overlap_ratio` of the smaller box (for example a blushed apple), keep only the one with more pixels.
6. Publish one bounding box per surviving contour.

## Topics

**Subscribes**
- `image_topic` (`sensor_msgs/CompressedImage`). Default in the YAML: `/gripper_camera/image_raw/compressed`.

**Publishes**
- `detections/apples/red` (`vision_msgs/BoundingBox2DArray`)
- `detections/apples/green` (`vision_msgs/BoundingBox2DArray`)
- `apple_detector/image` (`sensor_msgs/Image`): the frame with the detections drawn on it. It is only rendered while something is subscribed, and is not advertised at all when `publish_debug_image: false`.
- `apple_detector/mask/red`, `apple_detector/mask/green` (`sensor_msgs/Image`, mono8): each colour's final mask, before the shape filters. Same rules as the debug image. Use them to see whether a missed apple fails the colour check (missing from the mask) or the shape check (white in the mask but no box).

Both detection topics are published on every processed frame, even when empty.

Who uses them:
- The FSM uses red and green to decide whether an apple is in the human's hand.
- `arm_track_controller` follows the red apple.
- `gripper_controller` closes the gripper on green. It does **not** check whether the green apple is in a hand.

## Build & run

```bash
cd ~/dev_ws
colcon build --packages-select mirte_apple_detector
source install/setup.bash
ros2 launch mirte_apple_detector apple_detector.launch.xml
# or with your own config
ros2 launch mirte_apple_detector apple_detector.launch.xml config_file:=/path/to/your.yaml
```

Check what it sees (set `publish_debug_image: true` first):

```bash
ros2 topic echo /detections/apples/red
ros2 run rqt_image_view rqt_image_view /apple_detector/image
ros2 run rqt_image_view rqt_image_view /apple_detector/mask/red
```

## Parameters

All of them are set in [config/apple_detector.yaml](config/apple_detector.yaml). The top-level key must match the node name, `apple_detector`. The parameters are only read at startup, so restart the node after you change them.

### Input

| Parameter | YAML value | Code default | Meaning |
|---|---|---|---|
| `image_topic` | `/gripper_camera/image_raw/compressed` | `/camera/image_raw/compressed` | Compressed image input |
| `queue_size` | `1` | `1` | Subscriber queue. 1 = always process the newest frame. |
| `max_rate` | `5.0` | `0.0` | Maximum detection rate in Hz. Extra frames are dropped before decoding. 0 = every frame. |
| `publish_debug_image` | `true` | `true` | Advertise and draw `apple_detector/image` and `apple_detector/mask/<colour>` |

### Shape / clean-up

| Parameter | YAML value | Code default | Meaning |
|---|---|---|---|
| `min_area` | `700` | `500` | Smallest blob area in px² (700 ≈ a 30 px wide apple) |
| `max_area` | `100000` | `0` | Largest blob area in px². 0 = no limit. 100000 ≈ a third of a 640×480 frame. |
| `min_circularity` | `0.35` | `0.6` | Roundness threshold. Real apples measure about 0.35–0.8 because of highlights and stems. |
| `min_solidity` | `0.78` | `0.85` | How filled-in the blob must be. This is the main shape filter. |
| `min_aspect` | `0.55` | `0.0` | Box short side / long side. Rejects sleeves, bottles and other long shapes. 0 = off. |
| `min_fill` | `0.45` | `0.0` | Blob area / enclosing-circle area. Less noisy than circularity on jagged edges. 0 = off. |
| `overlap_ratio` | `0.3` | `0.3` | Red and green boxes overlapping more than this fraction of the smaller box clash; the one with more pixels wins. 0 = off. |
| `blur_ksize` | `5` | `5` | Gaussian blur kernel before HSV. Odd; values < 3 turn it off. |
| `morph_ksize` | `11` | `5` | Kernel size for the morphological open/close |

### Colour (one block per colour: `red.*`, `green.*`)

| Parameter | Red | Green | Meaning |
|---|---|---|---|
| `hue_ranges` | `[0, 10, 170, 179]` | `[28, 95]` | Flat list of `[lo, hi, lo, hi, …]` hue pairs. OpenCV hue runs from 0 to 179. |
| `sat_min` | `120` | `40` | Minimum saturation (0–255). Higher = only vivid colours. |
| `val_min` | `60` | `30` | Minimum brightness (0–255). Lower = accept darker, shadowed pixels. |
| `a_min` | `145` | `0` | Minimum LAB `a*` (0–255, 128 = neutral). Red apples are about 160–190, skin about 135–150. |
| `a_max` | `255` | `120` | Maximum LAB `a*`. Green apples are about 90–115, grey and teal about 125–130. |

Set `a_min: 0` and `a_max: 255` to turn the LAB check off for a colour.

The maximum saturation and value are always 255. Hue cheat sheet: red 0–10 & 170–179 · orange 11–25 · yellow 25–34 · green 35–85 · cyan 85–100 · blue 100–130.

## Tuning

Turn on `publish_debug_image`, open `rqt_image_view` and tune under the **real lighting** on the robot camera. Colour thresholds that work at your desk will not work in another room.

**1. Colour first.** Get each colour's mask right before you touch the shape filters.

- To find an apple's real HSV values, grab a frame and inspect it:
  ```bash
  ros2 run image_view image_saver --ros-args -r image:=/gripper_camera/image_raw -p filename_format:=frame.png
  python3 -c "import cv2; h=cv2.cvtColor(cv2.imread('frame.png'),cv2.COLOR_BGR2HSV); print(h[240,320])"   # pixel at (x=320,y=240)
  python3 -c "import cv2; l=cv2.cvtColor(cv2.imread('frame.png'),cv2.COLOR_BGR2Lab); print(l[240,320])"    # [L, a*, b*]
  ```
- **Watch the masks** (`apple_detector/mask/<colour>`), not just the boxes. Tune until the apple is a solid white blob and the background is black.
- **Apple missed or only partly boxed:** widen the hue range, or lower `sat_min`/`val_min`. Red apples often fall into 10–20 (orange-red), so try `[0, 18, 170, 179]`.
- **Skin, wood or orange objects detected as red:** raise red `a_min` (150–155), then red `sat_min`, or narrow the low hue range. Skin is a low-saturation orange, around hue 5–20 with S 50–150.
- **Red apple missing from the mask in dim light:** lower red `a_min` (135–140). `a*` is stable across brightness, but very dark pixels drift towards neutral.
- **Walls, plants or clothes detected as green:** lower green `a_max` (110–115) first, then narrow `hue_ranges` (e.g. `[35, 85]`) or raise `sat_min`. The current `[28, 95]` is wide, so it also catches yellow-green and teal.
- **One apple boxed as both colours:** this is handled by `overlap_ratio`. If the colour flickers between frames, the apple is genuinely half-and-half; tighten the losing colour's `a_min`/`a_max`.
- **Detections disappear in shadow:** lower `val_min`.

**2. Then shape.**

- **Small specks detected:** raise `min_area`. Don't raise it beyond the size of an apple held at the farthest distance you care about. For example, a 7 cm apple 1.5 m away is only about 20–25 px wide, roughly 400 px².
- **Big blobs (a red shirt, a green wall) detected:** set `max_area`, or raise `min_solidity`/`min_circularity`.
- **Shiny apple splits into pieces, or rejected because the highlight leaves a hole:** raise `morph_ksize` (odd, 9–15) or `blur_ksize`. Larger kernels cost CPU and merge nearby blobs.
- **Long, thin or crescent shapes accepted:** raise `min_aspect` (0.6–0.7) and `min_solidity` (0.8–0.9), then `min_circularity`.
- **Apple held in a hand is rejected:** fingers cut into the outline, which lowers solidity, circularity and fill. Lower `min_solidity` a little (0.7–0.75), or `min_fill` (0.35–0.4).
- **Apple very close to the camera is rejected:** raise `max_area`.

**3. Rate.** The FSM and `gripper_controller` count detections in a row, with gaps of less than 0.5 s, so keep `max_rate` at 3 Hz or above. Before you lower it, check that the CPU actually needs the headroom.

**4. Lock the camera on the real robot.** Auto white balance and auto exposure shift every colour when the scene changes, for example when a person in a red shirt walks into view. On the robot, list the camera controls and turn the automatic ones off, then set fixed values that look right in the venue:
```bash
v4l2-ctl -d /dev/video0 --list-ctrls
v4l2-ctl -d /dev/video0 -c white_balance_automatic=0 -c auto_exposure=1   # names/values differ per camera
```
These settings reset when the camera is replugged or the robot reboots.

Colour and shape detection is lighting-sensitive by nature. Re-check the thresholds whenever the venue or lighting changes.
