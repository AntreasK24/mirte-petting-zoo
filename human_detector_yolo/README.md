# human_detector_yolo

A ROS 2 node that detects **humans** in a compressed image stream using **YOLO-FastestV2** on the **NCNN** runtime. It runs on the CPU only (no GPU) with a ~0.5 MB model, and is tuned for small ARM boards such as the Orange Pi 3 (quad Cortex-A53).

Apples are handled by the separate `mirte_apple_detector` package.

## How it works

For each frame (after rate limiting):

1. Decode the JPEG.
2. Resize it to `ncnn_input_size`² and run YOLO-FastestV2, a COCO model with 80 classes.
3. Keep only cells whose best class is `person` and whose `objectness × class score > confidence_threshold`.
4. Remove duplicate boxes with NMS (IoU 0.25).
5. Publish the boxes in the coordinates of the original image.

## Topics

**Subscribes**
- `image_topic` (`sensor_msgs/CompressedImage`). Default in the YAML: `/gripper_camera/image_raw/compressed`.

**Publishes**
- `detections/humans` (`vision_msgs/BoundingBox2DArray`)
- `human_detector/image` (`sensor_msgs/Image`) — the camera frame with
  detections drawn on it, for visualisation. Only rendered while something
  subscribes; turn it off entirely with `publish_debug_image` (then the topic
  is never advertised).

## Install & build

```bash
# model (~0.5 MB) + ncnn runtime
bash src/human_detector_yolo/scripts/download_model.sh
pip install --no-deps ncnn      # --no-deps: a pip OpenCV would shadow the ROS one and break cv_bridge
cd ~/dev_ws
colcon build --packages-select human_detector_yolo
source install/setup.bash
```

## Run

```bash
ros2 launch human_detector_yolo human_detector_yolo.launch.xml
# or with your own config
ros2 launch human_detector_yolo human_detector_yolo.launch.xml config_file:=/path/to/your.yaml
```

Check what it sees:

```bash
ros2 topic echo /detections/humans
ros2 topic hz /detections/humans
ros2 run rqt_image_view rqt_image_view /human_detector/image
```

## Parameters

All of them are set in [config/human_detector_yolo.yaml](config/human_detector_yolo.yaml). The top-level key must match the node name, `human_detector_yolo`. The parameters are only read at startup, so restart the node after you change them.

<<<<<<< HEAD
- `max_rate` — cap detection Hz (dropped frames skip decode; frees CPU).
- `ncnn_input_size` — multiple of 32. Lower = faster (352→224 ~2×, 352→192 ~2.5×).
- `num_threads` — CPU cores to use.
- `confidence_threshold` — raise to be stricter.
- `publish_debug_image` (`true`/`false`) — whether to publish the annotated
  `human_detector/image` feed at all.
=======
| Parameter | YAML value | Code default | Meaning |
|---|---|---|---|
| `image_topic` | `/gripper_camera/image_raw/compressed` | `/camera/image_raw/compressed` | Compressed image input |
| `queue_size` | `1` | `1` | Subscriber queue. 1 = always process the newest frame. |
| `max_rate` | `5.0` | `0.0` | Maximum detection rate in Hz. Extra frames are dropped **before** decoding. 0 = every frame. |
| `model_path` | `''` | `''` | `.bin` weights. Empty = the bundled `models/yolo-fastestv2-opt.bin`. |
| `config_path` | `''` | `''` | `.param` network file. Empty = the bundled one. |
| `num_threads` | `1` | `4` | CPU threads NCNN uses |
| `ncnn_input_size` | `224` | `352` | Network input size in pixels. Must be a multiple of 32 (other values are rounded). |
| `ncnn_fp16` | `false` | `false` | FP16 storage/arithmetic. Can help on ARMv8.2+ (A55 and newer). No gain on A53. |
| `confidence_threshold` | `0.5` | `0.5` | Minimum `objectness × class score` for a box to be kept |

These are hard-coded in [human_detector_yolo/human_detector_node.py](human_detector_yolo/human_detector_node.py): `NMS_THRESH = 0.25` and `PERSON_CLASS = 0`.

## Tuning

Tune for speed first, then for accuracy.

**Speed / CPU load**
- `ros2 topic hz /detections/humans` shows the real rate. If it is below `max_rate`, the board can't keep up.
- **Too slow:** lower `ncnn_input_size` (224 → 192 is about 1.3× faster), or raise `num_threads`. The comment in the YAML says "use all 4 cores" but the value is `1`. Use 3 if other heavy nodes share the board, and 4 if nothing else does.
- **CPU too busy for the rest of the stack:** lower `max_rate`. The FSM and `gripper_controller` assume about 5 Hz (0.5 s gaps reset their streaks). If you go much below 3 Hz, raise those thresholds too.

**Accuracy**
- **People are missed, especially far away:** raise `ncnn_input_size` (256, 288, 320, 352), or lower `confidence_threshold` to 0.35–0.4. A larger input helps small, distant people the most.
- **False people (chairs, posters, coats):** raise `confidence_threshold` to 0.6–0.7.
- **Two boxes on one person:** lower `NMS_THRESH` in the source.
- **Box flickers on and off:** the FSM and arm already handle short dropouts, so you usually don't need to fix it here. If it is very bad, lower `confidence_threshold` a little.

Check the overlay image with your real camera, at the distances you care about, while you tune.
>>>>>>> 516e97f (Added button behaviour)
