# Cameras

| ID | Camera | udev name | Matched by | Format |
|---|---|---|---|---|
| `zbot_cam_follower_1` | SO-100 kit camera, Sonix 05a3:9230 | `/dev/zbot_cam_follower_1` | vendor + model (any port) | MJPEG 640×480 @ 30 |
| `zbot_cam_follower_2` | IR-capable, Sonix 0c45:6369 | `/dev/zbot_cam_follower_2` | vendor + model (any port) | MJPEG 640×480 @ 30 |

Driver: `zbot_arm/launch/zbot_cam.launch.py cam:=<ID>`, one per camera, configured only by
`zbot_arm/config/<ID>.yaml` (top key `usb_cam:`). Publishes `/<ID>/image_raw` and
`/<ID>/camera_info`.

**Rules**
- Always the udev name (`/dev/zbot_cam_…`), never `/dev/videoN`: the numbers shift with plug
  order, and the laptops' built-in webcams take `video0`/`video1`. The launch file refuses
  anything else.
- Both cameras have a generic serial, but they are different models, so each is named by
  vendor + model and works in any port, on the laptop hub or a Pi. **Only one camera of each
  model per machine.** A second identical camera on the same machine needs port-based rules
  (see "New camera"), written per machine because a Pi's port paths differ from the laptop
  hub's. When buying, prefer cameras with a real serial, or at least a model not already used.
- One program per camera, like serial ports: stop the camera's node before `lerobot-record`,
  `v4l2-ctl` streaming, or a viewer that opens the device.
- Prefer MJPEG. Raw YUYV at 640×480 @ 30 fills about a third of a USB 2.0 hub by itself.

---

## One-time per machine

1. `sudo apt install v4l-utils ros-jazzy-usb-cam ros-jazzy-rqt-image-view`
2. Be in the `video` group (needed when nobody is logged in at the screen, e.g. on the cart):
   `groups | grep video` → if missing, `sudo usermod -aG video $USER`, then log out and in.
3. `./system/setup.sh` (installs `system/udev/99-zbot-cameras.rules`), then
   `ls -l /dev/zbot_cam_*`.
4. `cd ~/zbot_ws && colcon build --symlink-install --packages-select zbot_arm` (new launch file
   and YAML).

## New camera: find its identity and give it a name

1. Plug in **only** the new camera (besides the built-in webcam).
2. ```
   v4l2-ctl --list-devices
   udevadm info -q property -n /dev/videoN | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL|ID_PATH=|ID_V4L_CAPABILITIES'
   v4l2-ctl -d /dev/videoN --list-formats-ext
   ```
   Use the node whose `ID_V4L_CAPABILITIES` is `:capture:`.
3. Add a rule to `system/udev/99-zbot-cameras.rules` (the file header explains the cases):
   - **Unique serial** (`ID_SERIAL_SHORT` looks like a real serial): match `ID_VENDOR_ID`,
     `ID_MODEL_ID`, `ID_SERIAL_SHORT`.
   - **Generic serial, model not used by another camera on that machine**: match
     `ID_VENDOR_ID` and `ID_MODEL_ID` only.
   - **Same model as another camera on the same machine**: also match
     `ID_PATH=="<path>"` copied from `udevadm info` on **that** machine, and label the port.
   - Always include `ENV{ID_V4L_CAPABILITIES}==":capture:"` and `SYMLINK+="zbot_cam_<name>"`.
   - Check its controls: `v4l2-ctl -d /dev/videoN --list-ctrls-menus`. If it has
     `power_line_frequency` (set 2 = 60 Hz) or `exposure_dynamic_framerate` (set 0), add a
     `RUN+="/usr/bin/v4l2-ctl -d $devnode -c …"` like the existing rules.
4. `./system/setup.sh`, then `sudo udevadm settle && ls -l /dev/zbot_cam_*`. Unplug and replug
   once: the link must come back. Then check the controls stuck:
   `v4l2-ctl -d /dev/zbot_cam_<name> --get-ctrl power_line_frequency,exposure_dynamic_framerate`.
5. Copy `zbot_arm/config/zbot_cam_follower_1.yaml` to `zbot_cam_<name>.yaml`; change
   `camera_name`, `video_device`, `frame_id`, and the format to one the camera lists at 30 fps
   (MJPEG). `colcon build` once. Add a row to the table above. Commit.

OAK-D Lite and other depth cameras are not UVC devices here; they use their own ROS driver
(depthai-ros) and are identified by the device's own ID. Separate section when we add one.

**IR camera (follower_2):** one sensor; any IR LEDs or IR-filter switching is automatic in the
camera and can't be turned off. If the picture turns grey in dim light, it has switched: record
only in light bright enough that it never does.

## Every session: start a camera

Every terminal: `ros`.

1. `ls -l /dev/zbot_cam_*`
2. One terminal per camera:
   ```
   ros2 launch zbot_arm zbot_cam.launch.py cam:=zbot_cam_follower_1
   ros2 launch zbot_arm zbot_cam.launch.py cam:=zbot_cam_follower_2    # second terminal
   ```
   The `[zbot_cam.launch]` line shows the device, size, rate and format. A warning about no
   camera calibration file is expected.
3. Check:
   ```
   ros2 topic hz /zbot_cam_follower_1/image_raw     # ~30
   ros2 run rqt_image_view rqt_image_view            # pick /zbot_cam_follower_1/image_raw
   ```
   In RViz: Add → By topic → `/zbot_cam_follower_1/image_raw` → Image.
4. Stop: Ctrl-C.

## Bandwidth check (all arms + all cameras on the hub)

Start every arm node and every camera, then watch each camera's rate for a minute:
`ros2 topic hz /<cam>/image_raw`. All should hold ~30 Hz, and the arms' logs should show no
`bus reads failing`. A camera that won't start with `No space left on device` (or similar)
means the hub's USB bandwidth is full: drop that camera to MJPEG or a lower size, or move it
to another root port.

## Quick fixes

| Symptom | Fix |
|---|---|
| `ls: cannot access '/dev/zbot_cam_*'` right after setup | `sudo udevadm settle`, then look again |
| No link after `setup.sh` with the camera already plugged in | Rules apply when a device appears: replug, or `sudo udevadm trigger --subsystem-match=video4linux --action=add` (setup.sh does this) |
| Still no link | Vendor/model differ from the rule (another camera?), or a port-based rule and the camera moved: compare with `udevadm info`. `udevadm test $(udevadm info -q path -n /dev/videoN) 2>&1 \| grep zbot` shows whether the rule matches |
| `video_device … is not a udev name` | Use `/dev/zbot_cam_…` in the YAML |
| `… does not exist` at launch | Unplugged, wrong port, or rule not installed (`./system/setup.sh`) |
| `Permission denied` opening the device | Not in the `video` group (One-time, step 2) |
| `Device or resource busy` | Another program has the camera (viewer, `lerobot-record`, another node) |
| Banding/flicker stripes under room lights | `power_line_frequency` not at 60 Hz: replug (the rule sets it) or `v4l2-ctl -d … -c power_line_frequency=2` |
| Image goes grey (follower_2) | IR mode switched on in dim light: more light |
| `topic hz` well under 30 | Low light (the camera lengthens exposure) or USB bandwidth; check with fewer devices on the hub |
| Package `usb_cam` not found | `sudo apt install ros-jazzy-usb-cam` |

*Last updated: 2026-11-02*
