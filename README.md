SDK-less ROS2 driver for Structure Core.

This package does not require the proprietary Structure SDK.
It supports two runtime modes:

- `hardware`/`auto`: try USB vendor protocol via `libusb` (`0x2959:0x3001`), then UVC fallback
- `synthetic`: software-generated test streams

It publishes the following topics:

- `depth/image`, `depth/camera_info`
- `depth_aligned/image`, `depth_aligned/camera_info`
- `depth_ir_aligned/image`, `depth_ir_aligned/camera_info`
- `visible/image_raw`, `visible/camera_info`
- `left/image_raw`, `left/camera_info`
- `right/image_raw`, `right/camera_info`

Build:

1) Source ROS2:

- `source /opt/ros/<distro>/setup.bash`

2) Build:

- `colcon build --packages-select structure_core --cmake-args -DCMAKE_BUILD_TYPE=Release`

3) Run:

- `source install/setup.bash`
- `ros2 run structure_core structure_driver`

Compatibility note:

- If your shell setup expects workspace-root `local_setup.bash`, use `source ./local_setup.bash` from this directory.

Useful parameters:

- `frame_rate` (default `15.0`)
- `depth_width`, `depth_height` (default `640x480`)
- `visible_width`, `visible_height` (default `640x480`)
- `ir_width`, `ir_height` (default `640x480`)
- `publish_if_subscribed_only` (default `true`)
- `depth_frame_id`, `visible_frame_id`, `left_frame_id`, `right_frame_id`
- `io_mode` (`auto` / `hardware` / `synthetic`, default `auto`)
- `hardware_backend` (`auto` / `libusb` / `uvc`, default `auto`)
- `vendor_poll_timeout_ms` (default `40`)
- `vendor_no_data_reconnect_sec` (default `6.0`, `<=0` to disable auto reconnect)
- `vendor_reconnect_retry_sec` (default `1.0`, retry period after backend reopen failure)
- `vendor_auto_start_command` (default `true`)
- `vendor_start_mode` (`frame20` / `payload12` / `header4` / `header8` /
  `frame_then_payload` / `broadcast_frame20` / `broadcast_payload12` /
  `broadcast_frame_then_payload`, default `frame20`)
- `vendor_start_auto_cycle` (default `false`, rotates through known start profiles on no-data)
- `vendor_start_auto_cycle_extended` (default `false`, expands command/arg sweep set)
- `vendor_start_retry_sec` (default `1.5`, resend interval for start sequence while no-data)
- `vendor_start_retry_profiles_per_cycle` (default `3`, number of profiles sent per resend window)
- `vendor_start_endpoint` (default `1` for EP `0x01`)
- `vendor_start_repeat_count` (default `1`)
- `vendor_start_interval_ms` (default `25`)
- `vendor_start_preface` (default `false`, sends `header4` + `header8` before main payload)
- `vendor_start_command_id` (default `0x10000013`)
- `vendor_start_arg0` (default `0x0000000f`)
- `vendor_start_arg1` (default `0x00000000`)
- `device_name_filter` (default `Structure Core`, use `any`/`all` to match any camera)
- `visible_device_index` (default `0`)
- `ir_device_index` (default `-1`, disabled)
- `hardware_fallback_to_synthetic` (default `true`)
- `split_stereo_frame` (default `true`)

Example:

`ros2 run structure_core structure_driver --ros-args -p io_mode:=hardware -p hardware_backend:=libusb`

Standalone protocol scan (no ROS2 runtime required):

- `./tools/run_vendor_burst_scan.sh --max-tests 240 --read-ms 45 --open-retries 4 --open-wait-ms 20 --max-open-fails 6`
- The scanner tries interleaved templates (`frame20`, `payload12`, `frame_then_payload`, preface, broadcast).
- Use `--verbose` for per-candidate logs.

Notes:

- In `libusb` hardware mode, `visible/left/right` are derived from endpoint payload bytes.
- In `uvc` hardware mode, `visible` and `left/right` come from UVC frames.
- `depth*` topics are pseudo-depth generated from image intensity (not metric depth).
- If `libusb poll failed` appears repeatedly, increase `vendor_poll_timeout_ms` (for example `80` to `120`).
- For protocol probing, try `vendor_start_mode:=frame_then_payload` or
  `vendor_start_mode:=broadcast_frame_then_payload` with `vendor_start_preface:=true`.
- If auto-cycle is enabled, increase `vendor_start_retry_profiles_per_cycle` to `3` to `6`
  to test multiple candidates each retry window.
- If no payload is received, enable `vendor_start_auto_cycle:=true` to iterate known start profiles.
- If no hardware stream can be opened, `auto` falls back to synthetic mode.
