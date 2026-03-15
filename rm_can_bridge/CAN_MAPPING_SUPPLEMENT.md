# CAN Mapping Supplement (NUC -> Lower Controller)

Base references:
- `/home/thom/下载/（初版）thom战队电控can通信对照表.xlsx`
- `/home/thom/下载/USB转CAN-linux/can_common.py`

This document only defines fields used by `rm_can_bridge`.

## USB-CAN Transport Frame

The USB-CAN adapter uses a 30-byte serial frame (same layout as `can_common.py`):

- Fixed header/tail:
  - `[0]=0x55 [1]=0xAA [2]=0x1E ... [29]=0x88`
- CAN ID (little-endian `uint32`):
  - `[13..16]`
- DLC:
  - `[18]=0x08`
- CAN data bytes:
  - `[21..28]` -> `DATA[0..7]`

## Defined CAN IDs

### 0x00A (NUC Broadcast: E-Stop)

- Source from table: `0x00A` emergency stop.
- `DATA[0..7]`:
  - E-stop active: all `0xFF`
  - E-stop inactive: all `0x00`

### 0x301 (NUC -> Gimbal Controller)

From table:
- `DATA[0..1]`: yaw target (`q10_6`)
- `DATA[2..3]`: pitch target (`q10_6`)
- `DATA[4]`: fire bool

Current bridge behavior:
- `DATA[0..1]`: yaw target (`q10.6`, big-endian), default from `/detector/armors` or `/tracker/target`
- `DATA[2..3]`: pitch target (`q10.6`, big-endian), default from `/detector/armors` or `/tracker/target`
- `DATA[4]`: fire command (`0`/`1`)
- `DATA[5]`: supplement status bits
  - bit0: aim data fresh
  - bit1: fire active
- `DATA[6..7]`: reserved (`0`)

When `aim_reference_mode=absolute_from_feedback`:
- `0x402` feedback is decoded first.
- Relative camera aim offsets are converted into lower-controller target angles before encoding:
  - `yaw_cmd_deg = wrapTo180(yaw_feedback_deg + yaw_relative_deg)`
  - `pitch_cmd_deg = pitch_feedback_deg + pitch_relative_deg`
- The transmitted `0x301` yaw/pitch fields are still encoded as big-endian `q10.6` using the configured `angle_unit`.

### 0x311 (NUC -> Chassis Controller)

From table:
- `DATA[0..1]`: chassis forward speed target (`int16`)
- `DATA[2..3]`: chassis lateral speed target (`int16`)
- Others were placeholders.

Current bridge behavior:
- `DATA[0..1]`: `vx` from `/cmd_vel.linear.x`, scaled by `linear_scale` (`default 1000`, unit m/s -> mm/s), big-endian
- `DATA[2..3]`: `vy` from `/cmd_vel.linear.y`, scaled by `linear_scale`, big-endian
- `DATA[4..5]`: supplement `wz` from `/cmd_vel.angular.z`, scaled by `angular_scale` (`default 1000`, unit rad/s -> mrad/s), big-endian
- `DATA[6]`: supplement status bits
  - bit0: nav cmd fresh
  - bit1: avoid active (`/can/avoid_active`)
  - bit2: aim data fresh
  - bit3: fire active
- `DATA[7]`: reserved (`0`)

## Source Topics Used by `rm_can_bridge`

- `/detector/armors` (`auto_aim_interfaces/msg/Armors`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`) - nav/obstacle-avoid output velocity
- `/can/e_stop` (`std_msgs/msg/Bool`)
- `/can/fire_enable` (`std_msgs/msg/Bool`)
- `/can/avoid_active` (`std_msgs/msg/Bool`)

## Notes

- `q10_6` conversion:
  - encoded value = `round(angle * 64)`
  - default angle unit in node: degree (`angle_unit=deg`)
- If your lower controller expects radians for `q10_6`, set `angle_unit=rad`.

### 0x402 (Gimbal Controller -> NUC)

- `DATA[0..1]`: yaw (`q10.6`, big-endian, deg)
- `DATA[2..3]`: pitch (`q10.6`, big-endian, deg)
- `DATA[4..5]`: roll (`q10.6`, big-endian, deg)
- `DATA[6..7]`: yaw motor position (`q10.6`, big-endian, deg)

ROS feedback topics published by `rm_can_bridge`:
- `/gimbal/feedback/rpy_deg` (`geometry_msgs/msg/Vector3Stamped`)
  - `vector.x = roll`, `vector.y = pitch`, `vector.z = yaw`
- `/gimbal/feedback/yaw_motor_pos_deg` (`std_msgs/msg/Float64`)
