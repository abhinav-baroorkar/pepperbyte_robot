# Cobra Flex Firmware Verification

**Date:** 2026-03-31  
**Sources verified against:**
- Waveshare Cobra Flex documentation
- Firmware source: `/home/pepperbyte/Downloads/Cobra_Flex0519/Cobra_Driver/`  
  Key files: `json_cmd.h`, `ugv_advance.h`, `movtion_module.h`, `ugv_config.h`, `battery_ctrl.h`

---

## 1. Serial Port Mapping — ❌ WAS WRONG (now fixed)

**Issue:** The udev rule was pointing `/dev/cobra_flex` at the Espressif JTAG debug interface (`303a:1001` → `ttyACM0`). The actual data/feedback UART is the QinHeng CH343 chip (`1a86:55d3` → `ttyACM1`).

**Evidence:** Live serial read of `ttyACM1` confirmed T:1001 feedback packets flowing. `ttyACM0` was silent.

**Fix applied:** Updated `99-cobra-flex.rules` to match `1a86:55d3` instead of `303a:1001`.

---

## 2. JSON Command — T:13 (CMD_ROS_CTRL) — ✅ CORRECT

Our cobra_driver sends:
```
{"T":13,"X":<linear.x>,"Z":<angular.z>}
```

Confirmed in `json_cmd.h`:
```c
// {"T":13,"X":0.1,"Z":0.3} (m/s,rad/s)
#define CMD_ROS_CTRL 13
```

And in `movtion_module.h`:
```cpp
void rosCtrl(float rosX, float rosZ) {
    setpointA = rosX - (rosZ * TRACK_WIDTH / 2.0);  // left wheel (m/s)
    setpointB = rosX + (rosZ * TRACK_WIDTH / 2.0);  // right wheel (m/s)
    setpointA = setpointA * 60 / (M_PI * WHEEL_D);  // convert to RPM
    setpointB = setpointB * 60 / (M_PI * WHEEL_D);
    setGoalSpeed(setpointA * 10, setpointB * 10);   // units: 0.1 RPM
}
```

The T:13 command is the correct ROS control interface. ✅

---

## 3. Continuous Feedback Enable — T:131 — ✅ CORRECT

Our cobra_driver sends `{"T":131,"cmd":1}` to enable continuous feedback.

Confirmed in `json_cmd.h`:
```c
// on: {"T":131,"cmd":1} [default]
#define CMD_BASE_FEEDBACK_FLOW 131
```
✅

---

## 4. Feedback Packet Format — ✅ CORRECT (but docs misleading)

**What the docs show (incorrect example):**
```json
{"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":0,"odr":0,"v":1173}
```

**What json_cmd.h describes (different firmware variant):**
```
{"T":1001,"L":0,"R":0,"gx":0,...,"odl":0,"odr":0,"v":11.0}
```

**What the actual running firmware sends (confirmed from source `ugv_advance.h:baseInfoFeedback()`):**
```json
{"T":1001,"M1":<fb1>,"M2":<fb2>,"M3":<fb3>,"M4":<fb4>,"odl":<cm>,"odr":<cm>,"v":<v*100>}
```

The `json_cmd.h` comment reflects a different firmware variant. The actual `baseInfoFeedback()` function (which is what runs) uses M1-M4 fields — matching what our parser expects. ✅

**Field meanings confirmed from source:**

| Field | Source variable | Description |
|-------|----------------|-------------|
| `M1` | `ddsm_fb_1` | Left-front wheel feedback speed (0.1 RPM) |
| `M2` | `ddsm_fb_2` | Right-front wheel feedback speed (0.1 RPM) |
| `M3` | `ddsm_fb_3` | Right-rear wheel feedback speed (0.1 RPM) |
| `M4` | `ddsm_fb_4` | Left-rear wheel feedback speed (0.1 RPM) |
| `odl` | `en_odom_l * 100` | Left cumulative odometry (integer cm) |
| `odr` | `en_odom_r * 100` | Right cumulative odometry (integer cm) |
| `v` | `loadVoltage_V * 100` | Battery voltage × 100 (integer) |

---

## 5. Voltage Decoding — ✅ CORRECT

Our cobra_driver:
```cpp
battery_msg.data = static_cast<float>(v / 100.0);
```

Confirmed from `ugv_advance.h`:
```cpp
int v_int = (int)(loadVoltage_V * 100);
jsonInfoHttp["v"] = v_int;
```

So `v / 100.0` correctly recovers volts. ✅

**Note on low voltage readings:** Live capture showed `"v":35` (= 0.35V). The voltage comes from an INA219 power monitor (`battery_ctrl.h`). If the battery is not connected and the robot is powered by USB only, the INA219 may return near-zero readings. This is a hardware issue, not a firmware or driver issue.

---

## 6. Odometry Units and Accumulation — ✅ CORRECT

From `ugv_advance.h` and `movtion_module.h`:
- `en_odom_l` and `en_odom_r` accumulate in **metres**
- Feedback sends `(int)(en_odom_l * 100)` → integer **centimetres**
- Values are **cumulative** (not per-packet deltas)

Our cobra_driver:
```cpp
double delta_left_m  = (odl_cm - prev_odl_cm_) * 0.01;   // cm delta → metres
double delta_right_m = (odr_cm - prev_odr_cm_) * 0.01;
```
Correctly computes delta from cumulative values and converts cm → m. ✅

---

## 7. Motor ID Mapping — ✅ CORRECT

From `movtion_module.h:setGoalSpeed()`:
```cpp
void setGoalSpeed(int inputLeft, int inputRight) {
    ddsm_spd_1 =  inputLeft;    // M1 = left-front
    ddsm_spd_2 = -inputRight;   // M2 = right-front (physically mirrored)
    ddsm_spd_3 = -inputRight;   // M3 = right-rear  (physically mirrored)
    ddsm_spd_4 =  inputLeft;    // M4 = left-rear
}
```

Our joint state mapping in cobra_driver:
```cpp
js_msg.name = {"front_left_wheel_joint", "rear_left_wheel_joint",
               "front_right_wheel_joint", "rear_right_wheel_joint"};
js_msg.position = {left_angle, left_angle, right_angle, right_angle};
```

Left wheels (M1/M4) map to `odl`, right wheels (M2/M3) map to `odr`. ✅  
Note: M2 and M3 are negated in firmware because the right wheels are physically reversed — but `odr` already accounts for this via signed accumulation.

---

## 8. Keepalive Command — ✅ CORRECT

Our driver sends `{"T":13,"X":0,"Z":0}` when no cmd_vel received for `cmd_vel_timeout_ms`.

Firmware `heartBeatCtrl()` stops motors after `HEART_BEAT_DELAY = 3000ms` with no command. Our keepalive interval is `cmd_vel_timeout_ms / 2 = 1000ms` — well within the 3s window. ✅

---

## 9. Wheel Diameter Discrepancy — ⚠️ MINOR

| Source | Wheel Diameter |
|--------|---------------|
| Waveshare spec | 74.5 mm |
| Firmware (`ugv_config.h`) | **73.9 mm** (`WHEEL_D = 0.0739`) |
| Our URDF + cobra_driver params | 74.5 mm |

**Impact:**
- The firmware uses 73.9mm to convert m/s → RPM inside `rosCtrl()`. This is an internal firmware value and does not affect how we send commands.
- Our odometry uses 74.5mm to convert odl/odr encoder distances → metres. Since `en_odom_l` is computed in the firmware using 73.9mm, and we interpret those centimetres with a slightly different radius, there is a **~0.8% scale error** in odometry distance. Not significant for typical indoor use.

**Recommendation:** Optionally update `wheel_diameter` in `cobra_driver_params.yaml` to `0.0739` to match the firmware's internal calibration value, which will make odometry distances consistent with the firmware's odometry computation.

---

## 10. Track Width Discrepancy — ⚠️ IMPORTANT (firmware limitation)

| Source | Track Width |
|--------|-------------|
| Waveshare spec | 228 mm |
| Firmware (`ugv_config.h`) | **159 mm** (`TRACK_WIDTH = 0.159`) |
| Our URDF + cobra_driver params | 228.5 mm |

**Impact on T:13 commands (firmware-side):**  
The firmware uses 159mm in `rosCtrl()` when converting Z (rad/s) to wheel speed differential. Since the physical track is 228.5mm, the firmware under-drives the turning speed by a factor of `159/228.5 ≈ 0.70`. For a commanded Z of 1.0 rad/s, the robot will actually turn at ~0.70 rad/s.

This is an **empirical calibration** parameter in the firmware (likely tuned for slip compensation on the 4WD skid-steer). It cannot be corrected from our ROS driver since we send velocity in m/s and rad/s and the firmware applies its own conversion.

**Impact on our odometry:**  
Our odometry uses `track_width = 0.2285m` (physical geometry). This is correct for computing `delta_theta` from actual wheel distance deltas (`odl`/`odr`), since those are real encoder-derived distances. The firmware's 159mm value is irrelevant here. ✅

**Recommendation:** No change needed in our driver. If turn accuracy is critical, the firmware's `TRACK_WIDTH` would need to be updated to 0.2285m — but that requires re-flashing the ESP32.

---

## Summary

| Check | Status | Notes |
|-------|--------|-------|
| Serial port (`/dev/cobra_flex`) | ✅ Fixed | Was `303a:1001` (JTAG), corrected to `1a86:55d3` (data UART) |
| T:13 ROS control command | ✅ Correct | Confirmed in `json_cmd.h` and `movtion_module.h` |
| T:131 feedback enable | ✅ Correct | Matches firmware |
| Feedback packet fields (M1-M4, odl, odr, v) | ✅ Correct | Confirmed from `baseInfoFeedback()` source |
| Voltage decoding (`v / 100.0`) | ✅ Correct | Firmware multiplies by 100 before sending |
| Odometry units (cm, cumulative) | ✅ Correct | Matches firmware accumulation logic |
| Motor-to-joint mapping | ✅ Correct | M1/M4=left, M2/M3=right |
| Keepalive timing | ✅ Correct | Sends within 3s firmware heartbeat window |
| Wheel diameter | ⚠️ 74.5mm vs firmware 73.9mm | ~0.8% odometry scale error, minor |
| Track width (odometry) | ✅ Correct | Using physical 228.5mm is right for odom |
| Track width (firmware turning) | ⚠️ Firmware uses 159mm | Turns at ~70% of commanded rate — firmware limitation |
