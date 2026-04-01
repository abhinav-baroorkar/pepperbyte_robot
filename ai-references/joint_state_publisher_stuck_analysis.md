# Analysis: `joint_state_publisher` Stuck on Startup

**Date:** 2026-03-31  
**Symptom:** Launch hangs at `[joint_state_publisher]: Waiting for robot_description to be published on the robot_description topic...`  
**Environment:** ROS2 Humble, Jetson Orin Nano (Ubuntu 22.04), `rmw_fastrtps_cpp`

---

## What the Code Is Doing

### Launch chain

```
hardware_teleop.launch.py
  └── description.launch.py
        ├── robot_state_publisher  (gets robot_description param → publishes /robot_description topic)
        └── joint_state_publisher  (waits for /robot_description topic)
```

### `description.launch.py` — what we pass

The launch file runs xacro via `subprocess.check_output` at launch time and passes the resulting URDF string as a `robot_description` ROS parameter to both nodes:

```python
robot_description = subprocess.check_output(['xacro', xacro_file]).decode()

Node(package='joint_state_publisher', ...
     parameters=[{'robot_description': robot_description, ...}])
```

This looks correct but **does not work** due to the issue below.

---

## Root Cause

### JSP 2.4.0 ignores the `robot_description` ROS parameter

Inspecting the installed source at `/opt/ros/humble/lib/python3.10/site-packages/joint_state_publisher/joint_state_publisher.py`:

```python
def __init__(self, description_file):   # ← positional CLI arg, not a ROS param
    ...
    if description_file is not None:
        with open(description_file, 'r') as infp:
            description = infp.read()
        self.configure_robot(description)
    else:
        # Falls here whenever no file path is passed on the CLI
        self.get_logger().info('Waiting for robot_description to be published on the robot_description topic...')
        self.create_subscription(std_msgs.msg.String, 'robot_description',
                                 lambda msg: self.configure_robot(msg.data),
                                 rclpy.qos.QoSProfile(depth=1,
                                     durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL))
```

And the entry point:

```python
def main():
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('description_file', nargs='?', default=None)  # ← CLI positional arg
    parsed_args = parser.parse_args(args=stripped_args[1:])
    jsp = JointStatePublisher(parsed_args.description_file)
```

**JSP only reads `description_file` from a positional command-line argument — a file path.** The `robot_description` ROS parameter we pass in the launch file is declared by `automatically_declare_parameters_from_overrides=True` but is never read or used by JSP's logic. It is silently ignored.

Since we don't pass a CLI file argument via `arguments=` in the Node declaration, `description_file` is always `None`, and JSP always falls through to the topic subscription path.

---

## Secondary Issue: FastDDS TRANSIENT_LOCAL Race Condition

Even though JSP correctly subscribes with `TRANSIENT_LOCAL` QoS (matching `robot_state_publisher`'s publisher), the message is not delivered. Verified with `ros2 topic info /robot_description --verbose` while the launch was running:

```
Publisher:    robot_state_publisher  — Durability: TRANSIENT_LOCAL
Subscription: joint_state_publisher  — Durability: TRANSIENT_LOCAL
```

QoS is compatible. Yet JSP never fires its callback.

**Likely cause:** `rmw_fastrtps_cpp` (FastDDS 2.x) has a known race condition where TRANSIENT_LOCAL cache delivery to a late-joining subscriber can silently fail when both nodes start within milliseconds of each other on the same host. The DDS discovery/matching handshake completes after RSP has already written its sample to the durability cache, but FastDDS does not re-deliver the cached sample to the newly-matched JSP subscription in some edge cases.

This is a known upstream issue with FastRTPS/FastDDS TRANSIENT_LOCAL behaviour on loopback.

---

## Odom Topic Not Available

The user observed `/odom` not appearing even though `cobra_driver_node` initialised successfully.

From `cobra_driver_node.cpp`:

```cpp
bool odom_initialized_{false};

void updateOdometry(double odl_cm, double odr_cm) {
    if (!odom_initialized_) {
        // Set initial pose to zero on first feedback
        odom_initialized_ = true;
        ...
        return;  // ← does NOT publish on the very first call
    }
    ...
    odom_pub_->publish(odom_msg);  // Only publishes from second call onward
}
```

Odom is only published after **at least two feedback JSON packets** have been received from the ESP32 over serial. If the ESP32 is not actively sending feedback (e.g. motors idle, no movement), `parseFeedback` may not be called frequently enough to trigger odom publication.

Additionally, if the overall system is stuck (JSP blocking TF), downstream nodes relying on a complete TF tree will report topics as unavailable even if the publisher exists.

---

## Summary of Issues

| # | Issue | Location | Impact |
|---|-------|----------|--------|
| 1 | JSP ignores `robot_description` ROS parameter | `joint_state_publisher` 2.4.0 source | JSP always waits on topic |
| 2 | FastDDS TRANSIENT_LOCAL race condition | `rmw_fastrtps_cpp` middleware | JSP topic subscription never receives cached message |
| 3 | `odom_initialized_` skips first publish | `cobra_driver_node.cpp:218` | `/odom` not published until second ESP32 feedback packet |

---

## Potential Fixes

### Fix 1 — Pass URDF as a CLI argument to JSP (correct API usage)

Write the xacro output to a temp file in the launch script and pass it as `arguments=` to the JSP Node:

```python
import subprocess, tempfile

urdf_content = subprocess.check_output(['xacro', xacro_file]).decode()
urdf_tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
urdf_tmp.write(urdf_content)
urdf_tmp.flush()

Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    arguments=[urdf_tmp.name],   # ← this is what JSP actually reads
    ...
)
```

This bypasses both the parameter issue and the topic/DDS issue entirely.

**Pros:** Simple, correct per JSP API, no topic dependency.  
**Cons:** Leaves a temp file on disk (manageable).

---

### Fix 2 — Switch DDS to CycloneDDS

FastDDS TRANSIENT_LOCAL issues are well-documented. CycloneDDS has more reliable latched-topic behaviour:

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Or set permanently in `/etc/environment` or `~/.bashrc`.

**Pros:** Fixes the DDS race condition broadly, may improve other reliability issues.  
**Cons:** Doesn't fix Issue #1 (parameter still ignored). CycloneDDS has its own quirks.

---

### Fix 3 — Remove JSP and add joint state publishing to `cobra_driver`

Since this is a hardware-only robot, the wheel joint positions should ideally come from encoder feedback, not synthetic defaults. Add `/joint_states` publishing to `cobra_driver_node.cpp` using actual wheel encoder data.

This eliminates the need for `joint_state_publisher` entirely.

**Pros:** Accurate joint states, correct TF tree, no JSP dependency.  
**Cons:** Requires changes to `cobra_driver_node.cpp` — needs encoder-to-joint-angle conversion.

---

### Fix 4 — Add a `TimerAction` delay before JSP starts

Use `launch.actions.TimerAction` to delay JSP startup by 2 seconds, giving RSP time to fully publish and for DDS discovery to stabilise:

```python
from launch.actions import TimerAction

TimerAction(period=2.0, actions=[
    Node(package='joint_state_publisher', ...)
])
```

**Pros:** Quick, no source changes.  
**Cons:** Fragile — a timing hack, not a real fix. May still fail under load.

---

## Recommended Fix

**Fix 1 (temp file + `arguments=`)** is the most correct and reliable solution for the current codebase without changing the cobra_driver. It directly uses JSP's documented API, eliminates both the parameter issue and the DDS race condition, and is straightforward to implement.

**Fix 3** is the right long-term solution once encoder data is available from the cobra driver.
