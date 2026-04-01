#!/usr/bin/env python3
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# PepperByte Control App — Flask backend with rclpy ROS2 bridge.
# Serves the web UI at http://localhost:5000 and pushes live ROS2
# data (map, pose, scan) to the browser via Flask-SocketIO.

import os
import math
import time
import base64
import threading
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool

from flask import Flask, jsonify, request, render_template
from flask_socketio import SocketIO

# ---------------------------------------------------------------------------
# Flask setup
# ---------------------------------------------------------------------------
app = Flask(
    __name__,
    static_folder='static',
    static_url_path='/static',
    template_folder='templates',
)
app.config['SECRET_KEY'] = 'pepperbyte'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------
state = {
    'mode': 'teleop',
    'mapping_active': False,
    'mapping_paused': False,
    'map_name': '',
    'save_folder': os.path.expanduser('~/maps'),
    'linear_vel': 0.0,
    'angular_vel': 0.0,
    'pose_x': 0.0,
    'pose_y': 0.0,
    'pose_theta': 0.0,
}

ros_node = None

# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------

class PepperByteNode(Node):
    """Subscribes to key ROS2 topics and pushes data via SocketIO."""

    def __init__(self):
        super().__init__('pepperbyte_app')

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        sensor_qos = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, map_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, sensor_qos)
        self.vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.vel_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # slam_toolbox pause service client
        self.pause_cli = self.create_client(
            SetBool, '/slam_toolbox/pause_new_measurements')

        # Rate limiters (wall-clock seconds)
        self._last_map = 0.0
        self._last_pose = 0.0
        self._last_scan = 0.0

        self.create_timer(1.0, self._status_tick)
        self.get_logger().info('PepperByte App ROS2 node started')

    # -- callbacks ----------------------------------------------------------

    def map_cb(self, msg):
        now = time.time()
        if now - self._last_map < 5.0:
            return
        self._last_map = now

        w, h = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        # Convert signed int8 → unsigned byte; -1 (unknown) → 255
        data_b64 = base64.b64encode(
            bytes(d & 0xFF for d in msg.data)
        ).decode('ascii')

        socketio.emit('map_update', {
            'width': w, 'height': h,
            'resolution': res,
            'origin_x': ox, 'origin_y': oy,
            'data': data_b64,
        })

    def odom_cb(self, msg):
        now = time.time()
        if now - self._last_pose < 0.1:
            return
        self._last_pose = now

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        state['pose_x'] = p.x
        state['pose_y'] = p.y
        state['pose_theta'] = theta
        state['linear_vel'] = msg.twist.twist.linear.x
        state['angular_vel'] = msg.twist.twist.angular.z

        socketio.emit('pose_update', {
            'x': p.x, 'y': p.y, 'theta': theta,
        })

    def scan_cb(self, msg):
        now = time.time()
        if now - self._last_scan < 0.2:
            return
        self._last_scan = now

        pts = []
        for i in range(0, len(msg.ranges), 5):          # downsample ×5
            r = msg.ranges[i]
            if r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            pts.append([round(r * math.cos(a), 3),
                        round(r * math.sin(a), 3)])

        socketio.emit('scan_update', {'points': pts})

    def vel_cb(self, msg):
        state['linear_vel'] = msg.linear.x
        state['angular_vel'] = msg.angular.z

    def _status_tick(self):
        socketio.emit('status_update', {
            'mode':           state['mode'],
            'linear_vel':     round(state['linear_vel'], 3),
            'angular_vel':    round(state['angular_vel'], 3),
            'mapping_active': state['mapping_active'],
            'mapping_paused': state['mapping_paused'],
            'map_name':       state['map_name'],
            'save_folder':    state['save_folder'],
        })

    # -- helpers ------------------------------------------------------------

    def set_slam_pause(self, paused):
        """Call slam_toolbox pause_new_measurements service.
        paused=True  → stop processing new scans (freeze map).
        paused=False → resume processing (continue mapping from current pose).
        """
        if not self.pause_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('slam_toolbox pause service not available')
            return False, 'slam_toolbox service not available'
        req = SetBool.Request()
        req.data = paused
        future = self.pause_cli.call_async(req)
        # Spin until complete (called from Flask thread, not rclpy executor)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is not None:
            resp = future.result()
            self.get_logger().info(
                f'slam_toolbox pause={paused}: success={resp.success} '
                f'msg="{resp.message}"')
            return resp.success, resp.message
        return False, 'Service call timed out'

    def restart_slam(self):
        """Kill slam_toolbox and relaunch it for a completely fresh map."""
        from ament_index_python.packages import get_package_share_directory

        slam_share = get_package_share_directory('pepperbyte_slam')
        slam_config = os.path.join(
            slam_share, 'config', 'slam_toolbox_online_async.yaml')

        # Gracefully kill existing slam_toolbox (SIGINT)
        self.get_logger().info('Killing slam_toolbox for restart...')
        subprocess.run(
            ['pkill', '-INT', '-f', 'async_slam_toolbox'],
            timeout=5, capture_output=True)
        time.sleep(3)  # let it shut down

        # Relaunch with same params
        self.get_logger().info('Relaunching slam_toolbox...')
        subprocess.Popen(
            ['ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
             '--ros-args',
             '--params-file', slam_config,
             '-p', 'use_sim_time:=true'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(1)
        self.get_logger().info('slam_toolbox restarted — fresh map')
        return True, 'Map cleared, mapping restarted'

    def send_goal(self, x, y, theta):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(float(theta) / 2.0)
        msg.pose.orientation.w = math.cos(float(theta) / 2.0)
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Goal published: ({x:.2f}, {y:.2f}, {theta:.2f})')


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    return jsonify(state)


@app.route('/api/start_mapping', methods=['POST'])
def api_start_mapping():
    body = request.get_json(silent=True) or {}
    state['map_name'] = body.get('map_name', 'untitled')
    state['save_folder'] = body.get('save_folder', os.path.expanduser('~/maps'))
    state['mapping_active'] = True
    state['mapping_paused'] = False
    # Mode is chosen once at start and locked for the session
    mode = body.get('mode', 'mapping')
    if mode not in ('mapping', 'autonomous'):
        mode = 'mapping'
    state['mode'] = mode
    # Ensure slam_toolbox is unpaused
    if ros_node:
        ros_node.set_slam_pause(False)
    return jsonify({'status': 'ok', 'message': 'Mapping started'})


@app.route('/api/stop_mapping', methods=['POST'])
def api_stop_mapping():
    """Pause slam_toolbox — freezes the map but keeps the session alive."""
    state['mapping_paused'] = True
    msg = 'Mapping paused'
    if ros_node:
        ok, svc_msg = ros_node.set_slam_pause(True)
        if not ok:
            msg = f'Pause requested but service said: {svc_msg}'
    return jsonify({'status': 'ok', 'message': msg})


@app.route('/api/resume_mapping', methods=['POST'])
def api_resume_mapping():
    """Resume slam_toolbox from the current robot location."""
    state['mapping_paused'] = False
    state['mapping_active'] = True
    msg = 'Mapping resumed'
    if ros_node:
        ok, svc_msg = ros_node.set_slam_pause(False)
        if not ok:
            msg = f'Resume requested but service said: {svc_msg}'
    return jsonify({'status': 'ok', 'message': msg})


@app.route('/api/restart_mapping', methods=['POST'])
def api_restart_mapping():
    """Kill slam_toolbox, restart it, and begin mapping from scratch."""
    body = request.get_json(silent=True) or {}
    state['map_name'] = body.get('map_name', state['map_name']) or 'untitled'
    state['save_folder'] = body.get('save_folder', state['save_folder'])

    if ros_node is None:
        return jsonify({'status': 'error', 'message': 'ROS node not ready'}), 503

    ok, msg = ros_node.restart_slam()
    if ok:
        state['mapping_active'] = True
        state['mapping_paused'] = False
        # Keep the mode the user originally chose — don't override it
        return jsonify({'status': 'ok', 'message': msg})
    return jsonify({'status': 'error', 'message': msg}), 500


@app.route('/api/save_map', methods=['POST'])
def api_save_map():
    body = request.get_json(silent=True) or {}
    name = body.get('map_name', state['map_name']) or 'untitled'
    folder = body.get('save_folder', state['save_folder'])
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, name)

    try:
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', path, '--ros-args', '-p', 'save_map_timeout:=10000'],
            capture_output=True, text=True, timeout=30,
        )
        if result.returncode == 0:
            return jsonify({'status': 'ok', 'message': f'Map saved to {path}'})
        return jsonify({'status': 'error', 'message': result.stderr}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500


@app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    body = request.get_json(silent=True) or {}
    mode = body.get('mode', 'teleop')
    if mode not in ('teleop', 'mapping', 'autonomous'):
        return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400
    # Only allow mode change when no mapping session is running
    if state['mapping_active']:
        return jsonify({'status': 'error',
                        'message': 'Cannot change mode during active session'}), 400
    state['mode'] = mode
    return jsonify({'status': 'ok', 'mode': mode})


@app.route('/api/maps')
def api_maps():
    folder = request.args.get('folder', state['save_folder'])
    maps = []
    if os.path.isdir(folder):
        maps = sorted(f for f in os.listdir(folder) if f.endswith('.yaml'))
    return jsonify({'folder': folder, 'maps': maps})


@app.route('/api/send_goal', methods=['POST'])
def api_send_goal():
    body = request.get_json(silent=True) or {}
    if ros_node is None:
        return jsonify({'status': 'error', 'message': 'ROS node not ready'}), 503
    ros_node.send_goal(body.get('x', 0.0), body.get('y', 0.0),
                       body.get('theta', 0.0))
    return jsonify({'status': 'ok'})


# ---------------------------------------------------------------------------
# ROS2 spin thread
# ---------------------------------------------------------------------------

def _ros_spin():
    global ros_node
    rclpy.init()
    ros_node = PepperByteNode()
    try:
        rclpy.spin(ros_node)
    except Exception:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    threading.Thread(target=_ros_spin, daemon=True).start()

    print('\n' + '=' * 52)
    print('   Peppermint Robotics Control App')
    print('   http://localhost:5000')
    print('=' * 52 + '\n')

    socketio.run(app, host='0.0.0.0', port=5000,
                 allow_unsafe_werkzeug=True)
