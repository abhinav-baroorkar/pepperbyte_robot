#!/usr/bin/env python3
# Copyright (c) 2026 Peppermint Robotics. All rights reserved.
#
# Relay node: subscribes to /map, replaces unknown cells (-1) with free (0),
# and republishes on /map_clean so RViz shows walls only — no pink blob.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MapCleaner(Node):
    def __init__(self):
        super().__init__('map_cleaner')
        self.pub = self.create_publisher(OccupancyGrid, '/map_clean', LATCHED)
        self.create_subscription(OccupancyGrid, '/map', self._cb, LATCHED)

    def _cb(self, msg: OccupancyGrid):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = [70 if v == -1 else v for v in msg.data]
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(MapCleaner())


if __name__ == '__main__':
    main()
