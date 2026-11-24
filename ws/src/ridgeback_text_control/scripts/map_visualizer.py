#!/usr/bin/env python3
"""Publish visualization-friendly map layers.

RViz paints illegal negative OccupancyGrid values as red/yellow. This node keeps
the original /map untouched and republishes normalized display-only layers.
"""

import argparse
import math

import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker


def normalize_cell(value: int, occupied_threshold: int, free_threshold: int) -> int:
    if value < 0:
        return -1
    if value >= occupied_threshold:
        return 100
    if value <= free_threshold:
        return 0
    return -1


class MapVisualizer:
    def __init__(self, args):
        self.args = args
        self.pub = rospy.Publisher(args.output_topic, OccupancyGrid, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher(args.marker_topic, Marker, queue_size=1, latch=True)
        self.sub = rospy.Subscriber(args.input_topic, OccupancyGrid, self.callback, queue_size=1)

    def callback(self, msg):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = [
            normalize_cell(value, self.args.occupied_threshold, self.args.free_threshold)
            for value in msg.data
        ]
        self.pub.publish(out)
        self.marker_pub.publish(self.to_marker(msg))

    def to_marker(self, msg):
        marker = Marker()
        marker.header = msg.header
        marker.ns = "ridgeback_map_obstacles"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = msg.info.resolution
        marker.scale.y = msg.info.resolution
        marker.color.r = 0.02
        marker.color.g = 0.02
        marker.color.b = 0.02
        marker.color.a = 1.0

        width = msg.info.width
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        yaw = yaw_from_quaternion(msg.info.origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for index, value in enumerate(msg.data):
            if value < self.args.occupied_threshold:
                continue
            cell_x = index % width
            cell_y = index // width
            local_x = (cell_x + 0.5) * resolution
            local_y = (cell_y + 0.5) * resolution

            point = Point()
            point.x = origin_x + local_x * cos_yaw - local_y * sin_yaw
            point.y = origin_y + local_x * sin_yaw + local_y * cos_yaw
            point.z = 0.02
            marker.points.append(point)

        return marker


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    parser = argparse.ArgumentParser(description="Normalize /map for RViz display.")
    parser.add_argument("--input-topic", default="/map")
    parser.add_argument("--output-topic", default="/map_visual")
    parser.add_argument("--marker-topic", default="/map_obstacles")
    parser.add_argument("--occupied-threshold", type=int, default=65)
    parser.add_argument("--free-threshold", type=int, default=25)
    args = parser.parse_args()

    rospy.init_node("ridgeback_map_visualizer", anonymous=False)
    MapVisualizer(args)
    rospy.loginfo(
        "Publishing normalized map topic %s and obstacle marker %s from %s",
        args.output_topic,
        args.marker_topic,
        args.input_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
