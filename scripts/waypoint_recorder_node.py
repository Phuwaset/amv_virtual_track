#!/usr/bin/env python3
import rospy
import math
import os
import yaml

from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse

def normalize_quaternion(qx, qy, qz, qw):
    """Normalize Quaternion ป้องกัน Error/Warning ใน RViz"""
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm

class WaypointRecorderNode:
    def __init__(self):
        rospy.init_node('waypoint_recorder_node', anonymous=False)

        # 1. ไฟล์คอนฟิก YAML
        self.yaml_path = rospy.get_param('~yaml_path', os.path.expanduser('~/turtlebot3_ws/src/amv_virtual_track/config/waypoints.yaml'))
        self.frame_id = rospy.get_param('~frame_id', 'map')

        self.waypoints = []
        self.load_yaml()

        # 2. Publisher & Subscriber
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=10, latch=True)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # 3. Services
        self.srv_undo = rospy.Service('/amv/recorder/undo', Trigger, self.srv_undo_cb)
        self.srv_clear = rospy.Service('/amv/recorder/clear', Trigger, self.srv_clear_cb)
        self.srv_save = rospy.Service('/amv/recorder/save', Trigger, self.srv_save_cb)

        self.publish_markers()
        rospy.loginfo(f"[WaypointRecorder] Ready. Managing YAML: {self.yaml_path}")

    def load_yaml(self):
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data and 'waypoints' in data and isinstance(data['waypoints'], list):
                    for wp in data['waypoints']:
                        qx, qy, qz, qw = normalize_quaternion(
                            float(wp.get('qx', 0.0)),
                            float(wp.get('qy', 0.0)),
                            float(wp.get('qz', 0.0)),
                            float(wp.get('qw', 1.0))
                        )
                        self.waypoints.append({
                            'name': str(wp.get('name', '')).strip(),
                            'mode': str(wp.get('mode', 'Auto')).strip(),
                            'timer': float(wp.get('timer', 0.0)),
                            'pin': str(wp.get('pin', 'Up')).strip(),
                            'x': float(wp.get('x', 0.0)),
                            'y': float(wp.get('y', 0.0)),
                            'z': float(wp.get('z', 0.0)),
                            'qx': qx,
                            'qy': qy,
                            'qz': qz,
                            'qw': qw
                        })
            rospy.loginfo(f"[WaypointRecorder] Loaded {len(self.waypoints)} waypoints from YAML.")

    def save_yaml(self):
        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        data = {'waypoints': self.waypoints}
        with open(self.yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    def publish_markers(self):
        marker_arr = MarkerArray()
        now = rospy.Time.now()

        # เคลียร์ Marker ทั้งหมดใน RViz ก่อน
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        clear_marker.pose.orientation.w = 1.0
        marker_arr.markers.append(clear_marker)

        # หากไม่มีจุดในระบบ ให้ส่งเฉพาะ DELETEALL แล้วจบฟังก์ชัน (แก้ Error: Points should not be empty)
        if not self.waypoints:
            self.marker_pub.publish(marker_arr)
            return

        # 1. จุดกลมสีฟ้า (SPHERE_LIST)
        dots = Marker()
        dots.header.frame_id = self.frame_id
        dots.header.stamp = now
        dots.ns = "wp_dots"
        dots.id = 1
        dots.type = Marker.SPHERE_LIST
        dots.action = Marker.ADD
        dots.pose.orientation.w = 1.0  # ป้องกัน Uninitialized quaternion Warning
        dots.scale.x = dots.scale.y = dots.scale.z = 0.08
        dots.color.r, dots.color.g, dots.color.b, dots.color.a = 0.0, 0.7, 1.0, 0.95

        for i, wp in enumerate(self.waypoints):
            dots.points.append(Point(x=wp['x'], y=wp['y'], z=0.04))

            # 2. ป้ายข้อความ 3D
            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = now
            txt.ns = "wp_text"
            txt.id = i + 100
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = wp['x']
            txt.pose.position.y = wp['y']
            txt.pose.position.z = 0.22
            txt.pose.orientation.w = 1.0  # ป้องกัน Uninitialized quaternion Warning
            txt.scale.z = 0.11
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 1.0, 1.0, 1.0
            txt.text = f"{wp['name']} ({wp['timer']}s)"
            marker_arr.markers.append(txt)

            # 3. หัวลูกศร 3D สีแดงสด (ARROW)
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = now
            arrow.ns = "wp_arrows"
            arrow.id = i + 200
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = wp['x']
            arrow.pose.position.y = wp['y']
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.x = wp['qx']
            arrow.pose.orientation.y = wp['qy']
            arrow.pose.orientation.z = wp['qz']
            arrow.pose.orientation.w = wp['qw']
            arrow.scale.x = 0.30
            arrow.scale.y = 0.06
            arrow.scale.z = 0.06
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.1, 0.1, 1.0
            marker_arr.markers.append(arrow)

        marker_arr.markers.append(dots)
        self.marker_pub.publish(marker_arr)

    def goal_callback(self, msg):
        next_id = len(self.waypoints) + 1
        wp_name = f"wy{next_id}"

        qx, qy, qz, qw = normalize_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        new_wp = {
            'name': wp_name,
            'mode': 'Auto',
            'timer': 0.0,
            'pin': 'Up',
            'x': round(msg.pose.position.x, 3),
            'y': round(msg.pose.position.y, 3),
            'z': 0.0,
            'qx': round(qx, 6),
            'qy': round(qy, 6),
            'qz': round(qz, 6),
            'qw': round(qw, 6)
        }

        self.waypoints.append(new_wp)
        self.save_yaml()
        self.publish_markers()
        rospy.loginfo(f"[Recorded] Added [{wp_name}] at ({new_wp['x']}, {new_wp['y']}). Total: {len(self.waypoints)}")

    def srv_undo_cb(self, req):
        if self.waypoints:
            removed = self.waypoints.pop()
            self.save_yaml()
            self.publish_markers()
            return TriggerResponse(success=True, message=f"Removed: [{removed['name']}]")
        return TriggerResponse(success=False, message="Waypoint list is empty.")

    def srv_clear_cb(self, req):
        self.waypoints = []
        self.save_yaml()
        self.publish_markers()
        return TriggerResponse(success=True, message="Cleared all waypoints in YAML.")

    def srv_save_cb(self, req):
        self.save_yaml()
        return TriggerResponse(success=True, message="Saved YAML.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        recorder = WaypointRecorderNode()
        recorder.run()
    except rospy.ROSInterruptException:
        pass