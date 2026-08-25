#!/usr/bin/env python3
import rospy
import yaml
import csv
import math
import os
import tf2_ros

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# ตารางเส้นทางตรงตามผังทางแยกจริง (Direct Route Matrix)
# รูปแบบ: (สถานีเริ่มต้น, สถานีเป้าหมาย): [รายชื่อ Waypoints และสถานีระหว่างทาง]
MASTER_ROUTES = {
    # ------------------- ออกจาก Material Room -------------------
    ('material room', 'line1'): ['wy1', 'wy2', 'Line1'],
    ('material room', 'line3'): ['wy1', 'wy2', 'Line1', 'wy3', 'wy4', 'wy5', 'Line3'],
    ('material room', 'line2'): ['wy1', 'wy2', 'Line1', 'wy3', 'wy6', 'Line2'],
    ('material room', 'line4'): ['wy1', 'wy2', 'Line1', 'wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4'],
    ('material room', 'line5'): ['wy1', 'wy2', 'Line1', 'wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5'],
    ('material room', 'line6'): ['wy1', 'wy2', 'Line1', 'wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('material room', 'line7'): ['wy1', 'wy2', 'Line1', 'wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5', 'wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line1 -------------------
    ('line1', 'material room'): ['wy2', 'wy1', 'Material Room'],
    ('line1', 'line3'): ['wy3', 'wy4', 'wy5', 'Line3'],
    ('line1', 'line2'): ['wy3', 'wy6', 'Line2'],
    ('line1', 'line4'): ['wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4'],
    ('line1', 'line5'): ['wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5'],
    ('line1', 'line6'): ['wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('line1', 'line7'): ['wy3', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5', 'wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line3 (ถอยออกจากซอยกลาง) -------------------
    ('line3', 'material room'): ['wy5', 'wy4', 'wy3', 'Line1', 'wy2', 'wy1', 'Material Room'],
    ('line3', 'line1'): ['wy5', 'wy4', 'wy3', 'Line1'],
    ('line3', 'line2'): ['wy5', 'wy4', 'wy6', 'Line2'],
    ('line3', 'line4'): ['wy5', 'wy4', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4'],
    ('line3', 'line5'): ['wy5', 'wy4', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5'],
    ('line3', 'line6'): ['wy5', 'wy4', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('line3', 'line7'): ['wy5', 'wy4', 'wy6', 'Line2', 'wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5', 'wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line2 -------------------
    ('line2', 'material room'): ['wy6', 'wy3', 'Line1', 'wy2', 'wy1', 'Material Room'],
    ('line2', 'line1'): ['wy6', 'wy3', 'Line1'],
    ('line2', 'line3'): ['wy6', 'wy4', 'wy5', 'Line3'],
    ('line2', 'line4'): ['wy7', 'wy8', 'Line4'],
    ('line2', 'line5'): ['wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5'],
    ('line2', 'line6'): ['wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('line2', 'line7'): ['wy7', 'wy8', 'Line4', 'wy17', 'wy16', 'wy9', 'wy12', 'Line5', 'wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line4 -------------------
    ('line4', 'line2'): ['wy8', 'wy7', 'Line2'],
    ('line4', 'line3'): ['wy8', 'wy7', 'Line2', 'wy6', 'wy4', 'wy5', 'Line3'],
    ('line4', 'line5'): ['wy17', 'wy16', 'wy9', 'wy12', 'Line5'],
    ('line4', 'line6'): ['wy17', 'wy16', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('line4', 'line7'): ['wy17', 'wy16', 'wy9', 'wy12', 'Line5', 'wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line5 -------------------
    ('line5', 'line4'): ['wy12', 'wy9', 'wy16', 'wy17', 'Line4'],
    ('line5', 'line6'): ['wy12', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('line5', 'line7'): ['wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line6 (ถอยออกจากซอยล่าง) -------------------
    ('line6', 'line4'): ['wy11', 'wy10', 'wy18', 'wy16', 'wy17', 'Line4'],
    ('line6', 'line5'): ['wy11', 'wy10', 'wy18', 'wy12', 'Line5'],
    ('line6', 'line7'): ['wy11', 'wy10', 'wy18', 'wy12', 'Line5', 'wy13', 'wy14', 'wy15', 'Line7'],

    # ------------------- ออกจาก Line7 -------------------
    ('line7', 'line5'): ['wy15', 'wy14', 'wy13', 'Line5'],
    ('line7', 'line6'): ['wy15', 'wy14', 'wy13', 'Line5', 'wy12', 'wy18', 'wy10', 'wy11', 'Line6'],
    ('line7', 'line4'): ['wy15', 'wy14', 'wy13', 'Line5', 'wy12', 'wy9', 'wy16', 'wy17', 'Line4']
}

def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm

class ExactCorridorPathPlanner:
    def __init__(self):
        rospy.init_node('path_planner_node', anonymous=False)

        self.station_csv = rospy.get_param('~station_csv', os.path.expanduser('~/turtlebot3_ws/src/amv_virtual_track/config/station_list.csv'))
        self.waypoint_yaml = rospy.get_param('~waypoint_yaml', os.path.expanduser('~/turtlebot3_ws/src/amv_virtual_track/config/waypoints.yaml'))
        self.frame_id = rospy.get_param('~frame_id', 'map')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.cmd_sub = rospy.Subscriber('/amv/command/goto_station', String, self.command_cb)

        self.stations = {}
        self.waypoints_dict = {}
        self.load_data()

        # AMCL Auto-Localization Log (พ่น 2 ครั้งแล้วหยุด)
        self.log_count = 0
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.log_localization_status)

        rospy.loginfo("[PathPlanner] Direct Route Matrix Planner Online. Ready.")

    def load_data(self):
        # โหลด Station จาก CSV
        self.stations = {}
        if os.path.exists(self.station_csv):
            with open(self.station_csv, mode='r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    name = row.get('name', '').strip()
                    if name:
                        self.stations[name.lower()] = {
                            'name': name,
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'z': float(row.get('z', 0.0)),
                            'qx': float(row.get('qx', 0.0)),
                            'qy': float(row.get('qy', 0.0)),
                            'qz': float(row.get('qz', 0.0)),
                            'qw': float(row.get('qw', 1.0))
                        }

        # โหลด Waypoints จาก YAML
        self.waypoints_dict = {}
        if os.path.exists(self.waypoint_yaml):
            with open(self.waypoint_yaml, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
                if data and 'waypoints' in data and isinstance(data['waypoints'], list):
                    for wp in data['waypoints']:
                        w_name = str(wp.get('name', '')).strip().lower()
                        qx, qy, qz, qw = normalize_quaternion(
                            float(wp.get('qx', 0.0)),
                            float(wp.get('qy', 0.0)),
                            float(wp.get('qz', 0.0)),
                            float(wp.get('qw', 1.0))
                        )
                        self.waypoints_dict[w_name] = {
                            'name': wp.get('name', ''),
                            'x': float(wp.get('x', 0.0)),
                            'y': float(wp.get('y', 0.0)),
                            'z': float(wp.get('z', 0.0)),
                            'qx': qx,
                            'qy': qy,
                            'qz': qz,
                            'qw': qw
                        }

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, 'base_footprint', rospy.Time(0), rospy.Duration(0.1))
            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
        except Exception:
            return None, None, None, None

    def find_nearest_entities(self, cur_x, cur_y):
        nearest_st, min_st_dist = None, float('inf')
        for k, st in self.stations.items():
            d = math.hypot(st['x'] - cur_x, st['y'] - cur_y)
            if d < min_st_dist:
                min_st_dist = d
                nearest_st = st['name']

        nearest_wp, min_wp_dist = None, float('inf')
        for k, wp in self.waypoints_dict.items():
            d = math.hypot(wp['x'] - cur_x, wp['y'] - cur_y)
            if d < min_wp_dist:
                min_wp_dist = d
                nearest_wp = wp['name']

        return nearest_st, min_st_dist, nearest_wp, min_wp_dist

    def log_localization_status(self, event):
        if self.log_count >= 2:
            self.status_timer.shutdown()
            return

        rx, ry, _, _ = self.get_robot_pose()
        if rx is None:
            return

        self.load_data()
        st_name, st_dist, wp_name, wp_dist = self.find_nearest_entities(rx, ry)

        rospy.loginfo(f"[AMCL Localization #{self.log_count + 1}] Robot: ({rx:.2f}, {ry:.2f}) | Station: [{st_name}] ({st_dist:.2f}m) | Waypoint: [{wp_name}] ({wp_dist:.2f}m)")
        self.log_count += 1

    def command_cb(self, msg):
        target_name = msg.data.strip()
        self.load_data()

        if target_name.lower() not in self.stations:
            rospy.logerr(f"[PathPlanner] Target '{target_name}' not found in station database!")
            return

        rx, ry, r_qz, r_qw = self.get_robot_pose()
        if rx is None:
            rospy.logerr("[PathPlanner] Cannot acquire Robot Pose from AMCL/TF!")
            return

        # 1. หาสถานีตั้งต้นที่ใกล้ตัวรถที่สุด
        nearest_st_name, _, _, _ = self.find_nearest_entities(rx, ry)
        if not nearest_st_name:
            rospy.logerr("[PathPlanner] No stations found!")
            return

        start_key = nearest_st_name.lower()
        goal_key = target_name.lower()

        # 2. ดึงเส้นทางแบบ Direct Route
        route_keys = MASTER_ROUTES.get((start_key, goal_key), None)
        if route_keys is None:
            rospy.logerr(f"[PathPlanner] No explicit route mapped for: ({start_key} -> {goal_key})")
            return

        # 3. ประกอบโหนดเส้นทาง: [Robot Pose] -> [Waypoints/Stations ตามรายการ]
        final_nodes = []
        final_nodes.append({
            'name': 'Robot_Pose',
            'x': rx, 'y': ry, 'z': 0.0,
            'qx': 0.0, 'qy': 0.0, 'qz': r_qz, 'qw': r_qw
        })

        for key in route_keys:
            k_low = key.lower()
            if k_low in self.waypoints_dict:
                final_nodes.append(self.waypoints_dict[k_low])
            elif k_low in self.stations:
                final_nodes.append(self.stations[k_low])

        # 4. ส่งออก Topic /planned_path
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = rospy.Time.now()

        for node in final_nodes:
            p = PoseStamped()
            p.header.frame_id = self.frame_id
            p.header.stamp = path_msg.header.stamp
            p.pose.position.x = node['x']
            p.pose.position.y = node['y']
            p.pose.position.z = node.get('z', 0.0)
            p.pose.orientation.x = node.get('qx', 0.0)
            p.pose.orientation.y = node.get('qy', 0.0)
            p.pose.orientation.z = node.get('qz', 0.0)
            p.pose.orientation.w = node.get('qw', 1.0)
            path_msg.poses.append(p)

        self.path_pub.publish(path_msg)
        seq_names = [n['name'] for n in final_nodes]
        rospy.loginfo(f"[PathPlanner] Generated Exact Path -> {' -> '.join(seq_names)}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = ExactCorridorPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass