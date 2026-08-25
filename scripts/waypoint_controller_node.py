#!/usr/bin/env python3
import rospy
import math
import tf2_ros
import tf.transformations

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class PathFollowerController:
    def __init__(self):
        rospy.init_node('waypoint_controller_node', anonymous=False)

        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 0.25)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 0.20)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.8)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_cb)

        self.path_poses = []
        self.current_target_idx = 0
        self.is_active = False

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)
        rospy.loginfo("[WaypointController] Ready & Listening to /planned_path...")

    def path_cb(self, msg):
        if not msg.poses:
            return
        self.path_poses = msg.poses
        self.current_target_idx = 0
        self.is_active = True
        rospy.loginfo(f"[WaypointController] Path received! Total Points: {len(self.path_poses)}. Engaging motors...")

    def get_robot_pose(self):
        for b_frame in [self.base_frame, 'base_link']:
            try:
                trans = self.tf_buffer.lookup_transform(self.frame_id, b_frame, rospy.Time(0), rospy.Duration(0.05))
                x = trans.transform.translation.x
                y = trans.transform.translation.y
                q = trans.transform.rotation
                _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
                return x, y, yaw
            except Exception:
                continue
        return None, None, None

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def control_loop(self, event):
        if not self.is_active or not self.path_poses:
            return

        rx, ry, ryaw = self.get_robot_pose()
        if rx is None:
            rospy.logwarn_throttle(2.0, "[WaypointController] Waiting for TF lookup between map and base_footprint...")
            return

        # 1. ตรวจสอบระยะถึงสถานีเป้าหมายปลายทาง
        final_pose = self.path_poses[-1].pose
        dist_to_final = math.hypot(final_pose.position.x - rx, final_pose.position.y - ry)

        if dist_to_final <= self.goal_tolerance:
            # หมุนหัวรถเทียบจอดตาม Quaternion ปลายทาง
            fq = final_pose.orientation
            _, _, final_yaw = tf.transformations.euler_from_quaternion([fq.x, fq.y, fq.z, fq.w])
            yaw_err = math.atan2(math.sin(final_yaw - ryaw), math.cos(final_yaw - ryaw))

            if abs(yaw_err) > 0.1:
                cmd = Twist()
                cmd.angular.z = max(min(yaw_err * 1.5, self.max_angular_speed), -self.max_angular_speed)
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.is_active = False
                rospy.loginfo("[WaypointController] REACHED DESTINATION! Robot Stopped.")
            return

        # 2. เลื่อนหาจุด Target Waypoint ถัดไปข้างหน้า
        while self.current_target_idx < len(self.path_poses) - 1:
            target = self.path_poses[self.current_target_idx].pose
            d = math.hypot(target.position.x - rx, target.position.y - ry)
            if d < self.lookahead_dist:
                self.current_target_idx += 1
            else:
                break

        target_pose = self.path_poses[self.current_target_idx].pose
        tx = target_pose.position.x
        ty = target_pose.position.y

        # 3. คำนวณเวกเตอร์ความเร็ว (Pure Pursuit)
        dx = tx - rx
        dy = ty - ry
        target_angle = math.atan2(dy, dx)
        angle_err = math.atan2(math.sin(target_angle - ryaw), math.cos(target_angle - ryaw))

        cmd = Twist()
        if abs(angle_err) > 0.6:
            cmd.linear.x = 0.03
            cmd.angular.z = max(min(angle_err * 2.0, self.max_angular_speed), -self.max_angular_speed)
        else:
            cmd.linear.x = min(self.max_linear_speed, 0.4 * math.hypot(dx, dy) + 0.06)
            cmd.angular.z = max(min(angle_err * 2.2, self.max_angular_speed), -self.max_angular_speed)

        self.cmd_pub.publish(cmd)
        rospy.loginfo_throttle(1.0, f"[Driving] Towards WP #{self.current_target_idx} | Dist: {math.hypot(dx, dy):.2f}m | Vel: {cmd.linear.x:.2f} m/s")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = PathFollowerController()
        controller.run()
    except rospy.ROSInterruptException:
        pass