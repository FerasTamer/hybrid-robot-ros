#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class LyapunovController:

    def __init__(self):
        rospy.init_node('lyapunov_controller', anonymous=False)

        self.k_rho   = rospy.get_param('~k_rho',   0.4)
        self.k_alpha = rospy.get_param('~k_alpha',  1.2)
        self.k_beta  = rospy.get_param('~k_beta',  -0.3)

        self.max_linear  = rospy.get_param('~max_linear',  0.3)
        self.max_angular = rospy.get_param('~max_angular', 1.0)
        self.goal_tol    = rospy.get_param('~goal_tol',    0.05)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (0.0, 0.0),
        ]
        self.current_wp = 0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(20)
        rospy.loginfo("Lyapunov Controller ready.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_received = True

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def compute_control(self, gx, gy):
        dx  = gx - self.x
        dy  = gy - self.y
        rho = math.hypot(dx, dy)
        alpha = self.wrap_angle(math.atan2(dy, dx) - self.yaw)
        beta  = self.wrap_angle(-self.yaw - alpha)
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        v = max(-self.max_linear,  min(self.max_linear,  v))
        w = max(-self.max_angular, min(self.max_angular, w))
        return v, w, rho

    def run(self):
        rospy.loginfo("Waiting for odometry...")
        while not self.odom_received and not rospy.is_shutdown():
            self.rate.sleep()

        while not rospy.is_shutdown():
            if self.current_wp >= len(self.waypoints):
                rospy.loginfo("All waypoints reached!")
                self.stop()
                break

            gx, gy = self.waypoints[self.current_wp]
            rospy.loginfo_throttle(2, "Heading to waypoint %d: (%.1f, %.1f)" % (self.current_wp, gx, gy))

            v, w, rho = self.compute_control(gx, gy)

            if rho < self.goal_tol:
                rospy.loginfo("Waypoint %d reached!" % self.current_wp)
                self.current_wp += 1
                self.stop()
                rospy.sleep(0.5)
                continue

            cmd = Twist()
            cmd.linear.x  = v
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        controller = LyapunovController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
