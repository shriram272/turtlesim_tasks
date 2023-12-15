import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import math

class Turtle_GTG(Node):
    def __init__(self):
        super().__init__("Go_to_Goal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        self.pose = Pose()

        # PID coefficients
        self.kp = 10.0
        self.ki = 0.1
        self.kd = 0.01

        # PID error terms
        self.prev_angle_error = 0.0
        self.angle_integral = 0.0

        # Goal position
        self.goal = Pose()
        self.goal.x = float(sys.argv[1])
        self.goal.y = float(sys.argv[2])
        self.goal.theta = float(sys.argv[3])

    def pose_callback(self, data):
        self.pose = data

    def go_to_goal(self):
        distance_to_goal = math.sqrt((self.goal.x - self.pose.x)**2 + (self.goal.y - self.pose.y)**2)
        angle_to_goal = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)

        distance_tolerance = 0.1
        angle_tolerance = 0.01

        angle_error = angle_to_goal - self.pose.theta
        # Normalize angle_error to the range [-π, π]
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

        # PID calculations
        self.angle_integral += angle_error
        angle_derivative = angle_error - self.prev_angle_error

        angular_z = (self.kp * angle_error) + (self.ki * self.angle_integral) + (self.kd * angle_derivative)
        self.prev_angle_error = angle_error

        new_vel = Twist()
        if abs(angle_error) > angle_tolerance:
            new_vel.angular.z = angular_z
        else:
            if distance_to_goal >= distance_tolerance:
                new_vel.linear.x = self.kp * distance_to_goal
            else:
                new_vel.linear.x = 0.0
                self.get_logger().info("Goal Reached")
                self.timer.cancel()  # Optionally stop the timer

        self.cmd_vel_pub.publish(new_vel)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 4:
        print("Usage: go_to_goal.py <goal_x> <goal_y> <goal_theta>")
        return
    minimal_publisher = Turtle_GTG()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
