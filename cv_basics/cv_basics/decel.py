import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

class Turtle_PID_Controller(Node):
    def __init__(self):
        super().__init__("turtle_pid_controller")
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pose = Pose()
        self.goal = Pose()
        self.kp = 1.0  # Proportional gain
        self.ki = 0.1  # Integral gain
        self.kd = 0.01 # Derivative gain
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_acceleration = 0.2  # Max linear acceleration
        self.max_deceleration = -0.2  # Max linear deceleration
        self.current_linear_velocity = 0.0  # Current linear velocity of the turtle

    def pose_callback(self, data):
        self.pose = data

    def set_goal(self, x, y, theta):
        self.goal.x = x
        self.goal.y = y
        self.goal.theta = theta

    def control_loop(self):
        if not hasattr(self, 'goal'):
            return

        distance_to_goal = math.sqrt((self.goal.x - self.pose.x)**2 + (self.goal.y - self.pose.y)**2)
        angle_to_goal = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)

        distance_tolerance = 0.1
        angle_tolerance = 0.01

        angle_error = self.normalize_angle(angle_to_goal - self.pose.theta)

        # Calculate the integral term
        self.integral += angle_error

        # Calculate the derivative term
        derivative = angle_error - self.prev_error

        # Calculate control input using PID for angular velocity
        angular_velocity = self.kp * angle_error + self.ki * self.integral + self.kd * derivative

        # Calculate control input for linear velocity
        linear_velocity = min(distance_to_goal, 1.0)  # Limit max speed

        # Implement acceleration and deceleration constraints
        velocity_change = linear_velocity - self.current_linear_velocity
        if velocity_change > 0:
            # Accelerating
            velocity_change = min(velocity_change, self.max_acceleration)
        else:
            # Decelerating
            velocity_change = max(velocity_change, self.max_deceleration)

        # Update the current velocity
        self.current_linear_velocity += velocity_change

        # Update the control command
        control_command = Twist()
        if abs(angle_error) > angle_tolerance:
            control_command.angular.z = angular_velocity
            self.current_linear_velocity = 0.0  # Reset linear velocity when turning
        elif distance_to_goal > distance_tolerance:
            control_command.linear.x = self.current_linear_velocity

        self.cmd_vel_pub.publish(control_command)
        self.prev_error = angle_error

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def at_goal(self):
        """
        Check if the turtle is close enough to the goal position and orientation.
        """
        distance_tolerance = 0.1
        angle_tolerance = 0.1
        distance_to_goal = math.sqrt((self.goal.x - self.pose.x)**2 + (self.goal.y - self.pose.y)**2)
        angle_to_goal = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)
        angle_error = self.normalize_angle(angle_to_goal - self.pose.theta)

        return distance_to_goal < distance_tolerance and abs(angle_error) < angle_tolerance


def main(args=None):
    rclpy.init(args=args)
    turtle_controller = Turtle_PID_Controller()

    waypoints = [
        (2.0, 2.0, 0.0), (9.0, 2.0, 0.0), (9.0, 4.0, 0.0),
        (2.0, 4.0, 0.0), (2.0, 6.0, 0.0), (9.0, 6.0, 0.0),
        (9.0, 8.0, 0.0), (2.0, 8.0, 0.0), (2.0, 10.0, 0.0),
        (9.0, 10.0, 0.0), (9.0, 12.0, 0.0), (2.0, 12.0, 0.0)
    ]

    for waypoint in waypoints:
        x, y, theta = waypoint
        turtle_controller.set_goal(x, y, theta)

        while not turtle_controller.at_goal():
            rclpy.spin_once(turtle_controller)

    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
