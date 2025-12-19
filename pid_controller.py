#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Jacob Munoz
# Date: 9/12/25

# Import of python modules.
import math # use of pi.
import random # use for generating a random real number

# import of relevant libraries.
import rclpy # module for ROS APIs
from rclpy.node import Node
from geometry_msgs.msg import Twist # message type for cmd_vel
from sensor_msgs.msg import LaserScan # message type for scan
from nav_msgs.msg import Odometry

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion
from tf2_geometry_msgs import tf2_geometry_msgs
import tf_transformations

# NOTE: there might be some other libraries that can be useful
# as seen in lec02_example_go_forward.py, e.g., Duration

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan' # name of topic for Stage simulator. For Gazebo, 'scan'
DEFAULT_POSE_TOPIC = 'ground_truth' # name of the topic for the ground truth of the pose of the robot. 

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (TODO: feel free to tune)
LINEAR_VELOCITY = 0.25 # m/s
ANGULAR_VELOCITY = math.pi/8 # rad/s

# Threshold of minimum clearance distance (TODO: feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.35 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (TODO: feel free to tune)
# Note: these angles are with respect to the robot perspective, but needs to be
# converted to match how the laser is mounted.
MIN_SCAN_ANGLE_RAD = -85.0 / 180 * math.pi
MAX_SCAN_ANGLE_RAD = +85.0 / 180 * math.pi

USE_SIM_TIME = True

class Pose:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

class PIDController(Node):
    pose=Pose(0,0,0)
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
        scan_angle=[MIN_SCAN_ANGLE_RAD, MAX_SCAN_ANGLE_RAD],
        node_name="pid_controller", context=None):
        """Constructor."""
        super().__init__(node_name, context=context)

        # Workaround not to use roslaunch
        use_sim_time_param = rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            USE_SIM_TIME
        )
        self.set_parameters([use_sim_time_param])

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = self.create_publisher(Twist, DEFAULT_CMD_VEL_TOPIC, 1)
        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = self.create_subscription(LaserScan, DEFAULT_SCAN_TOPIC, self._laser_callback, 1)
        # Setting up subscriber receiving messages from the odometry.
        self._pose_sub = self.create_subscription(Odometry, DEFAULT_POSE_TOPIC, self._pose_callback, 1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.scan_angle = scan_angle

        # Rate at which to operate the while loop.
        self.rate = self.create_rate(FREQUENCY)

        # ___my added variables___
        self.xd = 1.0
        self.robot_on_line = False
        self.closest_x = None
        self.previous_error = 0


    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _pose_callback(self, msg):
        global x, y, heading, t

        # Position
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Get orientation (quaternion)
        orientation_q = msg.pose.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Convert quaternion to roll, pitch, yaw (heading)
        roll, pitch, self.pose.heading = tf_transformations.euler_from_quaternion(quaternion)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: index 0 corresponds to min_angle, 
        #       index 1 corresponds to min_angle + angle_inc
        #       index 2 corresponds to min_angle + angle_inc * 2
        #       ...

        self.get_logger().info(f"Testing")

            # Find the minimum range value between min_scan_angle and max_scan_angle
            # If the minimum range value found is closer to min_threshold_distance, change the flag self._close_obstacle
            # Note: You have to find the min index and max index.
            
            # Please double check the LaserScan message https://docs.ros2.org/foxy/api/sensor_msgs/msg/LaserScan.html
            ####### TODO: ANSWER CODE BEGIN #######

        min_index = int( (self.scan_angle[0] - msg.angle_min) / (msg.angle_increment) )
        max_index = int( (self.scan_angle[1] - msg.angle_min) / (msg.angle_increment) )

        desired_scan_area = msg.ranges[min_index:max_index]


        min_distance = 10
        min_distance_index = None
        for i,distance in enumerate(desired_scan_area):
            if distance < min_distance:
                min_distance = distance
                min_distance_index = i

        if min_distance_index is not None and min_distance < 5:
            self.closest_x = min_distance
            #self.angle = (min_index + min_distance_index)*msg.angle_increment + msg.angle_min + self.pose.heading
            self.angle = (min_index + min_distance_index)*msg.angle_increment + msg.angle_min

            print(self.closest_x)
            print(self.angle * 180/math.pi)
        else:
            self.get_logger().warn("No valid obstacle detected in scan window")
        

            ####### ANSWER CODE END #######

    def spin(self):
        while rclpy.ok():
            # Keep looping until user presses Ctrl+C
            
            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random angle (use random.uniform() to generate a random value)
            # after which the flag is set again to False.
            # Use the function move to publish velocities already implemented,
            # passing the default velocities saved in the corresponding class members.

            ####### TODO: ANSWER CODE BEGIN #######
            if self.closest_x is not None:
                dx = (self.closest_x-self.xd)*math.cos(self.angle)
                dy = (self.closest_x-self.xd)*math.sin(self.angle)

                heading_error = math.atan2(dy,dx)

                # P-Control
                p_term = 0.2*heading_error
                
                # I-Control
                dt = 0.1 # 10Hz
                I_term = 0.1*heading_error*dt

                # D-Control
                D_term = 0.2*(heading_error-self.previous_error)/dt
                self.previous_error = heading_error

                # PID-Control
                self.angular_velocity = p_term + I_term + D_term

                self.move(self.linear_velocity,self.angular_velocity)

                # #Debugging
                # print("DEBUGGING:START")
                # print(f"p-term: {p_term}")
                # print(f"I-term: {I_term}")
                # print(f"D-term: {D_term}")
                # print("DEBUGGING:END")
                

           

            ####### ANSWER CODE END #######

            # operating at the set frequency
            # https://robotics.stackexchange.com/questions/96684/rate-and-sleep-function-in-rclpy-library-for-ros2
            rclpy.spin_once(self)
            

        

def main(args=None):
    """Main function."""

    # 1st. initialization of node.
    rclpy.init(args=args)

    # Initialization of the class for the random walk.
    pid_controller = PIDController()

    interrupted = False

    # Robot random walks.
    try:
        pid_controller.spin()
    except KeyboardInterrupt:
        interrupted = True
        pid_controller.get_logger().error("ROS node interrupted.")
    finally:
        # workaround to send a stop
        # thread.join()
        if rclpy.ok():
            pid_controller.stop()

    if interrupted:
        new_context = rclpy.Context()
        rclpy.init(context=new_context)
        pid_controller = PIDController(node_name="pid_controller_end", context=new_context)
        pid_controller.get_logger().error("ROS node interrupted.")
        pid_controller.stop()
        rclpy.try_shutdown()


if __name__ == "__main__":
    """Run the main function."""
    main()
