#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WorkshopBasicNode(Node):
    def __init__(self):
        super().__init__("workshop_basic_node")

        # SENSE
        # Subscribe to the laser scan topic
        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 5)

        # Publish to /robot_base_controller/cmd_vel_unstamped
        self.robot_vel_pub = self.create_publisher(Twist, "/robot_base_controller/cmd_vel_unstamped", 10)

    # THINK
    def lidar_callback(self, scan_data):
        
        # Get all points of the scan data
        # num_points = len(scan_data.ranges)

        # Extract scan region we are interested in
        # fwd_range = scan_data.ranges[190:210]
        fwd_range = scan_data.ranges[185:215]

        # Compute an average distance to plants measurement
        distance_to_plants = sum(fwd_range)/len(fwd_range)

        approach_threshold = 1.0 #metres

        # Conditino: if distance to the plants is above 1 m, move forward
        if distance_to_plants > approach_threshold:
            move_flag = True
            self.get_logger().info("Distance to plants: %.3f" % (distance_to_plants))
        else:
            move_flag = False
            self.get_logger().info("Distance to plants: %.3f" % (distance_to_plants))
        
        self.act(move_flag)

    def act(self, move_flag):

        # Create twist object
        move_cmd = Twist()
        
        # Condition: if move_flag is true, move robot
        if move_flag:
            move_cmd.linear.x = 0.5 #m/s
        else:
            move_cmd.linear.x = 0.0
        
        # Publish velocity message to robot
        self.robot_vel_pub.publish(move_cmd)



def main(args=None):
    rclpy.init(args=args)
    
    workshop_basic_node = WorkshopBasicNode()
    rclpy.spin(workshop_basic_node)

    workshop_basic_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
