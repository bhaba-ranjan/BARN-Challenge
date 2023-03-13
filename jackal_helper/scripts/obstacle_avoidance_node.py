#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        rospy.loginfo("Initializing obstacle avoidance node")

        # Initialize parameters with default values from the parameter server, or use user-specified values if available
        self.min_distance = rospy.get_param('~min_distance', 0.5) # its a threshold basically

        self.max_distance = rospy.get_param('~max_distance', 1.5)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)
        self.max_velocity = rospy.get_param('~max_velocity', 0.5)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 0.5)
        self.obstacle_detected = False

        self.min_safest_dist = rospy.get_param('~min_safest_dist', 0.01) # we can keep it as 0.0 as due to oblation we already have that buffer space to avoid hitting the obstacle.
        self.window = rospy.get_param('~window', 5)
        # self.twist = None

    def laser_scan_callback(self, data):
        # Find the minimum distance to an obstacle from the laser scan data
        min_range = min(data.ranges)

        # Set the obstacle_detected flag based on whether the minimum distance is less than the minimum safe distance
        if min_range < self.min_distance:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def compute_command(self):
        twist = Twist()
        # Generate a velocity command based on whether an obstacle is detected
        if self.obstacle_detected:
            # Calculate the linear velocity based on the distance to the obstacle
            twist.linear.x = max(0.0, (min_range - self.min_safest_dist) / (self.min_distance - self.min_safest_dist)) * self.max_velocity

            # Calculate the angular velocity based on the direction of the obstacle
            # how will this change us away from the obstacle? 
            
            twist.angular.z = (0.5 - data.ranges[len(data.ranges)//2]) * self.max_angular_velocity

            # Publish the velocity command to the robot
            return twist
        else:
            # If no obstacle is detected, move forward at the maximum linear velocity and do not turn
            twist.linear.x = self.max_velocity
            twist.angular.z = 0.0
            return twist

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance', anonymous=True)
    oa = ObstacleAvoidance()
    # action = Twist()
    # self.cmd_vel_pub = rospy.Publisher(rospy.get_param('~cmd_vel_topic', '/cmd_vel'), Twist, queue_size=1)
    action_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) # queue_size = 50
    rospy.Subscriber(rospy.get_param('~scan_topic', '/scan'), LaserScan, oa.laser_scan_callback)
    # cam_sub = rospy.Subscriber("depth/image_raw/compressed", CompressedImage, oa.camCB)
    while not rospy.is_shutdown():
        vw = oa.compute_command()
        action_pub.publish(vw)
