#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, cos, sin
from std_msgs.msg import Float32MultiArray
import numpy as np

class Formation():
    def odom_callback(self, data):
        self.odom = data
    
    def __init__(self, formation, num_of_robots):
        self.fomation = formation
        self.num_of_robots = num_of_robots

        rospy.Subscriber('/sphero_{}/odom_est'.format(x), Odometry, self.odom_callback)
        self.odom = Odometry()


if __name__ == '__main__':

    rospy.init_node('formation_control')
    num_of_robots = rospy.get_param('/num_of_robots')
    formation = np.zeros((num_of_robots, num_of_robots))
