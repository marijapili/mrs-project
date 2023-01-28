#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from math import atan2

import numpy as np
from boid import Boid

class Node():

    def __init__(self):
        self.num_of_robots = rospy.get_param('/num_of_robots')
        self.circle = 3
        self.max_vel = 1.5 
        self.max_acc = 1.1

        self.flock = [Boid(x, self.circle, self.max_vel, self.max_acc, self.num_of_robots) for x in range(self.num_of_robots)]
        self.pubs = [rospy.Publisher('/sphero_{}/cmd_vel'.format(x), Twist, queue_size=1) for x in range(self.num_of_robots)]
        self.pub_data = []

        self.target = np.array([0.5, 0.5])



    def goto(self):
        #avg_center = np.array([0.0, 0.0])
        #center_to_target = np.array([0.0, 0.0])
        #j = 0
        #for boid in self.flock:
        #    avg_center += boid.get_position()
        #    j += 1

        #avg_center /= j
        #center_to_target  = self.target - avg_center
        center_to_target  = np.array([self.target-boid.get_position() for boid in self.flock])
        dist = np.linalg.norm(center_to_target)
        if(dist > 1.5):
            center_to_target = (center_to_target/dist) * self.max_vel
        else:
            center_to_target = (center_to_target/dist) * self.max_vel* dist/1.5
        
        return center_to_target


    def run(self):
        while not rospy.is_shutdown():
            center_to_target = self.goto()
            self.pub_data.clear()
            for boid in self.flock:
               self.pub_data.append(boid.apply_rules(self.flock, center_to_target))
            
            for x in range(self.num_of_robots):
                self.pubs[x].publish(self.pub_data[x])

        pass




if __name__ == '__main__':
    rospy.init_node('node')
    try:
        node = Node()
        node.run()
    except rospy.ROSInterruptException:
        pass