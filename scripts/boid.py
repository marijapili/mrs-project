#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, cos, sin
from std_msgs.msg import Float32MultiArray

import numpy as np

class Boid():

    def odom_callback(self, data):
        self.ground_truth_pose = data

    def obstacles_callback(self, data):
        self.obstacles_data = data



    def __init__(self, x, circle, max_vel, max_acc, num_of_robots):

        rospy.Subscriber('/sphero_{}/odom_est'.format(x), Odometry, self.odom_callback)
        rospy.Subscriber('/obstacles', Float32MultiArray, self.obstacles_callback)
        

        # Global frame
        self.ground_truth_pose = Odometry()
        self.obstacles_data = Float32MultiArray()

        self.obstacles_data.data = [0.0 for i in range(num_of_robots*2)]

        # Parameters
        self.no = x
        self.num_of_robots = num_of_robots
        self.circle = circle
        self.max_vel = max_vel
        self.max_acc = max_acc


    def get_position(self):
        return np.array([self.ground_truth_pose.pose.pose.position.x, self.ground_truth_pose.pose.pose.position.y])
        

    def get_orientation(self):
        return (2 * atan2(self.ground_truth_pose.pose.pose.orientation.z, self.ground_truth_pose.pose.pose.orientation.w))
        

    def get_velocity(self):
        return np.array([self.ground_truth_pose.twist.twist.linear.x, self.ground_truth_pose.twist.twist.linear.y])

    def get_index(self):
        return self.no

    def separation(self, flock):
        j = 0
        repellant_vec = np.array([0.0, 0.0])
        acc = np.array([0.0, 0.0])

        for boid in flock:
            dist = np.linalg.norm(boid.get_position() - self.get_position())
            if(self.get_index() != boid.get_index() and dist < self.circle and dist != 0):
                repellant_vec += (self.get_position() - boid.get_position())/(dist**2)
                j += 1
            
        
        if(j != 0):
            repellant_vec /= j
            if(np.linalg.norm(repellant_vec) > 0.0):
                repellant_vec = (repellant_vec/np.linalg.norm(repellant_vec)) * self.max_vel
            acc = repellant_vec - self.get_velocity()
            if(np.linalg.norm(acc) > self.max_acc):
                acc = (acc/np.linalg.norm(acc)) * self.max_acc

        return acc

    def alignment(self, flock):
        j = 0
        avg_vel = np.array([0.0, 0.0])
        acc = np.array([0.0, 0.0])
       
        for boid in flock:
            if(self.get_index() != boid.get_index() and np.linalg.norm(boid.get_position() - self.get_position()) < self.circle):
                avg_vel += boid.get_velocity()
                j += 1

        if(j > 0):
            avg_vel /= j
            if(np.linalg.norm(avg_vel) != 0):
                avg_vel = (avg_vel / np.linalg.norm(avg_vel)) * self.max_vel
            acc = avg_vel - self.get_velocity()
            if(np.linalg.norm(acc) > self.max_acc):
                acc = (acc/np.linalg.norm(acc)) * self.max_acc
        
        return acc

    def cohesion(self, flock):
        j = 0
        avg_center = np.array([0.0, 0.0])
        acc = np.array([0.0, 0.0])
    
        for boid in flock:
            if(self.get_index() != boid.get_index() and np.linalg.norm(boid.get_position() - self.get_position()) < self.circle):
                avg_center += boid.get_position()
                j += 1
        
        if(j != 0):
            avg_center /= j
            diff_to_center = avg_center - self.get_position()
            if(np.linalg.norm(diff_to_center) > 0):
                diff_to_center = (diff_to_center/np.linalg.norm(diff_to_center)) * self.max_vel
            acc = diff_to_center - self.get_velocity()
            if(np.linalg.norm(acc) > self.max_acc):
                acc = (acc/np.linalg.norm(acc)) * self.max_acc

        return acc

    def obstacle_avoidance(self):
        acc = np.array([0.0, 0.0])
        vec = np.array([self.obstacles_data.data[self.no*2], self.obstacles_data.data[self.no*2 + 1]])
        if(np.linalg.norm(vec) > 0.0):
            vec = (vec/np.linalg.norm(vec)) * self.max_vel
            acc = vec - self.get_velocity()
        if(np.linalg.norm(acc) > self.max_acc):
            acc = (acc/np.linalg.norm(acc)) * self.max_acc

        return acc


    def apply_rules(self, flock, center_to_target):
        
        # Weight parameters
        S = 0.4
        A = 0.5
        C = 0.6
        T = 0.7
        O = 2
        if(self.no == 0):
            acc_target = center_to_target[0] - self.get_velocity()
            if(np.linalg.norm(acc_target) > self.max_acc):
                acc_target = (acc_target/np.linalg.norm(acc_target)) * self.max_acc
            A = 0
            C = 0
            S = 0

        else:
            acc_target = 0


        separation = S*self.separation(flock)
        alignment = A*self.alignment(flock)
        cohesion = C*self.cohesion(flock)
        acc_obst = O*self.obstacle_avoidance()

        correction = self.get_velocity() + separation + alignment + cohesion + T*acc_target + acc_obst
        pub_data = self.calc_publish_data(correction)

        if self.no > 0 and self.calculate_distance(center_to_target[self.no]) < 0.2:
            pub_data = Twist()
        
        return pub_data


    def calc_publish_data(self, vel):
        phi = self.get_orientation()
        cmd_vel = Twist()
        R_i = np.linalg.inv(np.array([[cos(phi), -sin(phi)], [sin(phi), cos(phi)]]))
        pub_vel = R_i.dot(vel)
        cmd_vel.linear.x = 70*pub_vel[0]
        cmd_vel.linear.y = 70*pub_vel[1]
        return cmd_vel

    def calculate_distance(self,position):
        return np.sqrt(position[0]**2+position[1]**2)

    
    

