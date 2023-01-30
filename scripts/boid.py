#!/usr/bin/env python3
import math

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

    def __init__(self, x, circle, max_vel, max_acc, num_of_robots, formation):

        rospy.Subscriber('/robot_{}/base_pose_ground_truth'.format(x), Odometry, self.odom_callback)
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
        self.formation = formation

        # Consensus
        self.A = []  # neighbors
        self.E = []  # distances from leader in XY coordinates
        distance_mod = 0.6 # how far apart should the robots be?

        # Make all boids neighbors except for itself
        for i in range(0, num_of_robots):
            if i != self.no:
                self.A.append(1)
            else:
                self.A.append(0)

        if self.no != 0: # don't apply consesus to leader
            if self.no % 2 == 0: # Even numbered go right, odd numbered go left
                direction_mod = -1
            else:
                direction_mod = 1

            if self.formation == 0:  # line
                self.E = [direction_mod * math.ceil(self.no / 2) * distance_mod, 0]
            elif self.formation == 1:
                self.E = [0, - direction_mod * math.ceil(self.no / 2) * distance_mod]
            elif self.formation == 2:
                self.E = [direction_mod * math.ceil(self.no / 2) * distance_mod * math.cos(1/4 * math.pi),  math.ceil(self.no / 2) * distance_mod * math.sin(1/4 * math.pi)]
        else:
            self.E = [0,0]

        #print(self.A)
        print(self.E)

    def get_position(self):
        return np.array([self.ground_truth_pose.pose.pose.position.x, self.ground_truth_pose.pose.pose.position.y])

    def get_orientation(self):
        return (2 * atan2(self.ground_truth_pose.pose.pose.orientation.z, self.ground_truth_pose.pose.pose.orientation.w))

    def get_velocity(self):
        return np.array([self.ground_truth_pose.twist.twist.linear.x, self.ground_truth_pose.twist.twist.linear.y])

    def get_index(self):
        return self.no

    def get_E(self):
        return self.E

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

    def consensus(self, flock):
        vx = 0
        vy = 0

        if self.formation != -1:
            x_curr = self.get_position()[0]
            y_curr = self.get_position()[1]

            for i in range(0, len(self.A)):
                vx += self.A[i] * ( (flock[i].get_position()[0] - x_curr) - (self.E[0] - flock[i].get_E()[0]))

            for i in range(0, len(self.A)):
                vy += self.A[i] * ( (flock[i].get_position()[1] - y_curr) - (self.E[1] - flock[i].get_E()[1]))

            con_vec = [vx, vy]
            if(np.linalg.norm(con_vec) > 0.0):
                con_vec = (con_vec/np.linalg.norm(con_vec)) * self.max_vel
            acc = con_vec - self.get_velocity()
            if(np.linalg.norm(acc) > self.max_acc):
                acc = (acc/np.linalg.norm(acc)) * self.max_acc

        return acc

    def apply_rules(self, flock, center_to_target):
        
        # Weight parameters
        S = 0.2
        A = 0.5
        C = 0.2
        T = 0.7
        O = 2
        CON = 0.5

        if(self.no == 0):
            acc_target = center_to_target - self.get_velocity()
            if(np.linalg.norm(acc_target) > self.max_acc):
                acc_target = (acc_target/np.linalg.norm(acc_target)) * self.max_acc
            A = 0
            C = 0
            S = 0

        else:
            acc_target = 0


        consensus = CON * self.consensus(flock)
        separation = S*self.separation(flock)
        alignment = A*self.alignment(flock)
        cohesion = C*self.cohesion(flock)
        acc_obst = O*self.obstacle_avoidance()

        correction = self.get_velocity() + separation + alignment + cohesion + T*acc_target + acc_obst + consensus
        pub_data = self.calc_publish_data(correction)
        
        return pub_data

    def calc_publish_data(self, vel):
        phi = self.get_orientation()
        cmd_vel = Twist()
        R_i = np.linalg.inv(np.array([[cos(phi), -sin(phi)], [sin(phi), cos(phi)]]))
        pub_vel = R_i.dot(vel)
        cmd_vel.linear.x = pub_vel[0]
        cmd_vel.linear.y = pub_vel[1]
        return cmd_vel

    
    

