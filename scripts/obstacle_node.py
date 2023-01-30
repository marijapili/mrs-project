#!/usr/bin/env python3

import rospy

import numpy as np
import message_filters
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry


class Obstacles():

    def map_callback(self, data):
        self.width = data.info.width
        self.height = data.info.height
        self.resolution = data.info.resolution
        self.origin = data.info.origin.position
        self.circle = int(self.circle/self.resolution)

        for i in range(self.height - 1, -1, -1):
            self.map.append(data.data[i * self.width:(i + 1) * self.width])

    def odom_callback(self, *data):
        for boid in data:
            no = boid.header.frame_id.split('/')[1]
            no = int(no.split('_')[1])
            x = boid.pose.pose.position.x
            y = boid.pose.pose.position.y
            row, column = self.position_to_grid(x, y)

            visible_rows = range(max(0, int(row - self.circle)), min(self.height, int(row + self.circle + 1)))
            visible_columns = range(max(0, int(column - self.circle)), min(self.width, int(column + self.circle + 1)))
            vec = np.array([0.0, 0.0])
            j = 0
            for r in visible_rows:
                for c in visible_columns:
                    if(((r-row)**2 + (c-column)**2) <= (self.circle**2)):
                        if(self.map[r][c] == 100):
                            x_ob, y_ob = self.grid_to_position(r, c)
                            d = np.linalg.norm(np.array([(x-x_ob), (y-y_ob)]))
                            vec += np.array([(x-x_ob), (y-y_ob)])/(d**2)
            if(j != 0):
                vec /= j
            
            self.forces[no] = vec
        
        self.pub_data()



    def __init__(self):

        self.map = []
        self.width = 0
        self.height = 0
        self.resolution = 0.1
        self.origin = Point()

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.num_of_robots = rospy.get_param('/num_of_robots')

        self.circle = 0.6
        self.forces = []
        for i in range(self.num_of_robots):
            self.forces.append(np.array([0.0, 0.0]))

        self.pub = rospy.Publisher('/obstacles', Float32MultiArray, queue_size=1)

        subs = [message_filters.Subscriber('/robot_{}/base_pose_ground_truth'.format(x), Odometry) for x in range(self.num_of_robots)]
        sync = message_filters.ApproximateTimeSynchronizer(subs, 10, 0.05)
        sync.registerCallback(self.odom_callback)

        rospy.spin()


    def position_to_grid(self, x, y):
        # Gobal position inside map grid
        row = (self.origin.y + self.resolution*self.height - y)/(self.resolution)
        row = int(row)
        column = (x - self.origin.x)/self.resolution
        column = int(column)
        return row, column

    def grid_to_position(self, row, column):
        # Map grid to global positions
        x = self.origin.x + self.resolution*column
        y = self.origin.y + self.resolution*(self.height - row)
        return x, y

    def pub_data(self):
        #print(self.forces[0][1])
        arr = Float32MultiArray()
        for i in range(self.num_of_robots):
            arr.data.append(self.forces[i][0])
            arr.data.append(self.forces[i][1])
        self.pub.publish(arr)



if __name__ == '__main__':
    rospy.init_node('obstacles')
    try:
        ob = Obstacles()
    except rospy.ROSInterruptException:
        pass


        
