#!/usr/bin/env python

# Importing all dependencies
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import random
import math

# Initializing global values
ran_x = np.array([])
ran_y = np.array([])
pt_x = []
pt_y = []
    

# Main Ransac function
def ransac(x, y):

    # Initializing variables, thresholds & iteration
    iteration = 500
    distance_threshold = 0.05
    points_threshold = 5

    global ran_x
    global ran_y
    x = ran_x
    y = ran_y

    # callback for LaserScan which converts the laser data into r*sin & r*cos values for each 'r' (laser reading) at a particular degree.
    def cb(range):
        global ran_x
        global ran_y
        rang = range.ranges
        rang = np.array(rang)

        for i,val in enumerate(rang):
            if val == 3.0:
                rang[i] = 0

        #rang[rang==3.0] = 0
        y = rang * sin
        x = rang * cos
        ran_x = x.reshape((x.shape[0], 1))
        ran_y = y.reshape((y.shape[0], 1))

    rospy.Subscriber("/robot_0/base_scan", LaserScan, cb)


    
    # if x is empty then return
    if len(x) == 0:
        return

    # Initializing holder variable for in-situ ransac calculations
    global pt_x, pt_y
    pt_x = [-1]
    pt_y = [-1]
    maximum_x = -1
    maximum_y = -1
    minimum_x = -1
    minimum_y = -1
    inliers = []
    outliers = []
       
    # loop through all 'x'
    ind = range(len(x))
    for i in range(iteration):

        # Holder variable for current iteration outliers & inliers
        sample_in = []
        sample_out = []
        
        # Choose 2 random indices
        p1_i = random.randint(0,len(ind)-1)
        p2_i = random.randint(0,len(ind)-1)

        # if points are same or non-significant then skip
        if p1_i == p2_i or (x[ind[p1_i]]==0) or (x[ind[p2_i]]==0):
            continue

        # Get x & y of those 2 points
        p1x = x[ind[p1_i]]
        p1y = y[ind[p1_i]]   
        p2x = x[ind[p2_i]]
        p2y = y[ind[p2_i]]


        # loop through all x to calculate inliers & outliers
        for i in (ind):
            x_i = x[i]
            y_i = y[i]
            if x_i == 0 and y_i == 0:
                continue

            # calculate distance of points x_i & y_i from the line joining points p1 & p2
            dist = np.abs(((p2y - p1y) * x_i) - ((p2x - p1x)*y_i) + (p2x * p1y) - (p2y * p1x)) / math.sqrt(np.power((p2y - p1y),2) + np.power((p2x - p1x),2))
            
            # split inliers & outliers depending upon threshold
            if dist < distance_threshold:
                sample_in.append(i)
            else:
                sample_out.append(i)

        # if the current set of inliers is best then replace the previous best to current one
        if len(inliers) < len(sample_in):
            inliers = sample_in
            outliers = sample_out
            maximum_x, maximum_y, minimum_x, minimum_y = np.max(ran_x[inliers]), np.max(ran_y[inliers]), np.min(ran_x[inliers]), np.min(ran_y[inliers])
            
    # appending the maximum & minimum of best set of inliers
    if len(inliers) > points_threshold:
        pt_x.append(maximum_x)
        pt_y.append(maximum_y)
        pt_x.append(minimum_x)
        pt_y.append(minimum_y)
    if len(ind) < len(x) * 0.01:
        pass



# Main function for evader
def evader_func():

    # Initiating evader node
    rospy.init_node('perception')

    # Setting the rate of cycles
    rate = rospy.Rate(100) # 5Hz

    # Initializing Marker publisher for rviz (line: Ransac line || wall:actual laserscan data)
    line = rospy.Publisher("line", Marker, queue_size=10)
    wall = rospy.Publisher('wall', Marker, queue_size=10)

    # Initializing variables
    global sin
    global cos
    global ran
    x = 0
    y = 0


    # sin & cos values for field of view angles 
    laser_resolution = np.linspace(np.pi/2,-1 * np.pi / 2 , 361)
    sin = np.sin(laser_resolution)
    cos = np.cos(laser_resolution)
    

    
    while not rospy.is_shutdown():
        try:
            #Main Ransac call
            ransac(x, y)
            
            # Initializing marker for Ransac line data
            marker_line = Marker()
            marker_line.header.frame_id = "/odom"
            marker_line.header.stamp = rospy.Time.now()
            marker_line.type = Marker.LINE_STRIP
            marker_line.action = Marker.ADD
            marker_line.scale.x = 0.2   
            marker_line.color.r = 1.0
            marker_line.color.a = 1.0


            # Initializing marker for wall data
            marker_wall = Marker()
            marker_wall.header.frame_id = "/odom"
            marker_wall.header.stamp = rospy.Time.now()
            marker_wall.type = Marker.LINE_STRIP
            marker_wall.action = Marker.ADD
            marker_wall.scale.x = 0.2
            marker_wall.color.b = 1.0
            marker_wall.color.a = 1.0
            
            # appending line points data
            ln = len(pt_x)
            for i in range(1,ln):
                point = Point()
                point.x = pt_x[i]
                point.y = pt_y[i]
                marker_line.points.append(point)


            # appedning wall points data
            for i in range(361):
                if len(ran_x) == 0:
                    continue

                if ran_x[i] == 0 and ran_y[i] == 0:
                    continue
                point = Point()
                point.x = ran_x[i]
                point.y = ran_y[i]
                marker_wall.points.append(point)
               
            # Publish markers
            line.publish(marker_line)
            wall.publish(marker_wall)


            rate.sleep()
        except:
            continue		

    

if __name__ == '__main__':
    try:
        evader_func()
    except rospy.ROSInterruptException:
        pass