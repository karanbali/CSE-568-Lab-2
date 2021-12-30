#!/usr/bin/env python

# Importing all dependencies
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import geometry_msgs
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import tf
import random
import math
from tf.transformations import euler_from_quaternion


# Initialzing state ('GOALSEEK' == 1 || 'WALLFOLLOW' == 0)
state  = 1


# Main bug2 function
def bug2(msg):
    global sate
    global rang1

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = 0
    
    
    # Calulating distance of bug from start-goal line
    lndst = ln_dst(x,y)

    # Calulating distance of bug from goal
    goaldst = goal_dst(x,y)
    
    # Calculating bug's position angle after converting it from 'quaternion' to 'euler'
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
    
    # Bug's position angle
    robang = yaw

    
    # Bug's angle from goal
    goalang = (math.atan2((9.0-y),(4.5-x)) - robang)

    #Initializing Twist
    cmd_vel = Twist()
    
    # Function to calculate forward velocity
    def cmpt(sonar):
        minsonar = min(sonar[90:270])
        if minsonar < 0.6:
            return 0
        else:
            return 1

    # Function to calculate rotation when in "GOALSEEK"
    def goalrot(ang):
        global robang
           
        if abs(ang) > np.pi/20:
             
            return ang
                    
        else:
            return 0



    # Function to calculate rotation when in "WALLFOLLOW"
    def wallrot(sonar):

        # Initializing Left & Right sonar values
        global rang1
        rt = rang1[180:360]
        lf = rang1[0:180]


        # Minimum of left & right sonar values
        minrt = min(rt)
        minlf = min(lf)
        
        
        # If both minrt & minlf too close than take a hard right of 90 degress
        if (minrt < 0.01) and (minlf < 0.01):
            return np.pi/2

        # else turn according to distance b/w wall & bug
        else:
            turn = (np.pi/2 - ((np.pi/6)*(3 - minrt)))
            return turn
        

    # Function to detect obstacle in path
    def obst(ang, sonar):
    
        global state
        global rang1

        
        # If minimum for some fov is greater then 0.7 then return 0 else return 1
        if min(rang1[90:270]) > 0.7:
            return 0
        else:
            return 1   

    # Function to check the whether the goal has been reached
    def atGoal(dist): 
        if dist < 1:
            return 1
        else:
            return 0



    # Setting linear velocity of bug
    cmd_vel.linear.x = cmpt(rang1)

    global state

    obstck = obst(goalang, rang1)

    # If in "GOALSEEK", then...
    if (state == 1):
        cmd_vel.angular.z = goalrot(goalang)
    
    # If in "WALLFOLLOW", then...
    if (state == 0):
        cmd_vel.angular.z = wallrot(rang1)
    
        
    # Changes state depending upon the condition
    if (obstck == 1 and state == 1):
            print("State changed to WALLFOLLOW")
            state = 0
    elif (obstck == 0 and state == 0):
            print("State changed to GOALSEEK")
            state = 1


    # Final check if goal has been reached
    if  atGoal(goaldst) == 1:

        # Stop the bug from moving
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        print("Goal Reached")

    # Publishes the Bug's Twist
    pub = rospy.Publisher('/robot_0/cmd_vel', Twist)
    pub.publish(cmd_vel)
   

# Callback to get LaserScan data & store it in a Global variable 'rang1'
def cb(range):
    global rang1
    rang1 = range.ranges
    


# Function to calculatet the distance of bug from line joining intial position & goal position
def ln_dst(x,y):
    global init_x
    global init_y
    global goal_x
    global goal_y

    dist = np.abs(((goal_y - init_y) * x) - ((goal_x - init_x)*y) + (goal_x * init_y) - (goal_y * init_x)) / math.sqrt(np.power((goal_y - init_y),2) + np.power((goal_x - init_x),2))
            
    return dist

# Function to calculatet the distance of bug from goal position
def goal_dst(x,y):
    global goal_x
    global goal_y

    dist = math.sqrt((y - goal_y)*(y - goal_y) + (x - goal_x)*(x - goal_x))
    return dist



# Main function for evader
def evader_func():


    global init_x
    global init_y
    global goal_x
    global goal_y

    # Getting initial position of bug
    init_x = rospy.get_param("/bug2/init_x")
    init_y = rospy.get_param("/bug2/init_y")

    # Getting final goal position
    goal_x = rospy.get_param("/bug2/goal_x")
    goal_y = rospy.get_param("/bug2/goal_y")


    # Initiating evader node
    rospy.init_node('bug2')

    # Setting the rate of cycles
    rate = rospy.Rate(100) # 5Hz

    # Subsciber to get LaserScan data & store it in a Global variable 'rang1'
    rospy.Subscriber("/robot_0/base_scan", LaserScan, cb)

    
    while not rospy.is_shutdown():
        try:
            
            # Subscribing to '/base_pose_ground_truth' & implement 'bug2' algo as a callback
            #pos = rospy.Subscriber('/base_pose_ground_truth', Odometry, bug2, (ran_x,ran_y, pointX, pointY), queue_size=20)
            pos = rospy.Subscriber('/base_pose_ground_truth', Odometry, bug2, queue_size=20)

            rate.sleep()
        except:
            continue		

    

if __name__ == '__main__':
    try:
        evader_func()
    except rospy.ROSInterruptException:
        pass