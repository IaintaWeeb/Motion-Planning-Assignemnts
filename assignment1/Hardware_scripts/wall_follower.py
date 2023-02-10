#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import *

import math
import numpy as np 
#Defining Global Variables
active_ = False
pub_ = None
# LaserScan Data
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
#States
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
}
min_angle=0
def wall_follower_switch(req):
    global active_
    active_ = req.data
    # A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
    # when you return it this goes to the client stating that the service is recieved successfuly 
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# Callback for Laserscan
def clbk_laser(msg):
    global regions_,min_angle
    laserdata = np.array(msg.ranges)
    laserdata[(laserdata==0)]=10
    regions_ = {
        'right':  min(min(laserdata[46:86]), 10),# right is at 270 deg
        'fright': min(min(laserdata[73:119]), 10),
        'fleft':  min(min(laserdata[129:170]), 10),
        'front':  min(min(laserdata[99:139]), 10), # front is at 0 deg
        'left':   min(min(laserdata[169:198]), 10), # left is at 90 deg
    }
    min_angle=np.argmin(laserdata)
    take_action()
# State Changer
def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state

# State Controller : Determines the states based on Laserscan Data
def take_action():
    global regions_
    regions = regions_
    
    d = 0.2 # threshold
    # Cases
    if regions['front'] > d and regions['fleft'] > d and regions['fright'] > d:
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] > d:
        change_state(1)
    elif regions['front'] > d and regions['fleft'] > d and regions['fright'] < d:
        change_state(2)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] > d:
        change_state(0)
    elif regions['front'] < d and regions['fleft'] > d and regions['fright'] < d:
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] > d:
        change_state(1)
    elif regions['front'] < d and regions['fleft'] < d and regions['fright'] < d:
        change_state(1)
    elif regions['front'] > d and regions['fleft'] < d and regions['fright'] < d:
        change_state(0)
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)
# Find Wall : Moves forward and right to search for a wall
def find_wall():
    msg = Twist()
    msg.linear.x = 0.0002
    msg.angular.z = -0.08
    return msg

# Turn left
def turn_left():
    msg = Twist()
    msg.angular.z = 0.1
    return msg

# Follows the wall
def follow_the_wall():
    msg = Twist()
    msg.linear.x = 0.067
    msg.angular.z=0.020
    return msg

def main():
    global pub_, active_
    
    rospy.init_node('reading_laser')
    # This node publishes to the /cmd_vel topic
    pub_ = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=1)
    # This node subscribes to the /scan topic
    rospy.Subscriber('/tb3_2/scan', LaserScan, clbk_laser)
    # Service Declaration
    rospy.Service('wall_follower_switch', SetBool, wall_follower_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue
        
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
            pass
        else:
            rospy.logerr('Unknown state!')
        
        pub_.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    main()