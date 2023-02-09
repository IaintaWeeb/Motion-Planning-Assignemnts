#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *

import math
import numpy as np
curcumnavigated=False
# Initializing some Global variables
# activates the wall_follower, go_to_point program
srv_client_go_to_point_ = None 
srv_client_wall_follower_ = None
yaw_ = 0 # Rotation About z-axis, will by manipulated by using angular.z 
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
# Current, Initial and desired position initialized as point datatype 
position_ = Point()
initial_position_ = Point()
# initial position = (0,0,0)
initial_position_.x = 0
initial_position_.y = 0
initial_position_.z = 0
desired_position_ = Point()
desired_position_.x = 2.5
desired_position_.y = -2.5
desired_position_.z = 0
# datafrom laserscan divided into front,left and right
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
state_desc_ = ['Go to point', 'circumnavigate obstacle', 'go to closest point']
state_ = 0
# initializing p_hit and p_leave 
p_hit = Point()
p_leave = Point()
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - circumnavigate
# 2 - go to closest point

# callbacks
# 1. Callback from Odometry: provides with Yaw and current Position
def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
    pass
# 2. Callback from LaserScan :  laser data divided into front,left and right 
def clbk_laser(msg):
    global regions_
    laser_range = np.array(msg.ranges)
    regions_ = {
        'right':  min(min(msg.ranges[255:285]), 10),
        'fright': min(min(msg.ranges[285:324]), 10),
        'fleft':  min(min(msg.ranges[20:50]), 10),
        'front':  min(min(msg.ranges[0:20]), min(msg.ranges[325:359])),
        'left':   min(min(msg.ranges[50:120]), 10),
    }
    pass

# State Controller
def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0: # Go to point
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1: # Circumnavigate
        resp = srv_client_go_to_point_(False) 
        resp = srv_client_wall_follower_(True)
    if state_ == 2: # Go to nearest point
        resp = srv_client_go_to_point_(False) 
        resp = srv_client_wall_follower_(True)

#  Functions to calculate distance and Normalize angle 
def calc_dist_points(point1, point2):
    dist = math.sqrt((point1.y - point2.y)**2 + (point1.x - point2.x)**2)
    return dist

def normalize_angle(angle): # angle should belong to (-pi,pi)
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

 
def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global p_leave, p_hit
    global count_loop_, count_state_time_
    
    rospy.init_node('bug1')
    
    # This nodes subscribes to '/scan' and '/odom' topics
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    # Rospy Services
    # Waits for the services to become available
    rospy.wait_for_service('/go_to_point_switch')
    rospy.wait_for_service('/wall_follower_switch')
    
    # Calling the service
    srv_client_go_to_point_ = rospy.ServiceProxy('/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    
    # initialize going to the point
    change_state(0)
    
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        if state_ == 0:
            if regions_['front'] > 0.10 and regions_['front'] < 0.3:
                p_leave = position_
                p_hit = position_
                count_state_time_=0
                change_state(1)
        
        elif state_ == 1:
            # if current position is closer to the goal than the previous closest_position, assign current position to closest_point
            if calc_dist_points(position_, desired_position_) < calc_dist_points(p_leave, desired_position_):
                p_leave = position_
                
            # compare only after 5 seconds - need some time to get out of starting_point
            # if robot reaches (is close to) starting point
            if count_state_time_ > 20 and \
               calc_dist_points(position_, p_hit) < 0.2:
                change_state(2)
        
        elif state_ == 2:
            # if robot reaches (is close to) closest point
            if calc_dist_points(position_, p_leave) < 0.2:
                change_state(0)

        # Timer: Rate is 20 so 1 second is elapsed when the interation count reaches to 20.
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        rate.sleep()

if __name__ == "__main__":
    main()