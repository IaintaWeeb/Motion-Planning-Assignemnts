#!/usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math
active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 2.5
desired_position_.y = -2.5
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.2

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):
    global active_
    active_ = req.data
    # A service sends a message and also expects a response. SetBool has message called data which is of bool type and the response has 2 parts 1) success variable of bool type and message variable which is a string
    # when you return it this goes to the client stating that the service is recieved successfuly
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_
    
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

# State Changer
def change_state(state):
    global state_
    state_ = state
    print('State changed to [%s]' % state_)

# Normalizes angle: We want angle to be between (-pi,pi)
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# Yaw Controller
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    rospy.loginfo(err_yaw)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.233 if err_yaw > 0 else -0.233
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        change_state(1)

# Goes in the direction og the goal;simultenously correctes the yaw
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.033 if err_yaw > 0 else -0.033
        pub.publish(twist_msg)
    else:
        print( 'Position error: [%s]' % err_pos)
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    print('Done')
    pub.publish(twist_msg)

def main():
    global pub, active_
    
    rospy.init_node('go_to_point')
    # This node publishes to the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # This node subscribes to the /odom topic
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    #Service Declaration 
    rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
                break
            else:
                rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
    main()