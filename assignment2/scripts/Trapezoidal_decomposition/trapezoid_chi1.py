#!/usr/bin/env python3
#! /usr/bin/env python

# import ros stuff
import rospy
# import ros message
import time
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
# import ros service
from std_srvs.srv import *

import math
 
wait_ = False
yaw_ = 0
yaw_error_allowed_ = 1# 4 degrees
position_ = Point()
graph_len_ = 6
graph_count_ = 1
graph_points_ = [Point()]
#print(len(graph_points_))
p1 = Point(-0.78,-0.04,0)
p2 = Point(-1.53,-1.32,0)
p3 = Point(-1.98,-1.76,0)
p4 = Point(-2.46,-1.65,0)
p5 = Point(-3.11,-2.32,0)
graph_points_.append(p1)
graph_points_.append(p2)
graph_points_.append(p3)
graph_points_.append(p4)
graph_points_.append(p5)
#print(graph_points_[1])
desired_pos_ = graph_points_[1]
#print(desired_pos_)
state_desc_ = ['Go to point', 'circumnavigate obstacle', 'go to closest point']
state_ = 0
circumnavigate_starting_point_ = Point()
circumnavigate_closest_point_ = Point()
closest_obstacle_distance_ = 0
closest_obstacle_angle_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
sampler_ = 0 
position_list_ = []

def ret_dist(P1, P2):
    dist = math.sqrt((P1.x-P2.x)**2 + (P1.y-P2.y)**2)
    return dist

def ret_angle_to_goal():
    global position_
    global desired_pos_

    [err_x, err_y] = [desired_pos_.x-position_.x, desired_pos_.y-position_.y]
    atan_angle = abs(math.atan(err_y/err_x)*180//math.pi)
    angle_to_goal = atan_angle
    if err_x < 0 and err_y > 0:
        angle_to_goal = 180 - atan_angle

    elif err_x < 0 and err_y < 0:
        angle_to_goal = 180 + atan_angle
    
    elif err_x > 0 and err_y < 0:
        angle_to_goal = 360 - atan_angle

    return angle_to_goal

def calculate_goal_angle_in_global_frame():
    global desired_pos_, position_
    delta_x = desired_pos_.x - position_.x
    delta_y = desired_pos_.y - position_.y 
    beta = math.atan2(delta_y, delta_x)*180//math.pi
    beta = (beta + 360) % 360 
    return beta 

def is_collision():
    global closest_obstacle_distance_
    global closest_obstacle_angle_

    if closest_obstacle_distance_ < 0.3:
        return True
    return False

def is_reached_goal():
    global position_
    global desired_pos_

    if ret_dist(position_, desired_pos_) < 0.05:
        return True
    return False

def state0_action():                    #Move towards goal
    global position_
    global yaw_
    global desired_pos_
    global yaw_error_allowed_
    global state_
    global closest_obstacle_distance_
    global closest_obstacle_angle_
    global graph_len_, graph_count_
    global sampler_


    ''' print('Entered action 1')
    beta = calculate_goal_angle_in_global_frame()
    while (abs(beta - yaw_)) >= yaw_error_allowed_:
        move_the_bot.linear.x = 0.0
        if (abs(beta - yaw_) > 30): 
            move_the_bot.angular.z = -0.5
            publish_to_cmd_vel.publish(move_the_bot)
        elif (abs(beta - yaw_) > 10): 
            move_the_bot.angular.z = -0.1
            publish_to_cmd_vel.publish(move_the_bot)
        else:
            move_the_bot.angular.z = -0.01*abs(beta - yaw_) 
            publish_to_cmd_vel.publish(move_the_bot) '''



    ''' while not is_reached_goal():
        print("Current goal is: ", desired_pos_)
        move_the_bot.linear.x = 0.01
        move_the_bot.angular.z = 0.0
        publish_to_cmd_vel.publish(move_the_bot) '''    
    
    
    angle_to_goal = ret_angle_to_goal()
    yaw_error = angle_to_goal - yaw_ if abs(angle_to_goal - yaw_)<=180 else (360 - angle_to_goal + yaw_)
    if yaw_ >= (angle_to_goal + yaw_error_allowed_/2)%360 or yaw_ <= (abs(angle_to_goal - yaw_error_allowed_/2))%360:
        move_the_bot.angular.z = 0.01*yaw_error
        move_the_bot.linear.x = 0.1/(abs(yaw_error)+1e-5) if 0.1/(abs(yaw_error)+1e-5) < 1 else 1
        publish_to_cmd_vel.publish(move_the_bot)
        sampler_ = sampler_ + 1
        if(sampler_ % 10000 == 0):
            print(position_.x, position_.y, position_.z)

    
    #print(graph_count_, "Current position is: ", position_)
    
    
    

    # print("turning: ", angle_to_goal, " yaw: ", yaw_)
    # else:
    #     move_the_bot.linear.x = 0.1
    #     move_the_bot.angular.z = 0
    #     publish_to_cmd_vel.publish(move_the_bot)
    #     print("moving: ", angle_to_goal, " yaw: ", yaw_)
    # print(is_collision(), ", ", closest_obstacle_distance_)
    
    if is_reached_goal():
        move_the_bot.linear.x = 0
        move_the_bot.angular.z = 0
        publish_to_cmd_vel.publish(move_the_bot)
        graph_count_ = graph_count_ + 1
        if (graph_count_ < graph_len_): 
            desired_pos_ = graph_points_[graph_count_]

            state_ = 1

    state_ = 0
    return

def odom_callback(msg):
    global position_
    global yaw_

    position_ = msg.pose.pose.position              #Keep a track of position
    # print(type(position_))
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)  #Convert quaternion to euler angle
    yaw_ = euler[2]*180//math.pi                             #[Roll, Pitch, Yaw]
    yaw_ = yaw_ if yaw_ >= 0 else (yaw_ + 360)
    # print("yaw: ", yaw_)
    position_list_.append(position_)
    
def laserdata_callback(msg):
    global closest_obstacle_distance_
    global closest_obstacle_angle_
    global wait_
    #Prints the length of ranges array, in our case there are total 360 readings, one reading for each 1 degree

    #To make use of laser data, we will need directional readings. For example, front, back, left and right of the robot. In our case, 0: Front, 180: Back, 90: Right, 270: Left,  are directions of laserbeam for robot

    #print(msg.ranges[0], msg.ranges[90], msg.ranges[180], msg.ranges[270])
    
    # sample head collision avoidance algorithm(You will have to write your algorithm here
    closest_obstacle_distance_ = min(msg.ranges)
    closest_obstacle_angle_ = msg.ranges.index(closest_obstacle_distance_)
    wait_ = True
    # move_the_bot.linear.x = 0.0
    # move_the_bot.angular.z = 0.1
    # print("closest obstacle: ", closest_obstacle_distance_, " angle: ", closest_obstacle_angle_, " is_collision: ", is_collision())
    # publish_to_cmd_vel.publish(move_the_bot)


if __name__ == "__main__":

    rospy.init_node('turtlebot_controller_node')
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = laserdata_callback)
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    subscribe_to_odom = rospy.Subscriber('/odom', Odometry, callback = odom_callback)
    #create an object of Twist data

    move_the_bot = Twist()
    rate_hz = 20
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        while (not wait_):
            continue
        while (state_ == 0 and graph_count_ < graph_len_ ):
            state0_action()
        if graph_count_ == graph_len_: 
            #print("Reached the final goal")
            move_the_bot.linear.x = 0
            move_the_bot.angular.z = 0
            publish_to_cmd_vel.publish(move_the_bot)
            with open("example.txt", "w") as f:
                f.write(str(position_list_))
            rate.sleep()
        rate.sleep()
