#!/usr/bin/env python3

# import ros stuff
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

# robot state variables
# state = 0 : sync
# state = 1 : done
state_=0 
yaw_=np.array([1.0,2.0,3.0,4.0,5.0,6.0,7.0,10.0])
synced=False
# parameters : 
std_threshhold = 0.005
error=100
# publishers
pub = [0,0,0,0,0,0,0,0]
# callbacks
def clbk_bot1(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[0] = euler[2]

def clbk_bot2(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[1] = euler[2]

def clbk_bot3(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[2] = euler[2]

def clbk_bot4(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[3] = euler[2]

def clbk_bot5(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[4] = euler[2]

def clbk_bot6(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[5] = euler[2]

def clbk_bot7(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[6] = euler[2]


def clbk_bot8(msg):
    global yaw_
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[7] = euler[2]

# Synchronization field
def gradient_descent(i):
    global yaw_
    head_inp=0
    for j in range(8):
        head_inp=head_inp+(np.sin(yaw_[j])-yaw_[i])
    return 0.1*head_inp
                           

# This function returns true if synchronization is achieved
def is_synced():
    global synced,yaw_,error,std_threshhold
    std_=np.fabs(np.std(yaw_))
    print('prevverror: ' ,error)
    print("curr error", std_)
    if std_ <= error:
        error = np.fabs(np.std(yaw_))
        return False
    else:
        if std_ < std_threshhold:
            return True
        else:
            return False
    

# Controls the heading of the Robot : Gradient Descent
def heading():
    global  pub,yaw_,synced
    for i in range(8):
        th=Twist()
        th.angular.z=gradient_descent(i)
        th.linear.x = 0.05
        pub[i].publish(th)
    synced=is_synced()

# stops the function after synchronization is achieved
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    print('Done')
    for i in range(8):
        pub[i].publish(twist_msg)

def main():
    global pub,yaw_,iters,synced
    
    rospy.init_node('MA_sync')
    # This node publishes to the /cmd_vel topic
    pub[0]= rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=1)
    pub[1]= rospy.Publisher('/bot_2/cmd_vel', Twist, queue_size=1)
    pub[2]= rospy.Publisher('/bot_3/cmd_vel', Twist, queue_size=1)
    pub[3]= rospy.Publisher('/bot_4/cmd_vel', Twist, queue_size=1)
    pub[4]= rospy.Publisher('/bot_5/cmd_vel', Twist, queue_size=1)
    pub[5]= rospy.Publisher('/bot_6/cmd_vel', Twist, queue_size=1)
    pub[6]= rospy.Publisher('/bot_7/cmd_vel', Twist, queue_size=1)
    pub[7]= rospy.Publisher('/bot_8/cmd_vel', Twist, queue_size=1)
    # This node subscribes to the /odom topic
    rospy.Subscriber('/bot_1/odom', Odometry, clbk_bot1)
    rospy.Subscriber('/bot_2/odom', Odometry, clbk_bot2)
    rospy.Subscriber('/bot_3/odom', Odometry, clbk_bot3)
    rospy.Subscriber('/bot_4/odom', Odometry, clbk_bot4)
    rospy.Subscriber('/bot_5/odom', Odometry, clbk_bot5)
    rospy.Subscriber('/bot_6/odom', Odometry, clbk_bot6)
    rospy.Subscriber('/bot_7/odom', Odometry, clbk_bot7)
    rospy.Subscriber('/bot_8/odom', Odometry, clbk_bot8)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        np.set_printoptions(precision=2)
        if  synced == False:
            heading()
            print("after heading: ",yaw_)
        else :
            done()
            print(yaw_)
            break
        rate.sleep()

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException :
        pass