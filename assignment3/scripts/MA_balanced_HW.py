#!/usr/bin/env python3

# import ros stuff
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

# robot state variable:
yaw_=np.array([5.0,6.0,7.0,10.0])
pos_bot=np.array([[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]])
bot1_pos=[]
bot2_pos=[]
bot3_pos=[]
bot4_pos=[]

# yaw_=np.array([5.0,6.0])
# pos_bot=np.array([[0.0,0.0],[0.0,0.0]])
# publishers
pub = [0,0,0,0]
# callbacks
def clbk_bot1(msg):
    global yaw_,pos_bot,bot1_pos
    position_ = msg.pose.pose.position
    pos_bot[0]=[position_.x,position_.y]
    bot1_pos.append((position_.x,position_.y))
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[0] = euler[2]

def clbk_bot2(msg):
    global yaw_,pos_bot,bot2_pos
    position_ = msg.pose.pose.position
    pos_bot[1]=[position_.x,position_.y]
    bot2_pos.append((position_.x,position_.y))
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[1] = euler[2]

def clbk_bot3(msg):
    global yaw_,pos_bot,bot3_pos
    position_ = msg.pose.pose.position
    pos_bot[2]=[position_.x,position_.y]
    bot3_pos.append((position_.x,position_.y))
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[2] = euler[2]

def clbk_bot4(msg):
    global yaw_,pos_bot,bot4_pos
    position_ = msg.pose.pose.position
    pos_bot[3]=[position_.x,position_.y]
    bot4_pos.append((position_.x,position_.y)) 
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_[3] = euler[2]


# Synchronization field
def gradient_descent(i):
    global yaw_
    head_inp=0
    for j in range(4):
        head_inp=head_inp-(np.sin(yaw_[j]-yaw_[i]))
    return 0.3*head_inp
                           

# Controls the heading of the Robot : Gradient Descent
def heading():
    global  pub,yaw_,synced
    for i in range(4):
        th=Twist()
        th.angular.z=gradient_descent(i)
        th.linear.x = 0.1
        pub[i].publish(th)


def main():
    global pub,yaw_,pos_bot  
    rospy.init_node('MA_sync')
    # This node publishes to the /cmd_vel topic
    pub[0]= rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    pub[1]= rospy.Publisher('/tb3_3/cmd_vel', Twist, queue_size=1)
    pub[2]= rospy.Publisher('/tb3_4/cmd_vel', Twist, queue_size=1)
    pub[3]= rospy.Publisher('/tb3_5/cmd_vel', Twist, queue_size=1)

    # This node subscribes to the /odom topic
    rospy.Subscriber('/tb3_0/odom', Odometry, clbk_bot1)
    rospy.Subscriber('/tb3_3/odom', Odometry, clbk_bot2)
    rospy.Subscriber('/tb3_4/odom', Odometry, clbk_bot3)
    rospy.Subscriber('/tb3_5/odom', Odometry, clbk_bot4)
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        np.set_printoptions(precision=2)
        heading()
        print("COM: ",np.mean(pos_bot,axis=0))
        rate.sleep()
    print(bot4_pos)
    print(bot3_pos)
    print(bot2_pos)
    print(bot1_pos)

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException :
        pass