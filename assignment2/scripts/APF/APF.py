#!/usr/bin/env python3

# import ros stuff
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np
import math

# robot state variables
state_=0 
# state = 0 : heading
# state = 1 : move
# state = 2 : done
position_ = Point()
yaw_=0
r_pos=np.array([position_.x,position_.y])
pos_list=[]
# machine state
# goal
w=[1,1,1,1,1]
desired_position_ = Point()
desired_position_.x = -0.25
desired_position_.y = 3
desired_position_.z = 0
g_pos=np.array([desired_position_.x,desired_position_.y])
ob1_pos=np.array([0,2])
ob2_pos=np.array([1.5,0])
ob3_pos=np.array([0,-2])
ob4_pos=np.array([-1.5,0])
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_=0.02
zeta=0.05
eta=0.05
# publisher
pub = None
# callbacks
def clbk_odom(msg):
    global position_,pos_list
    global yaw_,r_pos
    # position
    position_ = msg.pose.pose.position
    pos_list.append((position_.x,position_.y))
    r_pos=np.array([position_.x,position_.y])
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

# Distance
def dist_pos(point1,point2):
    vd=point1-point2
    return np.sqrt((vd[0])**2+(vd[1])**2)

# Attractive field
def F_att():
    dist_goal=dist_pos(r_pos,g_pos)
    if dist_goal < 1:
        return 4*zeta*(g_pos-r_pos)
    else:
        return zeta*(g_pos-r_pos)/dist_goal
# repulsive fields : as a function of q and obstacle
def F_rep(ob,q):
    dist_ob = dist_pos(r_pos,ob)
    if dist_ob < q:
        return eta*((1/q)-(1/dist_ob))*((1/dist_ob)**3)*(ob-r_pos)
    else:
        return 0
      
# Normalizes angle: We want angle to be between (-pi,pi)
def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

# State Controller
def change_state(state):
    global state_
    state_ = state
    print('State changed to [%s]' % state_)

# Controls the heading of the Robot : towards the resultant magnetic field
def heading():
    global yaw_, pub, yaw_precision_,w
    F_total=w[0]*F_att()+w[1]*F_rep(ob1_pos,1)+w[2]*F_rep(ob2_pos,1)+w[3]*F_rep(ob3_pos,1)+w[4]*F_rep(ob4_pos,1) 
    desired_yaw= math.atan2(F_total[1],F_total[0])
    err_yaw = normalize_angle(desired_yaw - yaw_)
    twist_msg = Twist()
    #corrects the yaw
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print('Yaw error: [%s]' % err_yaw)
        change_state(1)

# Velocity control : magnitude of resultant force
def move():
    global yaw_, pub, yaw_precision_, state_, dist_precision_
    F_total=w[0]*F_att()+w[1]*F_rep(ob1_pos,1)+w[2]*F_rep(ob2_pos,1)+w[3]*F_rep(ob3_pos,1)+w[4]*F_rep(ob4_pos,1) 
    desired_yaw = math.atan2(F_total[1],F_total[0])
    velocity=dist_pos(F_total,0)
    print(velocity)
    err_yaw = desired_yaw - yaw_
    err_pos = dist_pos(r_pos,g_pos)
    twist_msg = Twist()
    twist_msg.linear.x = velocity
    twist_msg.angular.z = 0.033 if err_yaw > 0 else -0.033
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

    if math.fabs(err_pos) < dist_precision_:
        change_state(2)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    print('Done')
    pub.publish(twist_msg)

def main():
    global pub,pos_list
    
    rospy.init_node('APF')
    # This node publishes to the /cmd_vel topic
    pub = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=1)
    # This node subscribes to the /odom topic
    rospy.Subscriber('/tb3_2/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        print(F_att(),F_rep(ob1_pos,1),F_rep(ob2_pos,1),F_rep(ob3_pos,1),F_rep(ob2_pos,1),r_pos)
        if state_ == 0:
            heading()
        elif state_ == 1:
            move()
        elif state_== 2:
            done()
            print(pos_list)
            break
        rate.sleep()

if __name__ == '__main__':
    try:
       main()
    except rospy.ROSInterruptException :
        pass