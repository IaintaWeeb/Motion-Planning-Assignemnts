#!/usr/bin/env python3

# import ros stuff
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np

import math
goal=[0,3.5]
path=['0.025990201108921245,3.249569428646434', '0.05000000000000306,3.2501350849829205', '0.15000000000000366,3.245829919870603', '0.15121182358372456,3.245697028027798', '0.250000000000004,3.2381587472119913', '0.2770458265648633,3.2343005949078116', '0.3500000000000035,3.226311853609555', '0.40430635391723946,3.2167881416993933', '0.4500000000000045,3.210269978516042', '0.5338321783763624,3.1928463737293864', '0.5500000000000032,3.190004704481407', '0.6500000000000049,3.1659952588533704', '0.6660373155093727,3.161113049147584', '0.7500000000000044,3.138259839345638', '0.8016025218758658,3.120947010120318', '0.850000000000003,3.1062263316966323', '0.9421420955975376,3.0724711679282777', '0.9500000000000042,3.06983013422316', '1.0500000000000047,3.0301275857061025', '1.087387183580749,3.0130060508123773', '1.150000000000003,2.986228606900573', '1.2397765347295828,2.9424835756441325', '1.2500000000000044,2.9377974471905826', '1.350000000000004,2.8861386497773136', '1.3992011586097877,2.8578575247824625', '1.450000000000005,2.830118705565555', '1.5500000000000052,2.7698325942680206', '1.5684622204946885,2.7576536208381883', '1.6500000000000048,2.7061883324735803', '1.7487931379862856,2.6383593269289056', '1.750000000000003,2.6375628935044633', '1.850000000000008,2.5661719382715535', '1.9416592092746963,2.4957869928527208', '1.9500000000000055,2.489599560982067', '2.050000000000005,2.41015544825008', '2.1500000000000052,2.325450020010336', '2.1504846782431724,2.325013970777306', '2.177839114195796,2.3011324932841637', '1.9776309784036026,2.1279519280249537', '1.9402035991271889,2.0970911026966927', '1.7416544365420272,1.9275181689068903', '1.7330540203627856,1.9199206362001644', '1.5759392462963124,1.7888581341249035', '1.5364900778361859,1.7543875679124095', '1.4233438957641067,1.661947544960699', '1.3716266527753664,1.6174097935931813', '1.2910633953812218,1.5531787294933728', '1.2282650243715563,1.5000803037028234', '1.1732251685623882,1.4574045245720935', '1.0996248988899542,1.3965322104845397', '1.0656938282847161,1.371023402966616', '0.9809008094629285,1.3026666841170018', '0.9653985468569097,1.2913928853117647', '0.8698581887990513,1.2165445683780887', '0.8684868483993584,1.2153853752522974', '0.7764944168645621,1.1455059185866285', '0.7597831532772354,1.1317765750250295', '0.6843319502656542,1.076272039504854', '0.6519211029464466,1.0504539426274084', '0.591687538779103,1.0075468667352183', '0.5424053596465499,0.9695323623618224', '0.4968413138435992,0.938067705904003', '0.4286222609077702,0.8871191620117335', '0.3979115295313497,0.8665143543999853', '0.3075904812181771,0.8011379325054183', '0.29270516980385225,0.7914040742064968', '0.17849231623960232,0.7111192031965633', '0.1757419994348137,0.7090549511285084', '0.051770245309927194,0.6241842860481236', '0.028805095585920367,0.6074827023076181', '-0.09204101595179517,0.5266378020774', '-0.10590849210825737,0.5168381307551008', '-0.10574143956162763,0.5089441037212509', '-0.10446725282263436,0.48157778536340706', '-0.10313097685562056,0.35667922006384034', '-0.10141660983659939,0.31062983198122296', '-0.10144669177122667,0.21137065920530718', '-0.09970647931670651,0.14693247405352197', '-0.10059405302501234,0.07002866429751754', '-0.09924470453726442,-0.01344572102397941']
co_ordinate=[goal]
for i,node in enumerate(path):
    if i%20==0:
        a=node.split(',')
        co_ordinate.append([float(a[0]),float(a[1])])
co_ordinate=[[0. , 3.5], [0.025990201108921245, 3.249569428646434], [1.150000000000003, 2.986228606900573], [1.7416544365420272, 1.9275181689068903], [0.5424053596465499, 0.9695323623618224], [-0.09924470453726442, -0.01344572102397941]]
co_ordinate.reverse()
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 3.5
desired_position_.z = 0
pos_list=[]
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.01

# publishers
pub = None

# callbacks
def clbk_odom(msg):
    global position_
    global yaw_,pos_list
    
    # position
    position_ = msg.pose.pose.position
    pos_list.append((position_.x,position_.y))
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
def fix_yaw(node):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(node[1]- position_.y, node[0]- position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.233 if err_yaw > 0 else -0.233
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        change_state(1)

# Goes in the direction og the goal;simultenously correctes the yaw
def go_straight_ahead(node):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(node[1] - position_.y, node[0] - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(node[1] - position_.y, 2) + pow(node[0] - position_.x, 2))
    
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.033 if err_yaw > 0 else -0.033
        pub.publish(twist_msg)
    else:
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def reached(node):
    global dist_precision_
    err_pos = math.sqrt(pow(node[1] - position_.y, 2) + pow(node[0] - position_.x, 2))
    
    if err_pos > dist_precision_:
        return True
    else:
        return False


def main():
    global pub, co_ordinate
    
    rospy.init_node('go_to_point')
    # This node publishes to the /cmd_vel topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    # This node subscribes to the /odom topic
    rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        for node in co_ordinate:
            print(node)
            while(reached(node)):
                if state_ == 0:
                    fix_yaw(node)
                elif state_ == 1:
                    go_straight_ahead(node)
                elif state_ == 2:
                    done()
                    break
                else:
                    rospy.logerr('Unknown state!')
        done()
        print('Reached')
        print(pos_list)
        break
        

if __name__ == '__main__':
    main()