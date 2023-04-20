#!/usr/bin/env python3
# import ros stuff
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import numpy as np

import math
goal=[2,2]
path=['1.8031273667378223,0.9500000000000026', '1.803184153993659,0.9333771701843022', '1.8071030768294867,0.8836184297334916', '1.8123948002310206,0.8500000000000025', '1.8107778779854555,0.8345593634473135', '1.8095953862611383,0.7896246779232499', '1.8111476499328745,0.7500000000000023', '1.8098896187239903,0.7443653580470718', '1.8028391651876379,0.6953228895233494', '1.79981238675202,0.6500000000000024', '1.7980579311436482,0.6445499896182091', '1.7858791679608221,0.594539289298226', '1.7782880494957503,0.5500000000000023', '1.7748152112145066,0.5415663127287137', '1.7571429533078817,0.4891349978117322', '1.746590427702627,0.4500000000000021', '1.7383202605114478,0.43323136750869007', '1.7141456619342221,0.3762620000572481', '1.7045884869658159,0.3500000000000021', '1.6855758005588763,0.3165368663182515', '1.6527795183933145,0.25182012621514893', '1.651952993581252,0.2500000000000017', '1.61155857844256,0.18683227734491842', '1.5899602974333649,0.1500000000000014', '1.5637532988739609,0.11564749991865181', '1.5171498900362526,0.05000000000000143', '1.5068912024162993,0.03831583070995087', '1.4383616785866784,-0.04468157857847077', '1.4342033411109207,-0.049999999999998046', '1.3555275665086415,-0.13415979658653132', '1.3413946804846117,-0.14999999999999836', '1.254820853603251,-0.23336625797491573', '1.2382279720942813,-0.24999999999999845', '1.1512811892126509,-0.3264253735321544', '1.1347415328579766,-0.3207858673219023', '1.0747529433110992,-0.3015109662462879', '1.0259415820751696,-0.28485846607031895', '0.9668973088086336,-0.26589268272629574', '0.9183712467241824,-0.24933539646202074', '0.8590809608698762,-0.23028908323971065', '0.8098594010866054,-0.213499689076857', '0.7491127070143627,-0.19397661649956932', '0.6981577176071315,-0.17660877111941808', '0.6346228053993951,-0.15617288201602908', '0.5807486639206478,-0.13783104930429912', '0.5128487689268743,-0.1159659150871436', '0.45460345112364486,-0.09616594962287184', '0.38034522891781486,-0.0722184126094163', '0.3158369478840375,-0.05032990896874012', '0.23254765458863402,-0.023423727328585997', '0.15916843874028708,0.0014217880074985922', '0.12138137514052211,0.013652341590721262', '0.0929195889131631,-0.042399393388682816', '0.09161978346054567,-0.044874807730821364']
co_ordinate=[goal]
for i,node in enumerate(path):
    if i%20==0:
        a=node.split(',')
        co_ordinate.append([float(a[0]),float(a[1])])
co_ordinate.reverse()
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 2
desired_position_.y = 2
desired_position_.z = 0
pos_list=[]
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.05

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
        twist_msg.angular.z = 0.1 if err_yaw > 0 else -0.1
    
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
    
    rospy.init_node('Voronoi')
    # This node publishes to the /cmd_vel topic
    pub = rospy.Publisher('/tb3_2/cmd_vel', Twist, queue_size=1)
    # This node subscribes to the /odom topic
    rospy.Subscriber('/tb3_2/odom', Odometry, clbk_odom)
    
    rate = rospy.Rate(10)
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
                rate.sleep()
        done()
        print('Reached')
        print(pos_list)
        break
        

if __name__ == '__main__':
    main()