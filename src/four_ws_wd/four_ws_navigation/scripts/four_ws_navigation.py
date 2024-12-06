#!/usr/bin/env python3
import rospy, os, sys
import time
import math
import string
from std_msgs.msg import Header, String, Int32
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf
import select, termios, tty
import copy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import roslib; roslib.load_manifest('visualization_marker_tutorials')
import threading
from agcar_ctrl_msg.msg import pump_ctrl

count_marker = 0
MARKERS_MAX = 200
markerArray = MarkerArray()

vel_msg = Twist()
joy_msg = Twist()
location_msg = [0.0, 0.0, 0.0]
twist_msg  = [0.0,0.0,0.0]
goal_msg = [0.0,0.0,0.0]
goal_get = 0
quit_flag = 0

target_odom = Odometry()

origal_location = [0.0,0.0,0.0,0.0]
origal_flag = 0

test_goal = [[0,0],[8,0],[8,-5],[0,-5],[0,-10],[8,-10],[8,-15],[0,-15]]
test1_goal = [[0,0],[10,0],[12,-5],[0,-5],[0,-10],[14,-10],[15,-12.5],[15,-15],[0,-15]]
test2_goal = [[0,0],[5,0],[10,0]]

target_path = Path()

xy_msg = []
mission_count = 0
mission_state = 0

yaw_flag = 0
yaw_orignal = 0


class PID:
    def __init__(self):
        self.error = [0.0,0.0]
        self.Iout = 0
        

    def LimitMax(self,val,val_max):
        if val > val_max:
            val = val_max
        elif val< -val_max:
            val = -val_max
        return val

    def PID_Calc(self,Ref,Set,kp,ki,kd,max_out,max_iout):
        self.error[0] = Ref - Set
        pout = kp * self.error[0]
        self.Iout += ki *self.error[0]
        dout =kd*(self.error[0] - self.error[1])
        self.Iout = PID.LimitMax(self,self.Iout,max_iout)
        pidout =  pout + self.Iout + dout
        pidout = PID.LimitMax(self,pidout,max_out)
        self.error[1] = self.error[0]
        return pidout

def turn_tf(raw_pos,yaw_angle):
    tf_pos = []
    tf_point = [0,0]
    for i in range(len(raw_pos)):
        tf_point[0] = raw_pos[i][0]*math.cos(-yaw_angle) + raw_pos[i][1]*math.sin(-yaw_angle)
        tf_point[1] = raw_pos[i][1]*math.cos(-yaw_angle) - raw_pos[i][0]*math.sin(-yaw_angle)
        tf_pos.append(copy.deepcopy(tf_point))
    print(tf_pos)
    return tf_pos


def remap(output_max,output_min,input_max,input_min,input_value):
    input_cha = input_max - input_min
    present = (input_value-input_min)/input_cha
    output_cha = output_max-output_min
    output_value = output_min+(output_cha*present)
    return output_value


def pub_target_path(x,y):
    global target_path
    target_path.header.frame_id = "world"
    target_path.header.stamp = rospy.Time.now()
    p = PoseStamped()
    p.pose.position.x =x
    p.pose.position.y =y
    p.pose.position.z = 0.0
    target_path.poses.append(p)
    path_pub.publish(target_path)

def mark_point(x,y):
    global count_marker
    global MARKERS_MAX
    global markerArray
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 1.2
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.6
    if(count_marker > MARKERS_MAX):
       markerArray.markers.pop(0)
    markerArray.markers.append(marker)
     # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1
   # Publish the MarkerArray
    Marker_publisher.publish(markerArray)
    count_marker += 1

def mark_point_red(x,y):
    global count_marker
    global MARKERS_MAX
    global markerArray

    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.6
    if(count_marker > MARKERS_MAX):
       markerArray.markers.pop(0)
    markerArray.markers.append(marker)
     # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1
   # Publish the MarkerArray
    Marker_publisher.publish(markerArray)
    count_marker += 1

def two_point_distance(x1,y1,x2,y2):
    cha_distance = math.sqrt(math.pow((x2-x1),2)+math.pow((y2-y1),2))
    return cha_distance

def linear_Inter(ox,oy,gx,gy,num,tole):
    if math.fabs(ox)<0.1:
        ox = 0
    if math.fabs(oy)<0.1:
        oy = 0
    if two_point_distance(ox,oy,gx,gy)>tole:
        dx = (gx-ox)/num
        x_new = ox
        y_new = oy
        new_point_bag = []
        while two_point_distance(x_new,y_new,gx,gy)>tole:
            if (gx-ox) !=0:
                slope = (gy-oy)/(gx-ox)
                x_new +=dx
                y_new +=slope*(dx)
            else:
                y_new +=dx
            new_point = [x_new,y_new]
            new_point_bag.append(copy.deepcopy(new_point))
            mark_point_red(x_new,y_new)
            #rospy.sleep(1)
            #print(new_point)
            #print(slope)
        #print(new_point_bag)
        new_point = []
        return new_point_bag
    else:
        return 0
    
def linear_Inter2(ox,oy,gx,gy,dx,tole):
    if math.fabs(ox)<0.1:
        ox = 0
    if math.fabs(oy)<0.1:
        oy = 0
    if two_point_distance(ox,oy,gx,gy)>tole:
        if (gx-ox) !=0:
            num = math.fabs((gx-ox)/dx)
            if (gx-ox)<0:
                dx = -dx
        else:
            num = math.fabs((gy-oy)/dx)
            if (gy-oy)<0:
                dx = -dx
        x_new = ox
        y_new = oy
        new_point_bag = []
        num = int(num)
        for i in range(num):
            if two_point_distance(x_new,y_new,gx,gy)>tole:
                if (gx-ox) !=0:
                    slope = (gy-oy)/(gx-ox)
                    x_new +=dx
                    y_new +=slope*(dx)
                else:
                    y_new +=dx
                new_point = [x_new,y_new]
                new_point_bag.append(copy.deepcopy(new_point))
                mark_point_red(x_new,y_new)
            new_point = []
        return new_point_bag
    else:
        return 0

def cos_linear(point1,point2,point3):
    vector1 = [0,0]
    vector1[0] = point2[0]-point1[0]
    vector1[1] = point2[1]-point1[1]
    vector2 = [0,0]
    vector2[0] = point3[0]-point2[0]
    vector2[1] = point3[1]-point2[1]
    ab = vector1[0]*vector2[0]+vector1[1]*vector2[1]
    a = two_point_distance(point1[0],point1[1],point2[0],point2[1])
    b = two_point_distance(point2[0],point2[1],point3[0],point3[1])
    cos_ab = ab /a*b
    return cos_ab

def target_odom_pose(point):
    global target_odom
    global target_odom_pub
    target_odom.header.frame_id="world"
    target_odom.header.stamp = rospy.Time.now()
    target_odom.pose.pose.position.x = point[0]
    target_odom.pose.pose.position.y = point[1]
    target_odom.pose.pose.position.z = 0
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    target_odom.pose.pose.orientation.x = quat[0]
    target_odom.pose.pose.orientation.y = quat[1]
    target_odom.pose.pose.orientation.z = quat[2]
    target_odom.pose.pose.orientation.w = quat[3]
         
    target_odom.child_frame_id = "base_link"
    target_odom.twist.twist.linear.x = 0
    target_odom.twist.twist.linear.y = 0
    target_odom.twist.twist.angular.z = 0
       
    target_odom_pub.publish(target_odom)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def callback_controller(data):
    global joy_msg
    
    joy_msg.linear.x = data.axes[1]
    joy_msg.linear.y = data.axes[0]
    joy_msg.angular.z = data.axes[2]

def callback_odom(data):
    global location_msg
    global twist_msg
    location_msg[0] = data.pose.pose.position.x
    location_msg[1] = data.pose.pose.position.y
    (r,p,y) =  tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
    twist_msg [0] = data.twist.twist.linear.x
    twist_msg [1] = data.twist.twist.linear.y
    twist_msg [2] = data.twist.twist.angular.z
    location_msg[2] = y

def callback_goal(data):
    global goal_msg
    global goal_get 
    goal_msg[0] = data.pose.position.x
    goal_msg[1] = data.pose.position.y
    goal_msg[2] = tf.transformations.euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    print ("[INFO] Goal:[x:%f  , y:%f]" %(goal_msg[0] , goal_msg[1]))
    mark_point(goal_msg[0],goal_msg[1])
    goal_get = 1

def callback_xy_goal(data):
    global xy_msg
    global mission_count
    global mission_state
    xy = [0.0,0.0,0.0]
    if mission_state==1:
        if data.poses[-1].pose.position.z ==1:
            mission_state = 2
            print ("[INFO]Mission Receive Complete!\n")
        else:
            xy[0] = data.poses[-1].pose.position.x
            xy[1] = data.poses[-1].pose.position.y
            xy_msg.append(copy.deepcopy(xy))
            print ("[INFO]Receive Mission No:%d\n"%(mission_count))
            print ("[INFO]Point[%f,%f] "%(xy[0],xy[1]))
            mission_count +=1
            mission_state = 1

def Set_Mode():
    global quit_flag 
    mode_flag = 0
    rate = rospy.Rate(1)
    print ("[0] Go Line (X Y axis)\n")
    print ("[1] MAV_STATION MODE \n")
    print ("[2] Single Point Navigation\n")
    print ("[3] Multi Point Navigation\n ")
    print ("[4] EX_PID_NAV\n")
    print ("[5] EX_PID_NAV-SINPATH\n")
    print ("[6] EX_PURE_FOLLOW_NAV\n")
    print ("[7] LOVE Navigation\n ")

    mode = int(input("[Enter] Mode Number:\n"))
    while mode_flag ==0:
        if mode ==1:
            mode_flag =1
        elif mode ==2:
            mode_flag =1
        elif mode ==3:
            mode_flag =1
        elif mode ==0:
            mode_flag =1
        elif mode ==4:
            mode_flag =1
        elif mode ==5:
            mode_flag =1
        elif mode ==6:
            mode_flag =1
        elif mode ==7:
            mode_flag =1
        elif mode ==8:
            mode_flag =1
        else:
            print ("[ERROR] Wrong Number\n")
            mode = int(input("[Enter] Mode Number:\n"))
            mode_flag =0
        rate.sleep()
    vel = float(input("[Enter] Vel: \n"))
    print ("[INFO] MODE:[%d]  Vel:%f \n"%(mode,vel))
    quit_flag = 1
    return mode,vel

def PN(value):
    if value>0:
        return 1
    elif value<0:
        return -1
    else:
        return 0 

def goal_planner (goal_x,goal_y,location_x,location_y,location_yaw,vel_speed,goal_tole,pid):
    global Distance_PID
    global Yaw_PID
    global yaw_flag
    global yaw_orignal
    
    if yaw_flag == 0:
        yaw_orignal = location_yaw
        print(yaw_orignal)
        yaw_flag=1

    Dn = goal_x - location_x
    De = goal_y - location_y
    Distance =  math.sqrt(math.pow(Dn,2)+math.pow(De,2))
    Dx = Dn*math.cos(location_yaw) + De*math.sin(location_yaw)
    Dy = De*math.cos(location_yaw) - Dn*math.sin(location_yaw)
    vel_Sum = Distance_PID.PID_Calc(Distance,0,pid[0],pid[1],pid[2],vel_speed,pid[3])
    if math.fabs(location_yaw-yaw_orignal)>0.05:
        wz_vel  = 1*Yaw_PID.PID_Calc(location_yaw,yaw_orignal,1,0.001,0,1,0.5)
        wz_vel_new = remap(0.67,-0.67,0.05,-0.05,wz_vel)
    else:
        wz_vel_new =0
    theth = math.atan2(Dy,Dx)
    vel_x = math.fabs(vel_Sum *math.cos(theth))*PN(Dx)
    vel_y = math.fabs(vel_Sum *math.sin(theth))*PN(Dy)
    #print ("[TEST]vel_x:%f\n" %(vel_x))
    #print ("[TEST]vel_y:%f\n" %(vel_y))
    #print ("[TEST]vel_wz:%f\n" %(wz_vel))
    #print ("[TEST]Distance:%f\n" %(Distance))
    #if math.fabs(Dx)<goal_tole :
    #    vel_x = 0
    #if math.fabs(Dy)<goal_tole :
    #    vel_y = 0
    #if math.fabs(Dy)>goal_tole  and math.fabs(Dx)<1:
    #    vel_x = 0
    if math.fabs(Dn)<goal_tole and math.fabs(De)<goal_tole:
        vel_y = 0
        vel_x = 0
        wz_vel_new = 0
        arrive_flag = 1
    else:
        arrive_flag = 0
    return vel_x , vel_y, arrive_flag,wz_vel_new

def smart_planner(goal_x,goal_y,location_x,location_y,location_yaw,vel_speed,goal_tole,pid):
    global Distance_PID
    global Yaw_PID
    global origal_location
    global origal_flag

    Dn = goal_x - location_x
    De = goal_y - location_y
    Distance =  math.sqrt(math.pow(Dn,2)+math.pow(De,2))
    Dx = Dn*math.cos(location_yaw) + De*math.sin(location_yaw)
    Dy = De*math.cos(location_yaw) - Dn*math.sin(location_yaw)
    if origal_flag==0:
        origal_location[0] = Dx
        origal_location[1] = Dy
        origal_location[2] = location_yaw
        origal_flag=1
        t = math.atan2(origal_location[1],origal_location[0]) 
        #rint("[flag=1]\n")
    vel_Sum = Distance_PID.PID_Calc(Distance,0,pid[0],pid[1],pid[2],vel_speed,pid[3])
    #if math.fabs(location_yaw)>0.05:
    #   #wz_vel  = 1*Yaw_PID.PID_Calc(location_yaw,0,1,0.001,0,1,0.5)
    #   pass
    #else:
    wz_vel = 0
    theth = math.atan2(origal_location[1],origal_location[0]) 
    #if math.fabs(theth-location_yaw)
    vel_x = math.fabs(vel_Sum *math.cos(theth))*PN(Dx)
    vel_y = math.fabs(vel_Sum *math.sin(theth))*PN(Dy)
    #print ("[TEST]vel_x:%f\n" %(vel_x))
    #print ("[TEST]vel_y:%f\n" %(vel_y))
    #print ("[TEST]vel_wz:%f\n" %(wz_vel))
    #print ("[TEST]Distance:%f\n" %(Distance))
    if math.fabs(Dn)<goal_tole and math.fabs(De)<goal_tole:
        vel_y = 0
        vel_x = 0
        wz_vel = 0
        origal_flag = 0
        arrive_flag = 1
    else:
        arrive_flag = 0
    return vel_x , vel_y, arrive_flag,wz_vel

def pid_nav_planner(nav_points,location_data,vel_max,linear_dx,ex_distance,pid):
    #global var
    global vel_msg
    global pub 
    #check points distance
    old_count = len (nav_points)
    points_pkg = []
    for i in range(old_count):
        if i<old_count-1:
            if two_point_distance(nav_points[i][0],nav_points[i][1],nav_points[i+1][0],nav_points[i+1][1])>linear_dx:
                
                points_pkg.append(nav_points[i])
                linear_bag = linear_Inter2(nav_points[i][0],nav_points[i][1],nav_points[i+1][0],nav_points[i+1][1],linear_dx,linear_dx)
                print(linear_bag)
                points_pkg.extend(linear_bag)
            else:
                points_pkg.append(nav_points[i])
        else:
            points_pkg.append(nav_points[i])
    #mark points
    points_count = len(points_pkg)
    # for i in range(points_count):
    #     mark_point_red(points_pkg[i][0],points_pkg[i][1])
    #     pub_target_path(points_pkg[i][0],points_pkg[i][1])
    #     # target_odom_pose(points_pkg[i])
    # #pid follow
    points_count = points_count-1
    follow_flag = 1
    index = 0
    while index <= points_count and follow_flag==1:
        
        if (index-30 >0):
            target_odom_pose(points_pkg[index-30]) #相位差！！！！
        else:
            target_odom_pose(points_pkg[index])    

        if index>0 and index<points_count:
            if cos_linear(points_pkg[index-1],points_pkg[index],points_pkg[index+1])<0:
                Cim = 0.5
                
            else:
                Cim = 1
        if two_point_distance(location_data[0],location_data[1],points_pkg[index][0],points_pkg[index][1]) > ex_distance:
            vel_msg.linear.x , vel_msg.linear.y, Arrvie_flag, vel_msg.angular.z  = goal_planner(points_pkg[index][0],points_pkg[index][1],location_data[0],location_data[1],location_data[2],vel_max,0.1,pid)
            pub.publish(vel_msg)
        elif index == points_count:
            while Arrvie_flag==0:
                vel_msg.linear.x , vel_msg.linear.y, Arrvie_flag, vel_msg.angular.z  = goal_planner(points_pkg[index][0],points_pkg[index][1],location_data[0],location_data[1],location_data[2],vel_max,0.1,pid)
                vel_msg.linear.x = Cim*vel_msg.linear.x
                vel_msg.linear.y = Cim*vel_msg.linear.y
                pub.publish(vel_msg)
            follow_flag = 0
        else:
            if index <=points_count:
                index+=1
    return  1
        
def pure_follow_planner(nav_points,location_data,vel_max,linear_dx,ex_distance):
    #global var
    global vel_msg
    global pub 
    #check points distance
    old_count = len (nav_points)
    points_pkg = []
    for i in range(old_count):
        if i<old_count-1:
            if two_point_distance(nav_points[i][0],nav_points[i][1],nav_points[i+1][0],nav_points[i+1][1])>linear_dx:
                
                points_pkg.append(nav_points[i])
                linear_bag = linear_Inter2(nav_points[i][0],nav_points[i][1],nav_points[i+1][0],nav_points[i+1][1],linear_dx,linear_dx)
                print(linear_bag)
                points_pkg.extend(linear_bag)
            else:
                points_pkg.append(nav_points[i])
        else:
            points_pkg.append(nav_points[i])
    #mark points
    points_count = len(points_pkg)
    for i in range(points_count):
        mark_point_red(points_pkg[i][0],points_pkg[i][1])
        target_odom_pose(points_pkg[i])
    #pure follow
    points_count = points_count-1
    #print ("count"+str(points_count))
    follow_flag = 1
    index = 0
    Radio = 0
    V_speed = 0
    while index <= points_count and follow_flag==1:
        if two_point_distance(location_data[0],location_data[1],points_pkg[index][0],points_pkg[index][1]) > ex_distance or index == points_count:
            theth = math.atan2((points_pkg[index][1]-location_data[1]),(points_pkg[index][0]-location_data[0]))
            #print("yaw"+str(location_data[2]))
            # if  math.fabs(theth - location_data[2])>3.14:
            #     cha_theth = math.fabs(math.fabs(theth - location_data[2]) -6.28)
            # else:
            cha_theth = math.fabs(theth - location_data[2])

            #print("cha_theth:"+str(cha_theth))
            dis = two_point_distance(location_data[0],location_data[1],points_pkg[index][0],points_pkg[index][1])
            Radio =  dis/(2*math.sin(cha_theth))       
            V_speed = two_point_distance(location_data[0],location_data[1],points_pkg[index][0],points_pkg[index][1])
            V_speed = 0.6*V_speed
            if V_speed>vel_max :
                V_speed=vel_max
            wz = V_speed/Radio
            if math.fabs(wz) <0.01:
                wz = 0
            
            vel_msg.linear.x = V_speed
            if theth - location_data[2]>0:
                Cim = -1
            else:
                Cim = 1
            vel_msg.angular.z  = Cim*remap(0.68,-0.68,0.1,-0.1,wz)
            pub.publish(vel_msg)
            #rospy.sleep(1)
        else:
            if index <=points_count:
                index+=1
        if index == points_count:
            if two_point_distance(location_data[0],location_data[1],points_pkg[index][0],points_pkg[index][1])<0.3:
                follow_flag =0
            
        
            

def stop():
    global vel_msg
    global pub 
    vel_msg.linear.x = 0
    vel_msg.linear.y  =0
    vel_msg.angular.z  = 0
    pub.publish(vel_msg)

def make_sin_path():
    sin_path = []
    zero_point = [0,0]
    first_point = [1,0]
    theth = 0
    x_set = 0
    y_set = 0
    sin_path.append(zero_point)
    sin_path.append(first_point)
    while theth <6.28*2:
        theth += 0.02#
        y_set += 0.02#
        x_set = 3*math.sin(0.5*theth)+5
        sin_point=[0,0]
        sin_point[0] = x_set
        sin_point[1] = y_set
        sin_path.append(sin_point)
    end_point = [x_set+5,y_set]
    sin_path.append(end_point)
    return sin_path

def navigation():
    global vel_msg
    global goal_msg
    global location_msg
    global pub 
    global X_PID
    global Y_PID
    global goal_get
    global quit_flag

    global xy_msg
    global mission_count
    global mission_state

    global origal_flag

    global test_goal

    Point_bag =[ ]
    Length =0
    points_overflag = 0
    rate_nav = rospy.Rate(50)
    rate_wait = rospy.Rate(10)
    Arrvie_flag = 0

    while not rospy.is_shutdown():
        MODE , VEL = Set_Mode()
        while MODE == 0 and quit_flag ==1:
            Axis = int(input("[Enter]Axis(X[1]\Y[0]):\n"))
            Length  = float(input("[Enter]Length(+\-):\n"))
            distance = 0
            start_msg = [0,0]
            cha = [0,0]
            start_msg[0] = location_msg[0]
            start_msg[1] = location_msg[1]
            Arrvie_flag = 0
            while  Arrvie_flag ==0:
                cha[0]= location_msg[0]-start_msg[0]
                cha[1]= location_msg[1]-start_msg[1]
                distance =  math.sqrt(math.pow(cha[0],2)+math.pow(cha[1],2))
                if math.fabs(math.fabs(distance)-math.fabs(Length))<0.3:
                    vel_msg.linear.x =0
                    vel_msg.linear.y =0
                    Arrvie_flag = 1
                else:
                    if Axis == 1:
                        if Length>0:
                            vel_msg.linear.x =VEL
                            vel_msg.linear.y =0
                        else:
                            vel_msg.linear.x =-VEL
                            vel_msg.linear.y =0
                    else:
                        if Length>0:
                            vel_msg.linear.y =VEL
                            vel_msg.linear.x =0
                        else:
                            vel_msg.linear.y =-VEL
                            vel_msg.linear.x =0
                if Arrvie_flag ==1:
                    print ("[INFO]Done\n")
                print ("[TEST]x:%f\n" %(location_msg[0]))
                print ("[TEST]y:%f\n" %(location_msg[1]))
                print ("[TEST]Distance:%f\n" %(distance))
                pub.publish(vel_msg)
                rate_nav.sleep()
            quit_flag = int(input("[Enter]Continue:[1] / Quit:[0]\n"))


        

        while MODE==1 and quit_flag==1:
            mission_state =1
            print ("[INFO]Waiting Mission\n")
            pid = [0,0,0,0]
            pid[0] = 0.02
            pid[1] = 0.001
            pid[2] = 0.001
            pid[3] = 5
            while mission_state!=2:
                rate_wait.sleep()
            Arrvie_flag = 0
            for i in range(mission_count):
                print ("[INFO]Goto %d Point[%f,%f]"%(i+1,xy_msg[i][0],xy_msg[i][1]))
                while Arrvie_flag == 0:
                     vel_msg.linear.x ,  vel_msg.linear.y, Arrvie_flag, vel_msg.angular.z  = goal_planner(xy_msg[i][0],xy_msg[i][1],location_msg[0],location_msg[1],location_msg[2],VEL,0.3,pid)
                     pub.publish(vel_msg)
                     rate_nav.sleep()
                print ("[INFO]Arrive %d Point[%f,%f] "%(i+1,xy_msg[i][0],xy_msg[i][1]))
                Arrvie_flag = 0
            print ("[INFO]Mission Done!\n")
            mission_count =0

        while MODE==2 and quit_flag==1:
            print("[RVIZ] Waiting A Goal\n")
            print("[Enter] Enter [q] to quit\n")
            pid = [0,0,0,0]
            pid[0] = 0.02
            pid[1] = 0.001
            pid[2] = 0.001
            pid[3] = 5
            while  goal_get ==0 :
                key = getKey(0.5)
                if key == 'q':
                    quit_flag = 0
                    break
                rate_wait.sleep()
            if goal_get ==1 and quit_flag ==1:
                #print"[INFO] Goal:[x:%f  , y:%f] \n"%(goal_msg[0] , goal_msg[1] )
                Arrvie_flag = 0
                while Arrvie_flag == 0:
                    vel_msg.linear.x ,  vel_msg.linear.y, Arrvie_flag, vel_msg.angular.z  = goal_planner(goal_msg[0],goal_msg[1],location_msg[0],location_msg[1],location_msg[2],VEL,0.3,pid)
                    pub.publish(vel_msg)
                    rate_nav.sleep()
            if Arrvie_flag ==1:    
                goal_get = 0
                print ("[INFO]Done\n")

        while MODE==3  and quit_flag==1:
            print("[RVIZ] Waiting Goals And Enter [o] To Over\n")
            print("[Enter] Enter [q] to quit\n")
            pid = [0,0,0,0]
            pid[0] = 0.02
            pid[1] = 0.001
            pid[2] = 0.001
            pid[3] = 5
            count = 0
            while points_overflag == 0:
                key = getKey(0.5)
                if goal_get ==1:
                    count +=1
                    Point_bag.append(copy.deepcopy(goal_msg))
                    print("[Point %d]"%(count))
                    goal_get = 0
                if key == 'o' :
                    if len(Point_bag)>=2:
                        print("[Point]Total: %d  Points \n"%(count))
                        points_overflag= 1
                    else:
                        print("[Point]Points less than 2")
                if key =='q':
                    quit_flag = 0
                    Point_bag =[]
                    break
                rate_wait.sleep()
            Arrvie_flag = 0
            for i in range(count):
                print ("[INFO]Goto %d Point[%f,%f]"%(i+1,Point_bag[i][0],Point_bag[i][1]))
                while Arrvie_flag == 0:
                    vel_msg.linear.x ,  vel_msg.linear.y, Arrvie_flag , vel_msg.angular.z = goal_planner(Point_bag[i][0],Point_bag[i][1],location_msg[0],location_msg[1],location_msg[2],VEL,0.3,pid)
                    pub.publish(vel_msg)
                    rate_nav.sleep()
                print ("[INFO]Arrive %d Point[%f,%f] "%(i+1,Point_bag[i][0],Point_bag[i][1]))
                Arrvie_flag = 0
                rospy.sleep(1)
            if quit_flag == 1:
                print  ("[INFO]Done\n")
                points_overflag= 0
                Point_bag = []

        while MODE==4 and quit_flag==1:
            print("[4]EX_PID_NAV\n")
            pid = [0,0,0,0]
            pid[0] = 0.005
            pid[1] = 0.001
            pid[2] = 0.001
            pid[3] = 5
            sin_path = make_sin_path()
            pid_nav_planner(test_goal,location_msg,VEL,0.01,0.35,pid)
            print("[INFO]Done\n")
            quit_flag=0

        while MODE==5 and quit_flag==1:
            print("[5]EX_PID_NAV-SINPATH\n")
            pid = [0,0,0,0]
            pid[0] = 0.005
            pid[1] = 0.001
            pid[2] = 0.001
            pid[3] = 5
            sin_path = make_sin_path()
            pid_nav_planner(sin_path,location_msg,VEL,0.01,0.35,pid)
            print("[INFO]Done\n")
            quit_flag=0

        while MODE==6 and quit_flag==1:
            print("[6]PURE_FOLLOW_NAV\n")
            sin_path = make_sin_path()
            pure_follow_planner(test1_goal,location_msg,VEL,0.5,1.5)
            print("[INFO]Done\n")
            stop()
            quit_flag=0


        while MODE==7 and quit_flag==1:
            print ("LOVE")
            pid = [0,0,0,0]
            pid[0] = 0.02
            pid[1] = 0.001
            pid[2] = 0.001
            pid[3] = 5
            theth = 0
            while theth <6.2830:
                theth += 0.1 #
                y_set = 13 * math.cos(theth) - 5*math.cos(2*theth) - 2*math.cos(3*theth) -math.cos(4*theth)  #
                x_set = 16 * math.pow(math.sin(theth),3)
                y_set = y_set*0.5  #
                x_set = x_set*0.5
                Arrvie_flag = 0
                while Arrvie_flag == 0:
                    vel_msg.linear.x ,  vel_msg.linear.y, Arrvie_flag , vel_msg.angular.z = goal_planner(x_set,y_set,location_msg[0],location_msg[1],location_msg[2],VEL,0.2,pid)
                    pub.publish(vel_msg)
                    rate_nav.sleep()
            print  ("[INFO]Done\n")
            quit_flag =0

        

        rate_wait.sleep()


# def pump_control (pump_path,odom_location):
#     if 
    


# def thread_pump ():
#     pump_msg = pump_ctrl()
#     pump_rate =  rospy.Rate(100)
#     global pump_flag

#     while not rospy.is_shutdown():
#         if pump_flag ==0:
#             pump_msg.pump1_en =0
#             pump_msg.pump2_en =0
#         else pump_flag ==1:
#             pump_msg.pump1_en =1
#             pump_msg.pump2_en =1

#         pump_pub.publish(pump_pub)
#         pump_rate.sleep()


if __name__ == '__main__':
    global pub
    global Distance_PID
    global settings
    global Yaw_PID
    
    try:
        settings = termios.tcgetattr(sys.stdin)
        rospy.init_node('four_ws_navigation_node', anonymous=True)
        rospy.Subscriber("joy0", Joy, callback_controller)
        rospy.Subscriber("/odom",Odometry,callback_odom)
        rospy.Subscriber("/move_base_simple/goal",PoseStamped,callback_goal)
        rospy.Subscriber("/xy_goal_path",Path,callback_xy_goal)
        path_pub = rospy.Publisher('/target_path',Path,queue_size=1)
        target_odom_pub = rospy.Publisher('/target_odom',Odometry,queue_size=1)
        pump_pub = rospy.Publisher('/pump_ctrl',pump_ctrl,queue_size=1)

        #thread_pump_ = threading.Thread(target = thread_pump)
        #self.thread_pump_.start()

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        topic = 'visualization_marker_array'
        Marker_publisher = rospy.Publisher(topic, MarkerArray)
        Distance_PID = PID()
        Yaw_PID = PID()
        navigation()
        rospy.spin()
    except KeyboardInterrupt:
        stop()
        print("STOP!!!")
