#!/usr/bin/env python

import rospy, mavros, time, os
import math
from geometry_msgs.msg import PoseStamped, Twist
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import Float64, String
from tf.transformations import *
from sensor_msgs.msg import LaserScan, Imu
import keyboard
import pyperclip

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

#callback method for position subscriber
def position_cb(get_pose):
   global altitude
   altitude = get_pose.pose.position.z
   global x_pos
   x_pos = get_pose.pose.position.x
   global y_pos
   y_pos = get_pose.pose.position.y
   global current_yaw
   qx=get_pose.pose.orientation.x
   qy=get_pose.pose.orientation.y
   qz=get_pose.pose.orientation.z
   qw=get_pose.pose.orientation.w
   current_yaw = math.atan2( 2.0*(qw*qz + qx*qy),  1.0 - 2.0*(qy**2 + qz**2) )

#callback method for altitude subscriber
def alt_cb(data):
    global rel_alt
    rel_alt = data.relative

#callback method for imu data (accelerometer)
def imu_cb(data):
    global x_accel
    x_accel = data.linear_acceleration.x
    global y_accel
    y_accel = data.linear_acceleration.y
    global z_accel
    z_accel = data.linear_acceleration.z

#callback method for user input subscriber
def input_cb(user_input):
    global input_str
    input_str = user_input.data

#callback method for rplidar subscriber
def rplidar_cb(data):
    global range_min
    range_min = data.range_min
    global range_max
    range_max = data.range_max
    global ranges
    ranges = data.ranges


mavros.set_namespace()

########## Define Publishers
local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
body_vel_pub = rospy.Publisher(mavros.get_topic('setpoint_velocity', 'cmd_vel_unstamped'), Twist, queue_size=10)
set_geo_pub = rospy.Publisher(mavros.get_topic('global_position', 'set_gp_origin'), GeoPointStamped, queue_size=10)

########## Define Subscribers
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
local_pos_sub = rospy.Subscriber(mavros.get_topic('local_position', 'pose'), PoseStamped, position_cb)
alt_sub = rospy.Subscriber(mavros.get_topic('altitude'), Altitude, alt_cb)
imu_sub = rospy.Subscriber(mavros.get_topic('imu', 'data'), Imu, imu_cb)
input_sub = rospy.Subscriber('UserInput', String, input_cb)
laser_sub = rospy.Subscriber('scan', LaserScan, rplidar_cb)

########## Define Services
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode) 

targetHeight = 1.0  # Fly 1 meter high

geo_pos = GeoPointStamped()
geo_pos.position.latitude = 0
geo_pos.position.longitude = 0
geo_pos.position.altitude = 0

initial_pose = PoseStamped()
initial_pose.pose.position.x = 0
initial_pose.pose.position.y = 0
initial_pose.pose.position.z = 0

danger_zone = 1.0  # Dangerous Obstacle Distance
input_str = "ready" # Initialize input command

os.system('clear')  # Clear screen

print('Connecting...')

def position_control():
    rospy.init_node('offb_python_node', anonymous=True)
    prev_state = current_state
    freq = 20
    rate = rospy.Rate(freq) # MUST be more then 2Hz
    
    recent_angles = [0]
    recent_displacements = [0]
    global angle
    angle = 0
    global displacement
    displacement = 0
    global xvel
    xvel = 0
    global yvel
    yvel = 0
    global zvel
    zvel = 0

    global x_pos
    x_pos = 0
    global y_pos
    y_pos = 0
    global current_yaw
    current_yaw = 0
    global range_min
    range_min = 0
    cnt = 0
    global Height
    Height = 1
    zyaw = 0
    forward_speed_delta = 5
    angular_speed_delta = 5
    forward_speed_limit = -2
    angular_speed_limit = 2
    consecutive_misses = 0
    max_misses_tolerated = 200000

    max_unknown_count = 100
    unknown_count = 0
    last_direction = "U"
    
    pose = PoseStamped()
    pose.pose.position.x = x_pos
    pose.pose.position.y = y_pos
    pose.pose.position.z = targetHeight

    # send a few setpoints before starting
    
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()
    

    # wait for FCU connection
    while not current_state.connected:
        rate.sleep()

    last_request = rospy.get_rostime()
    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > rospy.Duration(5.)):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)
        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
            offboard_started_time = rospy.get_rostime()
        prev_state = current_state

        now = rospy.get_rostime()
        
        os.system('clear')
	    
        ######################################################################################################################
        ############# READY MODE ##############################################################################################
        #######################################################################################################################
        if input_str == "ready":
            print("Ready for mode")

            reset_pos = True
            first_run = True


            rate.sleep()

	    ######################################################################################################################
        ############# SAFETY BUTTON MODE ##############################################################################################
        #######################################################################################################################
        if input_str == "sb":
            print("Safety Button mode")

            reset_pos = True
            first_run = True

            rate.sleep()
        

        #######################################################################################################################
        ############# TAKEOFF MODE ############################################################################################
        #######################################################################################################################
        if input_str == "to":
            print("Takeoff")

            if reset_pos:
                pose = PoseStamped()
                pose.pose.position.x = x_pos
                pose.pose.position.y = y_pos
                pose.pose.position.z = targetHeight

                set_geo_pub.publish(geo_pos)


            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)

            reset_pos = False


            rate.sleep()
        #####################################################################
        elif input_str == "f":
            print("Moving forward")
            pose = PoseStamped()
            pose.pose.position.x = x_pos + 1
            pose.pose.position.y = y_pos
            pose.pose.position.z = targetHeight
            set_geo_pub.publish(geo_pos)
            
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)
            rate.sleep()
        elif input_str == 'b':
            print("Moving back")
            pose = PoseStamped()
            pose.pose.position.x = x_pos -1
            pose.pose.position.y = y_pos
            pose.pose.position.z = targetHeight
            set_geo_pub.publish(geo_pos)
            
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)
            rate.sleep()
        elif input_str == 'sr':
            print("Sliding right")
            pose = PoseStamped()
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos - 1
            pose.pose.position.z = targetHeight
            set_geo_pub.publish(geo_pos)
            
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)
            rate.sleep()
        elif input_str == 'sl':
            print("Sliding left")
            pose = PoseStamped()
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos +1
            pose.pose.position.z = targetHeight
            set_geo_pub.publish(geo_pos)
            
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)
            rate.sleep()
        elif input_str == 'gu':
            print("Going up")
            pose = PoseStamped()
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos
            pose.pose.position.z = altitude + 0.5
            set_geo_pub.publish(geo_pos)
            
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)
            rate.sleep()
        elif input_str == 'gd':
            print("Going down")
            pose = PoseStamped()
            pose.pose.position.x = x_pos
            pose.pose.position.y = y_pos
            pose.pose.position.z = altitude - 0.5
            set_geo_pub.publish(geo_pos)
            
            if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                arming_client(True)
                set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                last_request = now

            local_pos_pub.publish(pose)
            rate.sleep()
        #######################################################################################################################
        ############# LAND MODE ###############################################################################################
        #######################################################################################################################
        
        elif input_str == "l":
            print("Landing")

            reset_pos = True

            if current_state.mode != "AUTO.LAND" and (now - last_request > rospy.Duration(5.)):
                set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
                last_request = now

            rate.sleep()

        #######################################################################################################################
        ############# RETURN TO LAUNCH MODE ###############################################################################################
        #######################################################################################################################
        elif input_str == "return to launch":
            print("Returning to Launch Location")

            #reset_pos = True

            if current_state.mode != "AUTO.RTL" and (now - last_request > rospy.Duration(5.)):
                    set_mode_client(base_mode=0, custom_mode="AUTO.RTL")
                    last_request = now

            rate.sleep()


        #######################################################################################################################
        ############# DISARM  #################################################################################################
        #######################################################################################################################
        elif input_str == "d":
                print("Disarming")

                arming_client(False)

                rate.sleep()

        #######################################################################################################################
        ############# HOVER MODE ##############################################################################################
        #######################################################################################################################
        elif input_str == "h":
            print("Hovering")
	    
            twist=Twist()

            xvel = 0
            yvel = 0

            # Maintain Altitude
            zvel = (targetHeight-rel_alt)*2
            '''
            if altitude < targetHeight:
                zvel = 0.2
            else:
                zvel = 0
            '''
    
            print(zvel)

            twist.linear.x = xvel
            twist.linear.y = yvel
            twist.linear.z = zvel
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0
            body_vel_pub.publish(twist)

	    

            rate.sleep()

        #######################################################################################################################
        ############# LINE FOLLOW MODE ########################################################################################
        #######################################################################################################################
        elif input_str == "t":
            print("Following Line\n")

            twist=Twist()
            
            direction = pyperclip.paste()
            print(pyperclip.paste())

            if direction == "R":
			    #xvel += 0.05
			    # Turn yaw towards the right and slow down
			    #twist.linear.x, twist.linear.y = mov_xy(xvel, yvel-0.05, current_yaw)			
                twist.linear.x, twist.linear.y = mov_xy(xvel+1,yvel,current_yaw)
                twist.angular.z -= 2
			    #twist.angular.z -= 0.5
                pyperclip.copy("")
		
            elif direction == "L":
			    # Turn yaw towards the left and slow down
                twist.linear.x, twist.linear.y = mov_xy(xvel+1,yvel,current_yaw)
                twist.angular.z += 2
                pyperclip.copy("")
		
            elif direction == "N":
                # Strighten yaw and speed up to go forward
                twist.linear.x, twist.linear.y = mov_xy(xvel+2,yvel,current_yaw)
                twist.angular.z = 0
                pyperclip.copy("")

            elif direction == "U":
                # Slow down and straighten yaw for unknown values
                twist.linear.x, twist.linear.y = mov_xy(xvel, yvel, current_yaw)	
                #twist.linear.x, twist.linear.y = mov_xy(max(xvel-0.05,0),yvel,current_yaw)	#prevent going backwards
                twist.angular.z = 0
                print("I got U info")
                pyperclip.copy("")

            else:
                # Clipboard is empty or has undefined value so do nothing
                print("I got nothin man")
                pass
            
            zvel = (targetHeight - rel_alt)*2
            twist.linear.z = zvel		

            #print("x: " + str(twist.linear.x))
            #print("y: " + str(twist.linear.y))
            #print("z: " + str(twist.linear.z) + "\n")

            print("z: " + str(twist.angular.z))

            body_vel_pub.publish(twist)
            rate.sleep()


	#######################################################################################################################
        ############# TAKEOFF AND LINE FOLLOW MODE ############################################################################
        #######################################################################################################################
        elif input_str == "tt":
            print("Takeoff and Follow Line\n")

		
            if first_run:
                print("Takeoff")

                if reset_pos:
                    pose = PoseStamped()
                    pose.pose.position.x = x_pos
                    pose.pose.position.y = y_pos
                    pose.pose.position.z = targetHeight

                set_geo_pub.publish(geo_pos)

                if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
                    arming_client(True)
                    set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                    last_request = now

                local_pos_pub.publish(pose)

                reset_pos = False
                first_run = False
	
            else:
                twist=Twist()
                direction = pyperclip.paste()
                print(pyperclip.paste())
                pyperclip.copy("")

            if last_direction != direction: # and direction != "U":
                unknown_count = 0

            if consecutive_misses<max_misses_tolerated:
                try:
                    print(str(xvel)+"\n"+str(zyaw))

                    if direction == "U":
                        unknown_count += 1
                        if unknown_count >= max_unknown_count or last_direction == "U":
                            xvel -= forward_speed_delta
                            unknown_count = 0

                            if abs(zyaw) == angular_speed_delta:
                                zyaw == 0
                            elif zyaw < 0:
                                zyaw += angular_speed_delta + angular_speed_delta
                            elif zyaw > 0:
                                zyaw -= angular_speed_delta + angular_speed_delta

                            consecutive_misses = 0
                            last_direction = direction
                        else:
                            direction = last_direction

                    elif direction == "R":
                        xvel -= forward_speed_delta
                        zyaw += angular_speed_delta
                        consecutive_misses = 0
                        last_direction = direction

                    elif direction == "L":
                        xvel -= forward_speed_delta
                        zyaw -= angular_speed_delta
                        consecutive_misses = 0
                        last_direction = direction

                    elif direction == "N":
                        xvel -= (forward_speed_delta + forward_speed_delta)
						
                        if abs(zyaw) == angular_speed_delta:
                            zyaw == 0
                        elif zyaw < 0:
                            zyaw += angular_speed_delta + angular_speed_delta
                        elif zyaw > 0:
                            zyaw -= angular_speed_delta + angular_speed_delta
                        consecutive_misses = 0
                        last_direction = direction

                    elif direction != "U":
                        xvel += forward_speed_delta
						
                        if abs(zyaw) == angular_speed_delta:
                            zyaw == 0						
                        elif zyaw < 0:
                            zyaw += angular_speed_delta + angular_speed_delta
                        elif zyaw > 0:
                            zyaw -= angular_speed_delta + angular_speed_delta

                        consecutive_misses += 1

                        if xvel > 0:
                            xvel = 0
                        elif xvel < forward_speed_limit:
                            xvel = forward_speed_limit
                        if zyaw > angular_speed_limit:
                            zyaw = angular_speed_limit
                        elif zyaw < (-angular_speed_limit):
                            zyaw -= angular_speed_limit

                    twist.linear.x, twist.linear.y = mov_xy(xvel,0,current_yaw)
                    twist.linear.z = (targetHeight - rel_alt)*2
                    twist.angular.z = zyaw
                    body_vel_pub.publish(twist)
                except:
                    print("Fail")
            else:
                print("Landing\n")
                reset_pos = True
                if current_state.mode != "AUTO.LAND" and (now - last_request > rospy.Duration(5.)):
                    set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
                    last_request = now
            rate.sleep()

def mov_x(spd,yaw):
  return (math.cos(yaw)*spd,math.sin(yaw)*spd)
def mov_y(spd,yaw):
  return (math.sin(yaw)*spd,math.cos(yaw)*spd)
def mov_xy(x,y,yaw):
  x1,y1=mov_x(x,yaw)
  x2,y2=mov_y(y,yaw)
  return (x1+x2,y1+y2)

 
if __name__ == '__main__':
    try:
        position_control()
    except rospy.ROSInterruptException:
        pass
