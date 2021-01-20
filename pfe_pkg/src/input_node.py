#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String

os.system('clear')

def input_pub():
    pub = rospy.Publisher('UserInput', String, queue_size=2)
    rospy.init_node('UserInputNode', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    input_str = ""
    while not rospy.is_shutdown():
        if input_str == "":
            print("Please enter a mode \n")
        elif input_str == "to" or input_str == "tt" or input_str == "h" or input_str == "t" or input_str == "l" or input_str == "d" or input_str == "sr" or input_str == "sl" or input_str == "f" or input_str == "b" or input_str == "gu" or input_str == "gd":
	    print("Current mode: %s \n " % (input_str))
        else:
	    print("'%s' is not a valid mode. Please try again \n" % (input_str))
        print("Takeoff Mode --------------------> to ")
        print("Takeoff and Track Mode ----------> tt ")
        print("Hover Mode ----------------------> h ")
        print("Track Mode ----------------------> t ")
        print("Land Mode -----------------------> l ")
        print("Slide right----------------------> sr")
        print("Slide left-----------------------> sl")
        print("Move forward---------------------> f")
        print("Move back------------------------> b")
        print("Go up----------------------------> gu")
        print("Go down--------------------------> gd")
        print("Disarm Mode ---------------------> d \n")
	
        input_str = raw_input("Mode: ")
	
    	os.system('clear')

    	pub.publish(input_str)
	
    	rate.sleep()

if __name__ == '__main__':
    try:
        input_pub()
    except rospy.ROSInterruptException:
        pass
