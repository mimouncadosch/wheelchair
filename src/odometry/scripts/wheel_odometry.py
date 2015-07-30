#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import sys
import serial
import re
import threading;
import math
import tf

"""
This file reads the wheel revolutions from the Arduino on the wheelchair and computes wheel odometry.
Pass USB port as command line argument. Else, default value will be used.
"""

D = 0.34 # Wheel diameter, 34cm
b = 0.61 # Distance between two wheels

#revs_left = 0	# left wheel revolutions since last update
#revs_right = 0	# right wheel revolutions since last update

right_rev_count	= 0	# total number of revolutions right wheel
left_rev_count 	= 0	# total number of revolutions left wheel

last_right_rev_count 	= 0	# number of revolutions right wheel in last time step
last_left_rev_count 	= 0		# number of revolutions left wheel in last time step 

# Position variables
theta 	= 0
x	= 0 
y 	= 0 

def parse_args():
	if len(sys.argv) == 1:
		print "No USB port given, default will be used (/dev/cu.usbmodem1411)"
		port = "/dev/cu.usbmodem1411"
	elif len(sys.argv) == 2:
		print "USB port given: ", str(sys.argv[1])
		port = str(sys.argv[1])
	elif len(sys.argv) > 2:
		print "Too many arguments, default port will be used (/dev/cu.usbmodem1411)"
		port = "/dev/cu.usbmodem1411"

	return port

#port = rospy.get_param("/wheel_odometry_publisher/port", "/dev/ttyACM0")
#rospy.loginfo("Port is %s ", port)

port = parse_args()
print ("Port is %s ", port)

ser = serial.Serial(port, 9600)

def parse_vals(raw_vals):
	parsed_vals = re.findall("([A-Za-z]+):\s(-?[0-9]+)", raw_vals)[0]
	wheel = parsed_vals[0]
	stripes= int(parsed_vals[1]) # Number of black/white stripes detected by encoder
	return wheel, stripes

def update_robot_state(): 
	global last_right_rev_count
	global right_rev_count
	global last_left_rev_count 	
	global left_rev_count	
	global D
	global b
	global theta
	global x
	global y

	threading.Timer(0.5, update_robot_state).start()
	
	# Number of revolutions in dt (time step)
	d_right_revs = right_rev_count - last_right_rev_count 
	d_left_revs = left_rev_count - last_left_rev_count
	
	# Compute right and left wheel displacement d_sr and d_sl in time step
	d_sr = math.pi * D * d_right_revs 
	d_sl = math.pi * D * d_left_revs

	# Compute x and y position of wheelchair assuming differential drive	
	# Important: d_theta and theta are in radians
	d_s = (d_sr + d_sl) / 2.0
	d_theta = (d_sr - d_sl) / b 

	theta += d_theta
	# Convert theta and d_theta to degrees
	d_x = d_s * math.cos(math.degrees(theta) + (math.degrees(d_theta)/2.0))
	d_y = d_s * math.sin(math.degrees(theta) + (math.degrees(d_theta)/2.0))

	x += d_x
	y += d_y	

	# update system	
	last_right_rev_count = right_rev_count
	last_left_rev_count = left_rev_count	


def publish_state():
	global x
	global y 
	global theta 
	# Set up topic to be published
	pub = rospy.Publisher("wheel_odometry", PoseStamped, queue_size=100)
	rospy.init_node("wheel_odometry_publisher", anonymous=True)
	rate = rospy.Rate(2) # Units in Hz

	while not rospy.is_shutdown():
		# Constantly receive wheel odometry data from Arduino 
		raw_vals = ser.readline()
		wheel, stripes = parse_vals(raw_vals)
			
		if wheel ==  "right":
			right_rev_count = stripes/8.0

		if wheel ==  "left":
			left_rev_count = stripes/8.0

		# Prepare position message to be sent in topic
		position = PoseStamped()
		position.header.stamp = rospy.get_rostime()
		position.header.frame_id = "base_link"
		position.pose.position.x = x
		position.pose.position.y = y
		position.pose.position.z = 0    

		quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
		position.pose.orientation = quaternion
		
		pub.publish(position)
		print x,y
		rate.sleep()
	
	return True

def main():
	# Update robot state (x,y,theta), launches separate threads
	update_robot_state()
	# Publish robot state in ROS system
	publish_state()
	
	"""
	# Constantly receive wheel odometry data from Arduino 
	while True:
		raw_vals = ser.readline()
		wheel, stripes = parse_vals(raw_vals)
			
		if wheel ==  "right":
			right_rev_count = stripes/8.0

		if wheel ==  "left":
			left_rev_count = stripes/8.0
	"""


if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
		ser.close()
