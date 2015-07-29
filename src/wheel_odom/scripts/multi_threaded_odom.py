#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import String
import geometry_msgs.msg 
from threading import Thread
import serial
import re
import math
import time
import signal 
import sys

""" 
This file reads the wheel revolutions from the Arduino on the wheelchair and computes wheel odometry. The pose, as calculated from the wheel odometry is published over ROS.
Pass USB port as command line argument. Else, default value will be used.  
"""

# Global variables

# Encoder count (right, left)
encoder_count_right = 0
encoder_count_left  = 0

# Encoder count in last time step (right, left)
last_encoder_count_right = 0
last_encoder_count_left  = 0

"""
# Number of wheel revolutions (right, left)
right_revs = 0
left_revs  = 0
# Number of wheel revolutions in last time step (right, left)
last_right_revs = 0
last_left_revs  = 0
"""

# Position variables
x 		= 0
y 		= 0 
theta 	= 0	# in radians

# Wheelchair parameter values
D = 0.34 # Wheel diameter, 34cm
b = 0.61 # Distance between two wheels, 61cm

# Communication variables
serial_communication_rate = 100 # in Hz
stop_serial_communication = False

def main():
	#port = rospy.get_param("/wheel_odometry_publisher/port", "/dev/ttyACM0")
	#rospy.loginfo("Port is %s ", port)
	port = parse_args()
	print ("Port is %s ", port)

	# First, create and start separate thread that reads values from the Arduino and updates the corresponding variables
	thread = Thread(target=update_odometry_values, args=("Background-Thread", port, ))
	thread.daemon = True
	thread.start()
	
	# Compute and publish the updated position of the vehicle
	update_vehicle_position(thread)

"""
This thread reads wheel revolutions from the serial port, and updates the corresponding global variables.

Updates odometry values 100 times a second
"""
def update_odometry_values(threadname, port):
	global encoder_count_right
	global encoder_count_left
	ignore_vars = 0

	try:	
		ser = serial.Serial(port, 9600)
	except:
		print "Failed to open serial connection"
		return False

	serial_sleep_time =  (1.0 / serial_communication_rate) # T = 1/f

	while True:
		raw_vals = ser.readline()
		if ignore_vars < 15:
			print "discarding data, ignore_vars: ", ignore_vars
			ignore_vars += 1
			continue

		if check_raw_vals(raw_vals) is False:
			continue

		# Parse the values as received from the serial port
		encoder_count_right, encoder_count_left = parse_vals(raw_vals)
		#print encoder_count_right, encoder_count_left
		time.sleep(serial_sleep_time)

		if stop_serial_communication == True:
			print "Breaking Serial Communication with the Arduino."
			break


"""
This function computes the vehicle position from odometry data
"""
def update_vehicle_position(thread):
	global D

	# Create ROS node and topic
	pub = rospy.Publisher("wheel_odometry", geometry_msgs.msg.PoseStamped, queue_size=100)
	rospy.init_node("wheel_odometry_publisher", anonymous=True)
	rate = rospy.Rate(1) # Units in Hz
	
	wheel_circumference = math.pi * D
	counts_per_rev = 514
	distance_per_encoder_count = wheel_circumference / counts_per_rev 

	try:
		# This loop will run every (1/rate) seconds
		while not rospy.is_shutdown():
			compute_position_from_odometry(distance_per_encoder_count)
		 	#print "VEHICLE POSITION (theta in deg): ", x, y, math.degrees(theta)

			# Prepare position message to be sent in topic
			position = geometry_msgs.msg.PoseStamped()
			position.header.stamp = rospy.get_rostime()
			position.header.frame_id = "base_link"
			position.pose.position.x = x
			position.pose.position.y = y
			position.pose.position.z = 0    

			quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
			position.pose.orientation.x = quaternion[0]
			position.pose.orientation.y = quaternion[1]
			position.pose.orientation.z = quaternion[2]
			position.pose.orientation.w = quaternion[3]

			pub.publish(position)

			#print quaternion
			# Publish transformations
			odom_broadcaster = tf.TransformBroadcaster()
			odom_broadcaster.sendTransform((x,y,0), (quaternion[0],quaternion[1],quaternion[2],quaternion[3]), rospy.get_rostime(), "base_link", "vehicle_link")

			rate.sleep()
	except KeyboardInterrupt:
		print "Stopping Serial Communication"
		stop_serial_communication = True
		thread.join()

	
def compute_position_from_odometry(distance_per_encoder_count):
	global encoder_count_right
	global encoder_count_left
	global last_encoder_count_right
	global last_encoder_count_left
	global b
	global x
	global y
	global theta

	# Number of revolutions in dt (time step)
	d_encoder_count_right = encoder_count_right - last_encoder_count_right
	d_encoder_count_left  = encoder_count_left - last_encoder_count_left
	#print ">>> ", distance_per_encoder_count
	#print d_encoder_count_right, d_encoder_count_left

	# Compute right and left wheel displacement d_sr and d_sl in time step
	d_sr = distance_per_encoder_count * d_encoder_count_right 
	d_sl = distance_per_encoder_count * d_encoder_count_left

	# Compute x and y position of wheelchair assuming differential drive	
	# Important: d_theta and theta are in radians
	d_s = (d_sr + d_sl) / 2.0

	d_theta = (d_sr - d_sl) / b
	#print "d_sr: ", d_sr, " d_sl: ", d_sl
	#print "d_theta :", math.degrees(d_theta)

	theta += d_theta

	# math.cos and math.sin take radians!!!
	d_x = d_s * math.cos(theta + (d_theta/2.0))
	d_y = d_s * math.sin(theta + (d_theta/2.0))

	"""
	theta += d_theta

	# Convert theta and d_theta to degrees
	d_x = d_s * math.cos(math.degrees(theta) + (math.degrees(d_theta)/2.0))
	d_y = d_s * math.sin(math.degrees(theta) + (math.degrees(d_theta)/2.0))
	"""

	x += d_x
	y += d_y	

	# update system	
	last_encoder_count_right = encoder_count_right
	last_encoder_count_left  = encoder_count_left

	return True

def parse_args():
	default_port = "/dev/ttyACM0"
	if len(sys.argv) == 1:
		print "No USB port given, default will be used (/dev/cu.usbmodem1411)"
		port = default_port
	elif len(sys.argv) == 2:
		print "USB port given: ", str(sys.argv[1])
		port = str(sys.argv[1])
	elif len(sys.argv) > 2:
		print "Too many arguments, default port will be used (/dev/cu.usbmodem1411)"
		port = default_port

	return port

def check_raw_vals(raw_vals):
	parsed_vals = re.findall("(right):\s(-?[0-9]+)\t\s(left):\s(-?[0-9]+)", raw_vals)
	# If nothing found
	if not parsed_vals: 
		return False
	# If wrong number of arguments found
	elif len(parsed_vals[0]) != 4:
		return False
	# If correct number of arguments found
	elif len(parsed_vals[0]) == 4:
		return True
	
def parse_vals(raw_vals):
	parsed_vals = re.findall("(right):\s(-?[0-9]+)\t\s(left):\s(-?[0-9]+)", raw_vals)[0]
	stripes_right = int(parsed_vals[1])
	stripes_left = int(parsed_vals[3])
	return stripes_right, stripes_left

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print "Stopping ROS"
		pass
