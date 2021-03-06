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
Pass USB port as command line argument, or in launch file. Else, default value will be used.  
"""

# Global variables

# Encoder count (right, left)
right_ticks = 0
left_ticks = 0

# Encoder count in last time step (right, left)
last_right_ticks = 0 
last_left_ticks = 0

# Position variables
x 		= 0
y 		= 0 
theta 	= 0	# in radians


# Communication variables
serial_port_is_open = True
serial_communication_rate = 100 # in Hz

def main():
	# Serial port
	#port = rospy.get_param("/wheel_odometry_publisher/port", "/dev/ttyACM0")
	#rospy.loginfo("Port is %s ", port)
	port = parse_args()
	print ("Port is %s ", port)

	# First, create and start separate thread that reads values from the Arduino and updates the corresponding variables
	thread = Thread(target=read_serial_port, args=("Background-Thread", port, ))
	thread.daemon = True
	thread.start()
	
	# Compute and publish the updated position of the vehicle
	update_vehicle_position(thread)

"""
This thread reads the number of wheel ticks from the serial port for the right and left wheels. 
This number is computed by the sensors on board. 
The number of right and left wheel ticks is updated, and assigned to its corresponding global variables.

Updates odometry values 100 times a second
"""
def read_serial_port(threadname, port):
	global serial_port_is_open, right_ticks, left_ticks

	ignore_vars = 0

	# Open serial port
	try:	
		ser = serial.Serial(port, 9600)
	except:
		print "Failed to open serial connection"
		return False

	while serial_port_is_open:
		raw_vals = ser.readline()

		# Ignore the 15 first values, which tend to be noise
		if ignore_vars < 15:
			print "discarding data, ignore_vars: ", ignore_vars
			ignore_vars += 1
			continue

		# Ensure the string received from the serial port is properly formatted
		if check_raw_vals(raw_vals) is False:
			continue

		# Parse the values as received from the serial port
		right_ticks, left_ticks = parse_vals(raw_vals)
		#print right_ticks, left_ticks

		if serial_port_is_open is False:
			print "Closing serial port"
			break

"""
This function computes the vehicle position from odometry data
"""
def update_vehicle_position(thread):
	global serial_port_is_open, x, y, theta

	# Create ROS node and topic
	pub = rospy.Publisher("wheel_odometry", geometry_msgs.msg.PoseStamped, queue_size=100)
	rospy.init_node("wheel_odometry_publisher", anonymous=True)
	rate = rospy.Rate(5) # Units in Hz
	
	# Wheelchair parameter values
	b = 0.61 # Distance between two wheels, 61cm
	D = 0.34 # Wheel diameter, 34cm
	counts_per_rev = 514

	wheel_circumference = math.pi * D
	distance_per_tick = wheel_circumference / counts_per_rev 

	try:
		# This loop will run every (1/rate) seconds
		while not rospy.is_shutdown():
			#rospy.loginfo("computing position from odometry values")
			compute_position_from_odometry(distance_per_tick)

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
		print "Keyboard Interrupt"
		serial_port_is_open = False
		thread.join()
		pass

	return True

def compute_position_from_odometry(distance_per_tick):
	global right_ticks, left_ticks, last_right_ticks, last_left_ticks, x,y, theta
	
	b = 0.61 # Distance between two wheels, 61cm

	# Number of revolutions in dt (time step)
	d_right_ticks = right_ticks - last_right_ticks 
	d_left_ticks  = left_ticks - last_left_ticks 

	print right_ticks, left_ticks 

	# Compute right and left wheel displacement d_sr and d_sl in time step
	d_sr = distance_per_tick * d_right_ticks 
	d_sl = distance_per_tick * d_left_ticks 

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
	# Convert theta and d_theta to degrees
	d_x = d_s * math.cos(math.degrees(theta) + (math.degrees(d_theta)/2.0))
	d_y = d_s * math.sin(math.degrees(theta) + (math.degrees(d_theta)/2.0))
	"""

	x += d_x
	y += d_y	

	print "Vehicle position (theta in deg): ", x, y, math.degrees(theta)
	# update system	
	last_right_ticks = right_ticks
	last_left_ticks = left_ticks

	return True

def parse_args():
	default_port = "/dev/ttyACM0"
	if len(sys.argv) == 1:
		print "No USB port given, default will be used (" , default_port, ")"
		port = default_port
	elif len(sys.argv) == 2:
		print "USB port given: ", str(sys.argv[1])
		port = str(sys.argv[1])
	elif len(sys.argv) > 2:
		print "Too many arguments, default port will be used (" , default_port, ")"
		port = default_port

	return port

"""
This function ensures that the raw strings received from the serial port are formatted correctly. 
The format is "right: (- if going backward)(number of right ticks) \t left: (- if going backward)(number of left ticks)".
If these raw strings are not properly formatted, the relevant values will not be properly parsed. 
"""
def check_raw_vals(raw_vals):
	parsed_vals = re.findall("(right):\s(-?[0-9]+)\t(left):\s(-?[0-9]+)", raw_vals)
	# If nothing found
	if not parsed_vals: 
		return False
	# If wrong number of arguments found
	elif len(parsed_vals[0]) != 2:
		return False
	# If correct number of arguments found
	elif len(parsed_vals[0]) == 2:
		return True
	
def parse_vals(raw_vals):
	global last_right_ticks, last_left_ticks
	parsed_vals = re.findall("(right|left):\s(-?[0-9]+)", raw_vals)[0]
	if parsed_vals[0] == "left":
		left_ticks = int(parsed_vals[1])
		right_ticks = last_right_ticks
	elif parsed_vals[0] == "right":
		right_ticks = int(parsed_vals[1])
		left_ticks = last_left_ticks

	return right_ticks, left_ticks 

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		print "Stopping ROS"
		pass
