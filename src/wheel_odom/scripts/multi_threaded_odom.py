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

""" This file reads the wheel revolutions from the Arduino on the wheelchair and computes wheel odometry. The pose, as calculated from the wheel odometry is published over ROS.
Pass USB port as command line argument. Else, default value will be used.  """

# Global variables
# Number of wheel revolutions (right, left)
right_revs = 0
left_revs  = 0

# Number of wheel revolutions in last time step (right, left)
last_right_revs = 0
last_left_revs  = 0

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
This thread reads wheel revolutions from the serial port, and updates the corresponding global variables over ROS.

Updates odometry values 100 times a second
"""
def update_odometry_values(threadname, port):
	global right_revs
	global left_revs
	ignore_vars = 0

	try:	
		ser = serial.Serial(port, 9600)
	except:
		print "Failed to open serial connection"
		return False

	serial_sleep_time =  (1.0 / serial_communication_rate) # T = 1/f

	while True:
		raw_vals = ser.readline()
		if ignore_vars < 30:
			print "discarding data, ignore_vars: ", ignore_vars
			ignore_vars += 1
			continue

		if check_raw_vals(raw_vals) is False:
			continue

		stripes_right, stripes_left = parse_vals(raw_vals)
		right_revs = stripes_right/8.0
		left_revs = stripes_left/8.0

		time.sleep(serial_sleep_time)

		if stop_serial_communication == True:
			print "Breaking Serial Communication with the Arduino."
			break


"""
This function computes the vehicle position from odometry data
"""
def update_vehicle_position(thread):

	# Create ROS node and topic
	pub = rospy.Publisher("wheel_odometry", geometry_msgs.msg.PoseStamped, queue_size=100)
	rospy.init_node("wheel_odometry_publisher", anonymous=True)
	rate = rospy.Rate(2) # Units in Hz

	try:
		# This loop will run every (1/rate) seconds
		while not rospy.is_shutdown():
			compute_position_from_odometry()
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

	
def compute_position_from_odometry():
	global right_revs
	global left_revs
	global last_right_revs
	global last_left_revs
	global D
	global b
	global x
	global y
	global theta

	# Number of revolutions in dt (time step)
	d_right_revs = right_revs - last_right_revs
	d_left_revs = left_revs - last_left_revs

	#print right_revs, left_revs	
	#print "d_right_revs: ", d_right_revs, "d_left_revs: ", d_left_revs

	# Compute right and left wheel displacement d_sr and d_sl in time step
	d_sr = math.pi * D * d_right_revs 
	d_sl = math.pi * D * d_left_revs

	# Compute x and y position of wheelchair assuming differential drive	
	# Important: d_theta and theta are in radians
	d_s = (d_sr + d_sl) / 2.0

	d_theta = (d_sr - d_sl) / b
	#print "RR: ", right_revs, " LR: ", left_revs
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
	last_right_revs = right_revs
	last_left_revs = left_revs

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
