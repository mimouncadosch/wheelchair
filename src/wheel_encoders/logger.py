"""
M. Delmar
7/2015
Logger for wheel encoders
"""

import sys
import re
import time
import serial
from threading import Thread

# Global variables
right_ticks = 0
seconds_passed = 0
run_timer = True
port = ""
last_right_ticks = 0

def main():
	global right_ticks, port

	port = parse_args()
	print ("Port is %s ", port)	

	# Thread for time keeping
	time_keeping_thread = Thread(target=keep_time, args=("Time-Keeping-Thread", ))
	time_keeping_thread.daemon = False
	
	time_keeping_thread.start()
	count_ticks(time_keeping_thread)
	

def keep_time(threadname):
	global seconds_passed, run_timer, last_right_ticks, right_ticks
	print "run_timer: ", run_timer
	print "Calling keep_time, threadname: ", threadname, "\n"

	# Logging
	# filename = "./logs.csv"
	# print "Opening the file..."
	# target = open(filename, 'w')	
	# target.write("seconds_passed: , new_right_ticks: ")
	# target.write("\n")

	while run_timer is True:
		seconds_passed += 1

		# Count number of new ticks
		new_right_ticks = int(right_ticks) - int(last_right_ticks)			
		last_right_ticks = right_ticks
		print "seconds_passed", seconds_passed
		print "new_right_ticks", new_right_ticks
		time.sleep(0.1)
		# Writing to file
		# target.write(str(seconds_passed))
		# target.write(", ")
		# target.write(str(new_right_ticks))
		# target.write("\n")

		if run_timer is False:
			print "Closing log file"
			# target.close()
	return True


"""
Read the stream from the serial port and count the number of ticks
"""
def count_ticks(thread):
	global right_ticks, run_timer, port, last_right_ticks
	# Read from serial port
	ser = serial.Serial(port, 9600)

	try:
		while True:
			# Read number of total ticks from serial port
			raw_vals = ser.readline()	
			right_ticks = parse_vals(raw_vals)
			
	except KeyboardInterrupt:
		print "Keyboard Interrupt"
		run_timer = False
		thread.join()
		pass
	
	return True
	
"""
Parse values read from serial port
"""
def parse_vals(raw_vals):
	parsed_vals = re.findall("n:\s([0-9]+)", raw_vals)
	if len(parsed_vals) > 0:
		return parsed_vals[0]
	elif len(parsed_vals) == 0:
		return parsed_vals

	# stripes_right = int(parsed_vals[1])
	# stripes_left = int(parsed_vals[3])
	# return stripes_right, stripes_left
	# return parsed_vals[0]

"""
Parse arguments for serial port
"""
def parse_args():
	default_port = "/dev/cu.usbmodem1411"
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


if __name__ == '__main__':
	main()
	
