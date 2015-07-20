import serial

def main(argv):


ser = serial.Serial(


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
