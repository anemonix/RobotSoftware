import socket
import sys
from ctypes import *

#direction of robot
velocity = 0;

#Feather IP and port
UDP_IP = "192.168.1.146";
UDP_PORT = 8765;

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
sock.connect((UDP_IP, UDP_PORT));

class cmdPacket(Structure):
	_fields_ = [("velocity", c_double)];

# Constantly check for inputs
#see when robot will start running
while True:
	input = raw_input("Which direction would you like to face? ");
	
	if (input == 'f'):
		velocity += 20;
		print("Going forward");
		sock.send(cmdPacket(velocity));
	else:
		print("Please enter 'f' when ready");