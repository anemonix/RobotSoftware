import socket
import sys
from ctypes import *

#direction of robot
distance = 0;

#Feather IP and port
UDP_IP = "192.168.1.146";
UDP_PORT = 8765;

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
sock.connect((UDP_IP, UDP_PORT));

class cmdPacket(Structure):
	_fields_ = [("distance", c_double)];

# Constantly check for inputs
# looking for a distance to travel forward
while True:
	input = raw_input("What distance would you like to travel? ");
	print("Going forward");
	sock.send(cmdPacket(distance));
		