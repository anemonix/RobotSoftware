import socket
import sys
from ctypes import *

#direction of robot
directionDegree = 0;

#Feather IP and port
UDP_IP = "192.168.1.146";
UDP_PORT = 8765;

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
sock.connect((UDP_IP, UDP_PORT));

class cmdPacket(Structure):
	_fields_ = [("theta", c_double)];

# Constantly check for inputs
#n for north, s for south, e for east, w for west
#Robot will turn appropiate degree for each of those
# based on IMU calibration
while True:
	input = raw_input("Which direction would you like to face? ");
	
	if (input == 'n'):
		directionDegree = 280;
		print("Facing North");
		sock.send(cmdPacket(directionDegree));
		
	elif (input == 's'):
		directionDegree = 80;
		print("Facing South");
		sock.send(cmdPacket(directionDegree));
		
	elif (input == 'w'):
		directionDegree = 180;
		print("Facing West");
		sock.send(cmdPacket(directionDegree));
		
	elif (input == 'e'):
		directionDegree = 356;
		print("Facing East");
		sock.send(cmdPacket(directionDegree));
	else:
		print("Please enter either 'n' 's' 'e' or 'w'");