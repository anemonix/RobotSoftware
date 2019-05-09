import socket
import sys
from ctypes import *
from getkey import getkey, keys

vel = 0
phi = 0

UDP_IP = "192.168.43.167"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.connect((UDP_IP, UDP_PORT))

class cmdPacket(Structure):
	_fields_ = [("velocity", c_double),("theta", c_double),("mode", c_int)]
	
class rtnPacket(Structure):
	_fields_ = [("x", c_double),("y", c_double),("head", c_double)]

while True:
	key = getkey()
	
	if (key == keys.UP):
		print("Increasing speed");
		vel = vel + 10;		
		sock.send(cmdPacket(vel, phi, 1));
		
	elif (key == keys.DOWN):
		print("Decreasing speed");
		vel = vel - 10;
		if (vel < 0):
			vel = 0;
		sock.send(cmdPacket(vel, phi, 1));
		
	elif (key == keys.LEFT):
		print("Turning left");
		phi = phi - 15;
		sock.send(cmdPacket(vel, phi, 1));
		
	elif (key == keys.RIGHT):
		print("Turning right");
		phi = phi + 15;
		sock.send( cmdPacket(vel, phi, 1));
		
	elif (key == 'w'):
		print("Face North")
		vel = 0;
		phi = 265;
		sock.send(cmdPacket(vel, phi, 2));
		
	elif (key == 's'):
		print("Face South");
		vel = 0;
		phi = 90;
		sock.send(cmdPacket(vel, phi, 2));
		
	elif (key == 'a'):
		print("Face West");
		vel = 0;
		phi = 180;
		sock.send(cmdPacket(vel, phi, 2));
		
	elif (key == 'd'):
		print("Face East");
		vel = 0;
		phi = 353;
		sock.send(cmdPacket(vel, phi, 2));
		
	elif (key == ' '):
		print("Stop");
		vel = 0;
		sock.send(cmdPacket(vel, phi, 1));
			
	elif ('q' == key):
		print("Send Report");
		
		sock.send(cmdPacket(vel, phi, 0));
		buffer = sock.recv(sizeof(rtnPacket));
		
		newRtnPacket = rtnPacket.from_buffer_copy(buffer);
		for field_name, field_type in newRtnPacket._fields_:
			print field_name, getattr(newRtnPacket, field_name);

	elif ('r' == key):
		print("Reset")
		vel = 0;
		sock.send(cmdPacket(vel, phi, 4));
		
	else:
		print("Command doesn't exist");