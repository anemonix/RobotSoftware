from pynput.keyboard import Key, Listener

mode = 0;
velocity = 0;
theta = 0;

class controlStruct():
    def __init__(velocity, theta, mode):
        self.velocity = velocity
        self.theta = theta
        self.mode = mode

def on_press(key):
	print('{0} pressed'.format(key))

def on_release(key):
	print(key)
	if key == 'w':
		print('Increasing speed')
	if key == 's':
		print('Decreasing speed')
	if key == 'u'a'':
		print('Moving left')
	if key == Key.right:
		print('Moving right')
	if(key == Key.esc):
		return False

with Listener( on_press=on_press, on_release=on_release) as listener:
	listener.join()
