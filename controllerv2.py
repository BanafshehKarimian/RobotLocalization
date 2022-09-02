#! /usr/bin/env python3
 
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
import shapely
from shapely.geometry import LineString,Point, Polygon
from std_msgs.msg import String
import math
import numpy as np
from utility import *
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Proximity
from anki_vector_ros.msg import Drive
from time import sleep
import matplotlib.pyplot as plt
import scipy
import scipy.stats
from particle_filter import *
get_laser = False
laser_data = -1
rects = []
lines = []
class Robot:
	def __init__(self):
		self.move_actions = [5,10,15]  #cm
		self.cntr = 0
		pass
	def read_laser(self):
		'''request the new laser and wait untill it updates'''
		global laser_data, get_laser
		get_laser = True
		while get_laser:
			pass
		return laser_data
	def get_action(self):
		'''check if the movements are possible depending the laser data and remove impossible movements and add rotation'''
		if self.cntr%4 ==0:
			threshold = 1 #1cm
			distance_to_obstacle = self.read_laser()*100
			actions = [[a, 0, 1] for a in self.move_actions if a < distance_to_obstacle - threshold]#[uv,uw,dt]
			actions.append([0,30,3])#rotation 90=3*30
			ind = np.random.choice(len(actions))
			return actions[ind]
		self.cntr = self.cntr + 1
		return [0,30,3]
	def move(self, uv, uw, dt=1):
		'''update state of particle'''
		global velocity_publisher
		if uw ==0:
			velocity_publisher.publish(100.0, 100.0, 0.0, 0.0)
			sleep(uv/10)
		else:
			velocity_publisher.publish(-100.0, 100.0, 0.0, 0.0)
			sleep(0.8)
		velocity_publisher.publish(0.0, 0.0, 0.0, 0.0)
		return 


def laser_callback(msg):
	'''if the new laser is needed update it and set the flag'''
	global laser_data, get_laser
	if True:#get_laser:
		laser_data = msg.distance/1000
		get_laser = False
	#print(msg.distance/1000)

def main_fun():
	global velocity_publisher, x, y, theta, rects, lines
	rospy.init_node("controller")
	#define subscriber for laser and publisher to publish robot movement commands
	laser_sub = rospy.Subscriber('/proximity', Proximity, callback = laser_callback)
	velocity_publisher = rospy.Publisher("/motors/wheels", Drive, queue_size=1)
	map_publisher = rospy.Publisher("/mapper", String, queue_size=10)
	particle_publisher = rospy.Publisher("/visualizer", String, queue_size=10)
	#create map obstacles
	wall = 16 #mm
	p1 = [0,0]
	p2 = [wall, 0]
	p3 = [2*wall+483+235,0]
	p4 = [p3[0]+wall,0]
	p5 = [wall, wall]
	p6 = [wall + 483, wall]
	p7 = [p6[0]+wall, wall]
	p8 = [p7[0]+235, wall]
	p9 = [p8[0]+wall,wall]
	p10 = [wall, wall + 245]
	p11 = [wall + 295, wall + 245]
	p12 = [wall + 295, wall + 245 - 90]
	p13 = [p12[0] + wall, p12[1]]
	p14 = [wall + 483, wall + 220]
	p15 = [p14[0] + wall, p14[1]]
	p16 = [p15[0] + 85, p15[1]]
	p17 = [p16[0], p16[1] + wall]
	p18 = [p14[0], p14[1] + wall]
	p19 = [p13[0], p13[1] + 200]
	p20 = [p12[0], p12[1] + 200]
	p21 = [p11[0], p11[1] + wall]
	p22 = [p10[0], p10[1] + wall]
	p23 = [wall, 2* wall + 245 + 237]
	p24 = [wall + 300, p23[1]]
	p25 = [p24[0], p24[1] + wall]
	p26 = [p23[0], p23[1] + wall]
	p27 = [wall, 3* wall + 245 + 237+225]
	p28 = [wall, p27[1] + wall]
	p29 = [0, p27[1] + wall]
	p30 = [p27[0] + 467, p27[1]]
	p31 = [p30[0] + wall, p30[1]]
	p32 = [p31[0] + 255, p31[1]]
	p33 = [p32[0], p32[1] + wall]
	p34 = [p33[0] + wall, p33[1]]
	p35 = [p31[0], p31[1] - 220]
	p36 = [p35[0] + 110 - wall - 45, p35[1]]
	p37 = [p36[0], p36[1] - wall]
	p38 = [p37[0] - 110, p37[1]]
	p39 = [p38[0], p38[1] + wall]
	p40 = [p39[0] + 45, p39[1]]
	p41 = [wall + 300, wall*2 + 245 + 237]
	p42 = [wall + 300, wall*3 + 245 + 237]
	p43 = [wall + 300 + 220, wall*3 + 245 + 237]
	p44 = [wall + 300 + 220, wall*2 + 245 + 237 - 5 - 40]
	p45 = [p44[0] + wall, p44[1]]
	p46 = [p44[0], p44[1] + 110]
	p47 = [p46[0] + wall, p46[1]]
	p48 = [wall + 300 + 220, wall*2 + 245 + 237]	
	rects = [[p1,p29,p2,p28], [p2,p5,p3,p8],[p3,p33,p4,p34],[p27,p28,p32,p33], [p10,p22,p11,p21], [p12,p20,p13,p19], [p6,p14,p7,p15],[p14,p18,p16,p17],[p23,p26,p24,p25], [p48,p41,p43,p42], [p44, p46, p45, p47]]#, [p40,p30,p35,p31], [p38, p39, p37, p36]
	
	rects = np.array(rects)/1000	
	#print(rects)	
	#get map info: width, height, ...
	mat, height, width, minx, miny, maxx, maxy, obstacles = Utility().map_info(rects)
	margin = np.load('margin_sample1.npy')
	lines = []
	lines2 = []
	#get lines of the map from rectangles
	for points in rects:
		lines.append(LineString([ np.array(points[0])+margin , np.array(points[1])+margin]) )
		lines.append(LineString([ np.array(points[1])+margin , np.array(points[3])+margin]) )
		lines.append(LineString([ np.array(points[3])+margin , np.array(points[2])+margin]) )
		lines.append(LineString([ np.array(points[2])+margin , np.array(points[0])+margin]) )
		lines2.append([ np.array(points[0])+margin , np.array(points[1])+margin])
		lines2.append([ np.array(points[1])+margin , np.array(points[3])+margin] )
		lines2.append([ np.array(points[3])+margin , np.array(points[2])+margin] )
		lines2.append([ np.array(points[2])+margin , np.array(points[0])+margin] )
	rob = Robot()
	#plot map
	for rect in lines2:
		rect = list(zip(*rect))
		plt.plot(rect[0], rect[1], c='black')
	plt.show()
	particle_count = 1000
	#define particle filters
	PF = ParticleFilter(rob, particle_count, obstacles, height, width,minx+margin[0], miny+margin[1], maxx+margin[0], maxy+margin[1], 0.01, lines, rects, margin, sigma = 0.0097, epsilon = 0.01)
	#p = Particle(0.5,0.9,0)
	#actions = [[5, 0, 1],[10, 0, 1],[0,30,3],[15, 0, 1]]	
	for i in range(30):
		#uv, uw, dt = actions[i]#rob.get_action()
		#rob.move(uv, uw, dt)
		#p.move(uv, uw, dt)
		#print(uv,uw,dt)
		#print(p.read_laser(lines))
		#print("iteration"+str(i))
		# one loop of particle filtering algorithm		
		PF.main_loop()
		#plot map obstacles		
		for rect in lines2:
			rect = list(zip(*rect))
			plt.plot(rect[0], rect[1], c='black')
		#plot particles
		xarray = []
		yarray = []		
		for p in PF.particles:
			xarray.append(p.x)
			yarray.append(p.y)
			if PF.check(p.x, p.y):
				plt.arrow(p.x+ np.random.normal(0, 0.005), p.y+ np.random.normal(0, 0.005), 0.00001*math.cos(math.radians(p.theta)), 0.00001*math.sin(math.radians(p.theta)), head_width = 0.02, fill=False, overhang = 0.6)
		plt.show()
	
if __name__ == "__main__":
	main_fun()
