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
		self.move_actions = [0.05,0.1,0.15]
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
		threshold = 0.01 #1cm
		distance_to_obstacle = self.read_laser()
		actions = []
		if self.cntr%2 ==0:
			actions = [[a, 0, 1] for a in self.move_actions if a < distance_to_obstacle - threshold]#[uv,uw,dt]
		if self.cntr%2 ==1 or len(actions) == 0:
			actions.append([0,30,3])#rotation 90=3*30
			actions.append([0,-30,3])#rotation 90=3*30
			actions.append([0,60,3])#rotation 90=3*30
		self.cntr = self.cntr + 1
		ind = np.random.choice(len(actions))
		return actions[ind]		
		amax = [a for a in self.move_actions if a < distance_to_obstacle - threshold]
		actions = []		
		if len(amax)>0:		
			actions = [[np.max(amax), 0, 1]]#[uv,uw,dt]
		actions.append([0,30,3])#rotation 90=3*30
		actions.append([0,-30,3])#rotation 90=3*30
		#print(actions)
		ind = np.random.choice(len(actions))
		
		return actions[ind]
	def move(self, uv, uw, dt=1):
		'''update state of particle'''
		global velocity_publisher
		uw = math.radians(uw)*2
		vel_msg = Twist()
		vel_msg.linear.x = uv
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = uw
		#rate = rospy.Rate(1)	
		t0 = rospy.Time.now().to_sec()
		while rospy.Time.now().to_sec()-t0<dt:
			velocity_publisher.publish(vel_msg)
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		velocity_publisher.publish(vel_msg)
		return 


def laser_callback(msg):
	'''if the new laser is needed update it and set the flag'''
	global laser_data, get_laser
	if get_laser:
		laser_data = msg.range
		get_laser = False
	#print(msg.range)

def main_fun():
	global velocity_publisher, x, y, theta, rects, lines
	rospy.init_node("controller")
	#define subscriber for laser and publisher to publish robot movement commands
	laser_sub = rospy.Subscriber('/vector/laser', Range, callback = laser_callback)
	velocity_publisher = rospy.Publisher("/vector/cmd_vel", Twist, queue_size=10)
	#read .world rectangles
	rects = np.load('rects_sample1.npy')
	margin = np.load('margin_sample1.npy')
	#get map info: width, height, ...
	mat, height, width, minx, miny, maxx, maxy, obstacles = Utility().map_info(rects)
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
		plt.plot(rect[1], rect[0], c='black')
	#reverece to visulaize better map		
	plt.ylim(minx,maxx)
	plt.show()
	#define particle filters
	particle_count = 1000
	PF = ParticleFilter(rob, particle_count, obstacles, height, width,minx+margin[0], miny+margin[1], maxx+margin[0], maxy+margin[1], 0.01, lines, rects, margin, sigma = 0.0097, epsilon = 0.01)
	iterations = 30
	for i in range(iterations):
		##place a particle on the robot init position uncomment to test robot and particle movement
		#uv, uw, dt = rob.get_action()
		#rob.move(uv, uw, dt)
		#p.move(uv, uw, dt)
		##uncomment to test particle lasers
		##print(p.read_laser(lines),rob.read_laser())
		# one loop of particle filtering algorithm		
		PF.main_loop()
		#plot map obstacles		
		for rect in lines2:
			rect = list(zip(*rect))
			plt.plot(rect[1], rect[0], c='black')
		#plot particles
		xarray = []
		yarray = []		
		for p in PF.particles:
			xarray.append(p.x)
			yarray.append(p.y)
			if PF.check(p.x, p.y):
				plt.arrow(p.y+ np.random.normal(0, 0.005), p.x+ np.random.normal(0, 0.005), 0.00001*math.sin(math.radians(p.theta)), \
                        0.00001*math.cos(math.radians(p.theta)), head_width = 0.02, fill=False, overhang = 0.6)
		plt.ylim(maxx,minx)
		plt.show()
	
if __name__ == "__main__":
	main_fun()
