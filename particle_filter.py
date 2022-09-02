import random
import math
from utility import *
import scipy
import scipy.stats
import bisect
import matplotlib.pyplot as plt
import numpy as np
simulator = True
class Particle:
	def __init__(self, x, y, theta, w = 1):
		self.x = x
		self.y = y
		self.w = w
		self.theta = theta
		self.flag = True
	def read_laser(self, lines):
		'''get map occupancy matrix and return the laser data'''
		xp = self.x + 0.4*np.cos(math.radians(self.theta))        #x(t+1)=x(t)+uv(t)cos(tetha(t))
		yp = self.y+0.4*np.sin(math.radians(self.theta))          #y(t+1)=y(t)+uv(t)sin(tetha(t))
		line = LineString([tuple([self.x,self.y]), tuple([xp,yp])])
		return Utility().laser(line, lines, self.x,self.y)+ np.random.normal(0.0, 0.0005)
	def move(self, uv, uw, dt):
		'''update state of particle'''
		#if simulator:
		#	self.x = self.x + dt * uv * np.cos(math.radians(self.theta)) + np.random.normal(0.000125, 0.0007) + np.random.normal(0, 0.0027)#+noise
		#	self.y = self.y + dt * uv * np.sin(math.radians(self.theta)) + np.random.normal(0.000125, 0.0007) + np.random.normal(0, 0.0027)#noise
		#	self.theta = self.theta + dt * uw + np.random.normal(0.0001, 0.0015) + np.random.normal(0, 0.0005)# + noise
		#else:
		dt = uv/10
		if uw == 0:
			uv = 0.1#cm
		else:
			if not simulator:
				uw = 90/0.8
				dt = 0.8
				uv = 0
		self.x = self.x + dt * uv * np.cos(math.radians(self.theta)) + np.random.normal(0.000125, 0.0007) + np.random.normal(0, 0.0027)#+noise
		self.y = self.y + dt * uv * np.sin(math.radians(self.theta)) + np.random.normal(0.000125, 0.0007) + np.random.normal(0, 0.0027)#noise
		self.theta = self.theta + dt * uw + np.random.normal(0.0001, 0.0015) + np.random.normal(0, 0.0005)# + noise
		return
class ParticleFilter:
	def __init__(self, robot, particle_count, obstacles, height, width, minx, miny, maxx, maxy, radius, lines, rects, margin, sigma = 0.0097, epsilon = 0.1):
		self.particle_count = particle_count
		self.obstacles = obstacles
		self.map = Utility().create_occupancy_matrix(obstacles, height, width)
		self.robot = robot
		self.sigma = sigma
		self.height = height
		self.width = width
		self.radius = radius
		self.epsilon = epsilon
		self.lines = lines
		self.minx = minx
		self.miny = miny
		self.maxx = maxx
		self.maxy = maxy
		self.rects = rects
		self.margin = margin
		self.wslow = 0
		self.wfast = 0
		self.wavg = 0
		self.particles = self.init_particles()
	def main_loop(self):
		print("moving robot and particles")
		self.move_robot_and_particles()
		print("reading laser data")
		Zi = self.robot.read_laser()
		mu = self.get_particle_sensor()
		print("calculate weights")
		self.calculate_weights(mu, Zi)
		print("resampling")
		self.resample_particles()
	def init_particles(self):
		particles = []
		for i in range(self.particle_count):
			x, y, theta = self.random_free_place()
			particles.append(Particle(x, y, theta))
		return particles
	def check(self, x, y):
		if x < self.minx or x >self.maxx or y < self.miny or y > self.maxy:
			return False
		if colision(self.radius, x,y , self.rects, self.margin):
			return False
		return True	
	def random_free_place(self):
		while True:
			x = random.uniform(self.minx, self.maxx)
			y = random.uniform(self.miny, self.maxy)
			theta = np.random.choice([0,90,180,270])#random.uniform(0, 360)
			if self.check(x, y):
				return x, y, theta
	def get_particle_sensor(self):
		return [p.read_laser(self.lines) for p in self.particles]
	def calculate_weights(self, mu, Zi):
		for p in range(self.particle_count):
			self.particles[p].w = 0
			if self.particles[p].flag:
				self.particles[p].w = scipy.stats.norm(mu[p], self.sigma).pdf(Zi) + 1e-8
		nu = sum(p.w for p in self.particles)
		if nu:
			for p in self.particles:
				p.w = p.w / nu
		if True:#self.wavg == 0:
			self.wavg = np.mean([p.w for p in self.particles if p.w > 0])
		else:
			self.wavg = np.mean([self.wavg, np.mean([p.w for p in self.particles if p.w > 0])])
	'''def sampling(self,weights, n):
		sampleW = random.uniform(0, 1)
		otherSamples = np.array([(sampleW+i/n)%1 for i in range(n)])
		weightsSum = np.cumsum(weights)
		sampleIndex = [np.where(s<weightsSum)[0][0] for s in otherSamples]
		return sampleIndex'''	
	def resample_particles(self):
		particles = []
		self.wslow = self.wslow + 0.3*(self.wavg-self.wslow)
		self.wfast = self.wfast + 0.7*(self.wavg - self.wfast)
		self.particles = np.array(self.particles)
		weights = np.array([p.w+1e-5 for p in self.particles])
		if True:#np.isnan(weights/np.sum(weights)).any():
			weights = weights + 0.00001
		for i in range(self.particle_count):
			if  i < int(self.particle_count*min(max(0, 1-self.wfast/self.wslow),0.25)):
				
				x, y, theta = self.random_free_place()
				particles.append(Particle(x, y, theta))
			else:
				#x = self.sampling(weights, self.particle_count - len(particles))
				#for index in x:
				#	particles.append(self.particles[index])
				#break
				p = (weights+0.00001)/np.sum((weights+0.00001))
				particles.append(self.particles[np.random.choice(self.particle_count,p = p)])
		self.particles = np.array(particles)
		return		
		'''


		weights = np.array([p.w for p in self.particles])
		
		to_keep = (-weights).argsort()[:int(self.particle_count*2/10)] # Keep 20% with largest weights
		
		particles = self.particles[to_keep]
		for i in range(int(self.particle_count*2/10)):
			x, y, theta = self.random_free_place()
			particles = np.append(particles, Particle(x, y, theta))
		while len(particles) < self.particle_count:
			particles = np.append(particles, self.particles[np.random.choice(to_keep)])
			particles[-1].x = particles[-1].x + np.random.uniform(low=-0.01, high=0.01)
			particles[-1].y = particles[-1].y + np.random.uniform(low=-0.01, high=0.01)
			#particles = np.append(particles,  self.particles[np.random.choice(self.particle_count, p = weights/np.sum(weights))])
		self.particles = particles#self.init_particles()'''
	'''def resample_particles(self):
			self.particles = np.array(self.particles)
			weights = np.array([p.w for p in self.particles])
			
			to_keep = (-weights).argsort()[:int(self.particle_count*3/10)] # Keep 30% with largest weights
			
			particles = np.array([])
			for i in range(self.particle_count):
				particles = np.append(particles, self.particles[np.random.choice(to_keep)])
				particles[-1].x = particles[-1].x + np.random.uniform(low=-0.01, high=0.01)
				particles[-1].y = particles[-1].y + np.random.uniform(low=-0.01, high=0.01)
				# particles[-1].theta = particles[-1].theta + np.random.choice([0, 90, -90])
				if i%5==0:
					x, y, theta = self.random_free_place()
					particles = np.append(particles, Particle(x, y, theta))
			particles[:int(self.particle_count*3/10)] = self.particles[to_keep]
			self.particles = particles
	'''
	def move_robot_and_particles(self):
		uv, uw, dt = self.robot.get_action()#uv,uw
		#print("random?")
		if True:#int(input())!=1:
			print("enter uv, uw, dt")
			uv = float(input())
			uw = float(input())
			dt = float(input())
		#print("action: "+str(uv),","+str(uw)+","+str(dt))
		self.robot.move(uv, uw, dt)
		#print("done moving")
		for p in self.particles:
			p.move(uv, uw, dt)
			p.flag = True
			if p.read_laser(self.lines)<=0.03:
				p.flag = False
def colision(r, xc, yc, rects, margin):
	xc = xc - margin[0]
	yc = yc - margin[1]
	r = 0.05
	for p1, p2, p3, p4 in rects:

		if xc > min(p1[0], p2[0], p3[0], p4[0]) - r and xc < max(p1[0], p2[0], p3[0], p4[0]) + r and yc > min(p1[1], p2[1], p3[1], p4[1]) - r and yc < max(p1[1], p2[1], p3[1], p4[1]) + r:
			'''print("Point")
			print(xc, yc, r)	
			print("Rect x1, x2")
			print(min(p1[0], p2[0], p3[0], p4[0]), max(p1[0], p2[0], p3[0], p4[0]))
			print("Rect y1, y2")
			print(min(p1[1], p2[1], p3[1], p4[1]), max(p1[1], p2[1], p3[1], p4[1]))	
			print(1)'''	
			return True
	
	'''print("Point")
	print(xc, yc, r)	
	print("Rect x1, x2")
	print(min(p1[0], p2[0], p3[0], p4[0]), max(p1[0], p2[0], p3[0], p4[0]))
	print("Rect y1, y2")
	print(min(p1[1], p2[1], p3[1], p4[1]), max(p1[1], p2[1], p3[1], p4[1]))	
	print(0)'''	
	return False
