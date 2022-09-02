import numpy as np
import math
from math import *
import copy
import shapely
from shapely.geometry import LineString,Point, Polygon

class Utility:
	def __init__(self):
		pass
	def line_intersection(self, line1, line2):
		intersection = line1.intersection(line2)
		if intersection:
			return intersection.x, intersection.y
		return False
	def laser(self, line, lines, x, y):
		intersections = []
		for l in lines:
			intersect = self.line_intersection(line, l)
			if intersect:
				intersections.append([intersect[0],intersect[1]])
		if len(intersections)==0:
			return 0.4 #laser maximum range
		return max(0.03,np.min([np.sqrt((x-i[0])**2+(y-i[1])**2) for i in intersections]))
	def create_occupancy_matrix(self, obstacle, height, width):#obstacle = 1, free = 0
		mat = np.zeros((height,width))
		for p in obstacle:
			mat[int(p[0]):int(p[0])+int(p[2]),int(p[1]):int(p[1]+p[3])] = 1#obstacle = x,y,h,w
		return mat 
	def map_info(self, rects, scale = 1):#obstacle = 1, free = 0
		xtmp = [x for rect in rects for x,_ in rect]
		ytmp = [y for rect in rects for _,y in rect]
		minx = min(xtmp)
		miny = min(ytmp)
		maxx = max(xtmp)
		maxy = max(ytmp)   
		rectstmp = np.copy(rects)	
		for i, rect in enumerate(rects):
			for j, (x,y) in enumerate(rect):
				rectstmp[i][j][0] = (rects[i][j][0]-minx) * scale
				rectstmp[i][j][1] = (rects[i][j][1]-miny) * scale
		rectstmp = np.array(rectstmp)
		obstacles = []
		scale = 1
		for i, rect in enumerate(rects):
			rxs = [x for x,_ in rect] #  
			rxs = [x for n, x in enumerate(rxs) if (x+1 not in rxs[:n]) and (x-1 not in rxs[:n]) and (x not in rxs[:n])]
			rys = [y for _,y in rect]
			rys = [y for n, y in enumerate(rys) if (y+1 not in rys[:n]) and (y-1 not in rys[:n]) and (y not in rys[:n])]
			rxs.sort()
			rys.sort()

			obstacle = [rxs[0]*scale, rys[0]*scale, (rxs[1]-rxs[0])*scale, (rys[1]-rys[0])*scale]

			obstacles.append(obstacle)

		scale = 1
		width = int((maxy - miny)*scale)+1
		height = int((maxx - minx)*scale)+1
		mat = []#self.create_occupancy_matrix(obstacles, height, width)	 
		return mat, height, width, minx, miny, maxx, maxy, obstacles
					
	def get_x1y1(self, x0, y0, t0, x_max, y_max): # Compute nearest distance to rectangle
		y1 = y_max
		x1_new = x0 + (y1 - y0) / (math.tan(t0*np.pi /180) + 0.001)
		
		d1 = np.sqrt((x1_new - x0)*2 + (y1 - y0)*2)
		
		x1 = x_max
		y1_new = y0 + (x1 - x0) * math.tan(t0*np.pi /180)
		d2 = np.sqrt((x1 - x0)*2 + (y1_new - y0)*2)

		if d1 > d2:
			y1 = y1_new
		else:
			x1 = x1_new
			
		return x1, y1
	
	def sensor_(self, particle, x_max, y_max): # Particle filter sensor reading
		out_list = []
		for i in particle:
			x0,y0,t0 = i[0],i[1],i[2]
			
			if 0<=t0<=90:
				x1, y1 = self.get_x1y1(x0, y0, t0, x_max, y_max)
			elif 91<=t0<=180:
				x1, y1 = self.get_x1y1(x0, y0, t0, 0, y_max)
			elif 181<=t0<=270:
				x1, y1 = self.get_x1y1(x0, y0, t0, 0, 0)
			else :
				x1, y1 = self.get_x1y1(x0, y0, t0, x_max, 0)
			
			out_list.append([x1, y1])
		return out_list

  
	def particle_distance(self,x0, y0, t0, mat):

		height = np.shape(mat)[0]
		width = np.shape(mat)[1]
		
		# in case of invalid values:::
		
		

		#print("Initial matrix")
		#for m in mat:
		#	print(m)

		x1, y1 = self.sensor_([[x0,y0,t0]],height, width)[0] # get border points

		#_______________ create line from initial point to border point, based on X
		if x0 < x1:
			x = np.round(np.arange(x0 , x1))

		else:
			x = np.round(np.arange(x1 , x0))

		x = x[(x>=0) & (x<height)]
		y = np.round(((y1 - y0) / (x1 - x0 + 0.001)) * (x - x0) + y0).astype(x.dtype)

		x = x[(y>=0) & (y<width)] # Remove invalid points
		y = y[(y>=0) & (y<width)]
	   
		mat = np.array(mat)
		# mat2 = np.array(copy.deepcopy(mat))
		# mat2[x.astype(int),y.astype(int)] = mat[x.astype(int),y.astype(int)] + 2
		
		xy = [[int(a),int(b)] for a,b in zip(x,y) if mat[int(a),int(b)]==1] # Save points in collision
		
		
		#________________ Create another line from initial point to border! based on Y	
		if y0 < y1:
			y = np.round(np.arange(y0 , y1))
		else:
			y = np.round(np.arange(y1 , y0))
		
		y = y[(y>=0) & (y<width)]
		x = np.round((y-y0)/((y1 - y0) / (x1 - x0 + 0.001))+x0).astype(y.dtype)
		y = y[(x>=0) & (x<height)]
		x = x[(x>=0) & (x<height)]
		
		# mat2[x.astype(int),y.astype(int)] = mat[x.astype(int),y.astype(int)] + 2
		
		# Save points in collision
		for a, b in zip(x, y):
			if [a,b] not in xy and mat[int(a), int(b)]==1:
				xy.append([a, b])
		# print(xy)

		#________________ Get distances
		equli_distance = [np.sqrt((a-x0)**2+(b-y0)**2) for a,b in xy]
		# If empty just compute distance to border
		if not equli_distance:
			equli_distance = np.sqrt((x1-x0)**2 + (y1-y0)**2)
		else:
			equli_distance = min(equli_distance)
		
		# print(equli_distance)
		
		# for m in mat2:
		#	print(m)
		
		return equli_distance
import xml.etree.ElementTree as ET
# import matplotlib.patches as patches
import matplotlib.pyplot as plt
import math
import shapely
from shapely.geometry import LineString,Point, Polygon

