import Tkinter as tk
#from Tkinter import *
import tkFileDialog
from PIL import Image, ImageTk
import datetime
from matplotlib import path 
import numpy as np
from tkColorChooser import askcolor
import math
import __builtin__
import tkMessageBox
import copy
import subprocess
import os
import signal
import rospy
import sys
import time
import threading
#import multiprocessing
#from multiprocessing import Process
#EXPLORE
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
#from maoris import test
from scipy import spatial
from scipy import ndimage
from collections import defaultdict
from random import randint
##GLOBALS##



clock_state = False


##MAP##
map_loaded=False
map_path=""
map_image=ImageTk.PhotoImage;
map_width=0;
map_height=0;
intialState = (0,0);
occMap={};
map_data=np
zoom_factor=2
flip_xy=False
regionMap = {}




fsiMap={}




##ROOMS##
rooms=list();
shader=100



##EDIT MAP##
map_with_rooms=Image;
color=((160,160,160),'gray');
selected_room=0;
create_new_mode=False;
vertex_index=0;
room_counter=1;
edit_mode=False;
default_profile="";
prompt_cancel=False;
rooms_backup=list();




##EXPLORE##
allow_map=False
end_it=False
driving=False




##NAVIGATE##
distMap={} #old
occMap={}
distanceMap={}
robot_status = 'offline'


##MAORIS
table=defaultdict(list);tableFull=defaultdict(list)
points_doors = list()
doorsMap={}

##root
world_loaded = False
world_path = ""




def donothing():
	a=1

def changeWorld(event):
	None
	global world_path
	global world_loaded
	if event=='Empty world':
		World_loaded = False
		return


	world_path = "/home/zeged/Documents/gui/worlds/" + event
	world_loaded = True
	print "/home/zeged/Documents/gui/worlds/" + event



def changeMap(event):
	None
	selectedVar3.set(event)
	global map_path
	global map_loaded
	global map_image

	if event == "new map":
		map_loaded = False
		label3.image = no_map_image
		label3.configure(image=no_map_image)
		button6.config(state='disabled') #navigate
		button4.config(state='disabled') #editMap
		map_image = no_map_image
		button2.config(state='normal')	
		return

	map_path = "/home/zeged/Documents/gui/maps/" + event + ".pgm"
	map_loaded = True
	print map_path
	root.filename = map_path
	loadMap()
	button6.config(state='normal')
	button4.config(state='normal')
	button2.config(state='disabled')
	#printRegionMap()






########MAORIS#######################################################################################


def endMaoris():
	global rooms
	global shader
	
	regionDictionary = defaultdict(int)
	for row in range(0,map_height):
		for col in range(0,map_width):
			regionDictionary[regionMap[row,col]]+=1 
 
 
	print 'regions:', len(regionDictionary)
	counter = 0
	for x in regionDictionary:
		if x>0:
			rgb = (randint(0,255), randint(0,255), randint(0,255))
			color = [rgb[0], rgb[1], rgb[2], shader]
			myroom_image = np.zeros((map_height, map_width, 4), dtype=np.uint8)
			
			myroom_image[regionMap==x]=color
			roomimg = Image.fromarray(myroom_image, 'RGBA')
			
			fullWHex = (rgb, '#%02x%02x%02x' % (rgb[0], rgb[1], rgb[2]))
			
			#myroom = (room_title, room_color, room_type, room_image_path, roomimg, regionPixels)
			regionPixels = getRegionPixels(x)
			counter+=1
			title = 'room #' + str(counter)
			rooms.append(  (title,fullWHex,"Office","images/noprofile.png", roomimg,regionPixels) )	
			
	
def getNeighbourRegion(row,col):

	if (regionMap[row-1,col-1]  !=0) : 
		return [row-1,col-1]
	if (regionMap[row  ,col-1]  !=0) : 
		return [row  ,col-1]
	if (regionMap[row+1,col-1]  !=0) : 
		return [row+1,col-1]
	if (regionMap[row-1,col  ]  !=0) :
		return [row-1,col  ]
	if (regionMap[row+1,col  ]  !=0) : 
		return [row+1,col  ]
	if (regionMap[row-1,col+1]  !=0) : 
		return [row-1,col+1]
	if (regionMap[row  ,col+1]  !=0) :
		return [row  ,col+1]
	if (regionMap[row+1,col+1]  !=0) : 
		return [row+1,col+1]
	
	return [row,col] 
	
	
	
	

#https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm
def drawLine(points):
    #"Bresenham's line algorithm"

	line = list()
	
	x0 = points[0][0]
	y0 = points[0][1]
	x1 = points[1][0]
	y1 = points[1][1]
	
	dx = abs(x1 - x0)
	dy = abs(y1 - y0)
	x, y = x0, y0
	sx = -1 if x0 > x1 else 1
	sy = -1 if y0 > y1 else 1
	if dx > dy:
		err = dx / 2.0
		while x != x1:
			line.append((x, y))
			err -= dy
			if err < 0:
				y += sy
				err += dx
			x += sx
	else:
		err = dy / 2.0
		while y != y1:
			line.append((x, y))
			err -= dx
			if err < 0:
				x += sx
				err += dy
			y += sy        
	
	return line
	
	
	

def getDistance(x,y):
	return math.sqrt((y[0]-x[0])**2 + (y[1]-x[1])**2)
						
def getFurthestPoints(points):
	
	longestDistance = 1
	longestPoints = ((0,0),(0,0))
	for x in points:
		for y in points:
			distance = getDistance(x,y)
			if distance > longestDistance:
				longestDistance = distance
				longestPoints = ((x[0],x[1]),(y[0],y[1]))
	return longestPoints
	
	

 #https://en.wikipedia.org/wiki/Connected-component_labeling
#connect neighbouring pixels
def connectDoors():
	print 'connecting doors'
	
	global doorsMap
	doorsMap = np.zeros((map_height, map_width), dtype=np.uint32)
	for x in table:
		for y in table[x]:
			doorsMap[y[0],y[1]]=1

	labeled_array, num_features = ndimage.measurements.label(doorsMap)
	print 'num_features:', num_features
	print labeled_array
	
	doorsMap = labeled_array
	
	print 'num_features:', num_features
	#unique, counts = np.unique(labeled_array, return_counts=True)
	#print dict(zip(unique, counts))
	
	
	
	for x in range(1, num_features+1):
		print x
		points = np.argwhere(doorsMap==x)
		#print points
		points_list = list()
		for y in points:
			points_list.append((y[0],y[1]))
		furthests_points = getFurthestPoints(points)
		print furthests_points
		line = drawLine(furthests_points)
		for y in line:
			doorsMap[y[0],y[1]]=x
			
			
		single_door = np.zeros((map_height, map_width), dtype=np.uint32)
		single_door[doorsMap==x]=1
		single_door = ndimage.binary_fill_holes(single_door).astype(int)
		
		
		regionMap[single_door==1]=0
		
		
		#coefficients = np.polyfit([furthests_points[0][1], furthests_points[0][0]], [furthests_points[1][1], furthests_points[1][0]], 1)
        #
		## Print the findings
		#print 'a =', coefficients[0]
		#print 'b =', coefficients[1]
		
		
		
		
		isabove = lambda p, a,b: np.cross(p-a, b-a) < 0

		#a = np.array([1,1])
		#b = np.array([4,3])
		#
		#p1 = np.array([2,4])
		#p2 = np.array([3,1])
		
		a = np.array([furthests_points[0][0],furthests_points[0][1]])
		b = np.array([furthests_points[1][0],furthests_points[1][1]])
		
		#p1 = np.array([2,4])
		#p2 = np.array([3,1])
		
		#p = np.array([p1,p2])
		above = 0
		below = 0
		area = np.argwhere(single_door==1)
		counter = 0
		for y in area:
			counter+=1
			p = y
			if counter>5:
				region = 0
				neighbour = getNeighbourRegion(y[0],y[1])
				region = regionMap[neighbour[0],neighbour[1]]
				p = neighbour
				if region != 0:
					if isabove(a,b,p):
						above = region
					else:
						below = region
				if above!=0 and below !=0:
					break
				
		
		for y in area:
			p = y
			#single_door[p[0],p[1]]=0
			if isabove(a,b,p):
				regionMap[p[0],p[1]]=above
			else:
				regionMap[p[0],p[1]]=below
		
		#single_door[single_door]
		
		
		#doorsMap[single_door==1]=x
		
		
		#move_row = furthests_points[0][0]
		#move_col = furthests_points[0][1]
		#distance = getDistance(furthests_points[0],furthests_points[1])
		#
		#
		#while isPixelEdge(move_row+1,move_col) and getDistance(move_row+1,move_col)<distance:
		#	distance = getDistance(move_row+1,move_col)
		#	move_row+=1
		#if x==1:
		
		#outer_loop = True
		#while outer_loop == True:
		#outer_loop = False
		
		fixed_door = np.zeros((map_height, map_width), dtype=np.uint32)
		
		
		distance = getDistance(furthests_points[0],furthests_points[1])
		move_points = furthests_points
		next_point = move_points[0]
		no_more_found = False
		last_point= (move_points[0][0],move_points[0][1])
		while no_more_found==False:
			no_more_found = True
        
			for row in range(-1,2):
				#print row
				for col in range (-1,2):
					if row == 0 and col == 0:
						continue
					test_point = (move_points[0][0]+row,move_points[0][1]+col)
					if getDistance(test_point,furthests_points[1]) <= distance and  isPixelEdge(test_point[0],test_point[1]) and last_point != test_point:
						
						no_more_found = False
						#outer_loop = True
						next_point = test_point
						distance = getDistance(next_point,furthests_points[1])
        
        
			line.append(next_point)
			last_point = next_point
			move_points = (next_point, move_points[1])
			
		
		#fixed_door = np.zeros((map_height, map_width), dtype=np.uint32)
		if distance == 0:
			print 'crash!'
			tmp = below
			below = above
			above = tmp
		print '2nd loop'

		if distance != 0 :
		
			distance = getDistance(next_point,furthests_points[1])
			move_points = (next_point, furthests_points[1])
			next_point2 = move_points[1]
			no_more_found = False
			last_point= (move_points[1][0],move_points[1][1])
			while no_more_found==False:
				no_more_found = True
				for row in range(-1,2):
					#print row
					for col in range (-1,2):
						if row == 0 and col == 0:
							continue
						test_point = (move_points[1][0]+row,move_points[1][1]+col)
						if x == num_features:
							print 'test_point', test_point
							print getDistance(test_point,next_point), ' < ', distance
						if getDistance(test_point,next_point) <= distance and  isPixelEdge(test_point[0],test_point[1])and last_point != test_point:
							#print test_point
							
							no_more_found = False
							#outer_loop = True
							next_point2 = test_point
							distance = getDistance(next_point, next_point2)
				line.append(next_point2)
				last_point = next_point2
				move_points = (next_point, next_point2)
				
			
			furthests_points = move_points
					
			
			
			
			
			
			
			
			
			line2 = drawLine((furthests_points[0],next_point))
			for pix in line2:
				line.append(pix)
			line2 = drawLine((next_point,next_point2))
			for pix in line2:
				line.append(pix)
			line2 = drawLine((next_point2,furthests_points[1]))
			for pix in line2:
				line.append(pix)
			
			
		for pixel in line:
			#print pixel
			fixed_door[pixel[0],pixel[1]]=1
			
		color = 0
		p = [next_point[0],next_point[1]]
		if isabove(a,b,p):
			color = below
		else:
			color = above			
			
		fixed_door= ndimage.binary_fill_holes(fixed_door).astype(int)
		regionMap[np.logical_and(fixed_door==1, regionMap!=0)]=color
			
		
		
	
def isPixelEdge(row,col):

	if ((regionMap[row-1,col-1]  ==0) |
		(regionMap[row  ,col-1]  ==0) |
		(regionMap[row+1,col-1]  ==0) |
		(regionMap[row-1,col  ]  ==0) |
		(regionMap[row+1,col  ]  ==0) |
		(regionMap[row-1,col+1]  ==0) |
		(regionMap[row  ,col+1]  ==0) |
		(regionMap[row+1,col+1]  ==0) ):
			return True
	else:
		return False
	
	
	
	
	
def detectDoors():
	print 'detecting doorways'
	points = list()
	for x in table:
		for y in table[x]:
			if isPixelEdge(y[0],y[1]):
				#or get components line
				points.append((y[0],y[1]))
				
	print 'num of points:', len(points)
	global points_doors
	points_doors = points

	
	
	
def lastMerge():
	t_merging=0.3
	m=0.0
	
	AllRipplesFound=False
	while AllRipplesFound==False:
		RippleFound=True
		completeList=list()
		AllRipplesFound=True
		regionDictionary = defaultdict(int)
		for j in range(0,map_height):
			for i in range(0,map_width):
					#regionDictionary[regionMap[j,i]]+=1 #rank by size
					regionDictionary[regionMap[j,i]]=fsiMap[j,i] #rank by value
		
		regionDictionary.pop(0)
		
		while RippleFound:
			RippleFound = False
		
			for x in completeList:
				regionDictionary.pop(x) if regionDictionary.has_key(x) else None
				
			#print regionDictionary	
			for key, value in sorted(regionDictionary.iteritems(), key=lambda (k,v): (v,k), reverse=True):
				while (RippleFound==False):
					print 'checking:', key
					regionID = key
					regionContour = getRegionContour(key)
					regionPixels = getRegionPixels(key)
					
					maxV1 = 0
					maxV2 = 0
					#get max V1
					for x in regionPixels:
						pixelValue = fsiMap[x[0],x[1]]
						maxV1= max(maxV1,pixelValue)
						
					
					
					#find neighbours
					
					
					regionNeighbours = getNeighbours(regionContour,regionID)
					
					#print regionID,' neighbours:', regionNeighbours
					
					#check for merge
					#all neighbours
					for y in regionNeighbours:
						neighbourPixels = getRegionPixels(y)
						#get max V1
						#all neighbour's pixels
						for x in neighbourPixels:
							sameNeighbourPixelValue = fsiMap[x[0],x[1]]
							maxV2= max(maxV2,sameNeighbourPixelValue)
						
						
						print 'contour size:',len(regionContour)
						
						neighbourContour = getRegionContour(y)
						regionBorder = getNeighbouringPixels(neighbourContour,regionID)
						neighbourBorder = getNeighbouringPixels(regionContour,y)
						
						
						
						minV1=1000
						minV2=1000
						
						
						for x in regionBorder:
							if fsiMap[x[0],x[1]]<minV1:
								minV1=fsiMap[x[0],x[1]]
								
	
						
						for x in neighbourBorder:
							if fsiMap[x[0],x[1]]<minV2:
								minV2=fsiMap[x[0],x[1]]
						

							
						#getRipplePixels(regionBorder, y, minV)
							
						minV=min(minV1,minV2)
																													#min region size
						if (abs(maxV2-maxV1)<=(max(maxV1,maxV2)*t_merging + m)) & (abs(minV-max(maxV1,maxV2)) < 3) | (len(neighbourPixels)<25):
					
							print abs(maxV2-maxV1),'<=',max(maxV1,maxV2)*(t_merging+m)
							print 'merging',regionID,y
							print maxV2,'-',maxV1 , 'min', minV
							mergeRegions(regionID,y)
							RippleFound = True
							AllRipplesFound = False
							completeList.append(y)
						else:
							print 'door found!'
							#points = list()
							#for x in regionBorder:
							#	#if regionMap[x[0],x[1]][0]==minV1:
							#		if isPixelEdge(x[0],x[1]):
							#			door = (x[0],x[1])
							#			points.append(door)
							#		
							#for x in neighbourBorder:
							#	#if regionMap[x[0],x[1]][0]==minV2:
							#		if isPixelEdge(x[0],x[1]):
							#			door = (x[0],x[1])
							#			points.append(door)
										
							#res = getFurthestPoints(points)
							#Doors.append(res)
							#print 'door', res
							#x1 = res[0][0]
							#x2 = res[1][0]
							#y1 = res[0][1]
							#y2 = res[1][1]
							#if (x2-x1 != 0):
							#	m = (0.0 + y2 - y1)/(x2-x1)
							#else:
							#	m=0
							#b = y1 - m*x1
							#print 'f(x)=',m, 'x+',b
							
							#for x in regionContour:
							#	if (x[0]*m + b < x[1]):
							#		changeRegion((x[0],x[1]),y)
								
							#for x in neighbourPixels:
							#	if (x[0]*m + b < x[1]):
							#		changeRegion((x[0],x[1]),regionID)
							#		
							#RippleFound = True
							#AllRipplesFound = False
							#break
							
							
							
					
					if RippleFound==False:
						RippleFound=True
						completeList.append(key)
						print 'region',key,'complete'
	
	
	
def mergeRegions(seg1,seg2):

	for x in tableFull[seg2]:
		regionMap[x[0],x[1]]=seg1
	table[seg1]+=table[seg2]
	tableFull[seg1]+=tableFull[seg2]

def getNeighbouringPixels(contourList, neighbourID):
	#neighbourCounter = 0
	neighbourPixels = list()
	for x in contourList:
		row=x[0]
		col=x[1]
		if ((regionMap[row-1,col-1] ==neighbourID) |
		(regionMap[row  ,col-1]   ==neighbourID) |
		(regionMap[row+1,col-1] ==neighbourID) |
		(regionMap[row-1,col  ]   ==neighbourID) |
		(regionMap[row+1,col  ]   ==neighbourID) |
		(regionMap[row-1,col+1] ==neighbourID) |
		(regionMap[row  ,col+1]   ==neighbourID) |
		(regionMap[row+1,col+1] ==neighbourID)):
			#neighbourCounter+=1
			neighbourPixels.append((row,col))
	
	#return neighbourCounter
	return neighbourPixels

	
def getNeighbours(contourList, myRegion):
	neighbourList = list()
	for x in contourList:
		row=x[0]
		col=x[1]
		if regionMap[row-1,col-1] !=myRegion:
			neighbourList.append(regionMap[row-1,col-1]) if regionMap[row-1,col-1] not in neighbourList else None
		if regionMap[row  ,col-1] !=myRegion:
			neighbourList.append(regionMap[row  ,col-1]) if regionMap[row  ,col-1] not in neighbourList else None
		if regionMap[row+1,col-1] !=myRegion:
			neighbourList.append(regionMap[row+1,col-1]) if regionMap[row+1,col-1] not in neighbourList else None
		if regionMap[row-1,col  ] !=myRegion:
			neighbourList.append(regionMap[row-1,col  ]) if regionMap[row-1,col  ] not in neighbourList else None
		if regionMap[row+1,col  ] !=myRegion:
			neighbourList.append(regionMap[row+1,col  ]) if regionMap[row+1,col  ] not in neighbourList else None
		if regionMap[row-1,col+1] !=myRegion:
			neighbourList.append(regionMap[row-1,col+1]) if regionMap[row-1,col+1] not in neighbourList else None
		if regionMap[row  ,col+1] !=myRegion:
			neighbourList.append(regionMap[row  ,col+1]) if regionMap[row  ,col+1] not in neighbourList else None
		if regionMap[row+1,col+1] !=myRegion:
			neighbourList.append(regionMap[row+1,col+1]) if regionMap[row+1,col+1] not in neighbourList else None
	
	
	if 0 in neighbourList:
		neighbourList.remove(0)
		#print 'yay'
	return neighbourList

def isPixelBorder(row,col):
	region=regionMap[row,col]
	if ( ( (regionMap[row-1,col-1]  !=region) and (regionMap[row-1,col-1]  !=0) ) | 
		 ( (regionMap[row  ,col-1]  !=region) and (regionMap[row  ,col-1]  !=0) ) |
		 ( (regionMap[row+1,col-1]  !=region) and (regionMap[row+1,col-1]  !=0) ) |
		 ( (regionMap[row-1,col  ]  !=region) and (regionMap[row-1,col  ]  !=0) ) |
		 ( (regionMap[row+1,col  ]  !=region) and (regionMap[row+1,col  ]  !=0) ) |
		 ( (regionMap[row-1,col+1]  !=region) and (regionMap[row-1,col+1]  !=0) ) |
		 ( (regionMap[row  ,col+1]  !=region) and (regionMap[row  ,col+1]  !=0) ) |
		 ( (regionMap[row+1,col+1]  !=region) and (regionMap[row+1,col+1]  !=0) ) ):
			return True
	else:
		return False
	
	
def isPixelContour(row,col):
	region=regionMap[row,col]
	if ((regionMap[row-1,col-1]  !=region) |
		(regionMap[row  ,col-1]  !=region) |
		(regionMap[row+1,col-1]  !=region) |
		(regionMap[row-1,col  ]  !=region) |
		(regionMap[row+1,col  ]  !=region) |
		(regionMap[row-1,col+1]  !=region) |
		(regionMap[row  ,col+1]  !=region) |
		(regionMap[row+1,col+1]  !=region) ):
			return True
	else:
		return False
	
	
def getRegionPixels(regionID):
	return tableFull[regionID]		
		
def getRegionContour(regionID):
	return table[regionID]


def generateHash2():
	print 'generating hash 2'
	
	table.clear()
	tableFull.clear()
	for row in range(0,map_height):
		for col in range(0,map_width):
			if regionMap[row,col]!=0:
				tableFull[regionMap[row,col]].append((row,col))
				#print 'pls'
				if isPixelBorder(row,col):
					#print 'yes'
					table[regionMap[row,col]].append((row,col))
	
def generateHash():
	print 'generating hash'
	
	table.clear()
	tableFull.clear()
	for row in range(0,map_height):
		for col in range(0,map_width):
			if regionMap[row,col]!=0:
				tableFull[regionMap[row,col]].append((row,col))
				if isPixelContour(row,col):
					table[regionMap[row,col]].append((row,col))
 
 
def mergeRipples():

	AllRipplesFound=False
	iteration = 0
	while AllRipplesFound==False:
		iteration+=1
		AllRipplesFound=True
		RippleFound=True
		complete=0
		completeList=list()
		regionDictionary = defaultdict(int)

		for j in range(0,map_height):
				for i in range(0,map_width):
					#regionDictionary[regionMap[j,i]]+=1 #rank by size
					regionDictionary[regionMap[j,i]]=fsiMap[j,i] #rank by value
		

		regionDictionary.pop(0)
		
		
		while RippleFound:
			RippleFound=False

		
			for x in completeList:
	
				regionDictionary.pop(x) if regionDictionary.has_key(x) else None
				
				
			
			for key, value in sorted(regionDictionary.iteritems(), key=lambda (k,v): (v,k), reverse=True):

				while (RippleFound==False):
					
					print 'checking region:', key
					
					regionID=key
					regionData = getRegionContour(regionID)
					
					regionContour=regionData
					regionNeighbours = getNeighbours(regionContour,regionID)
					neighbourCounters = 0
					#print 'region ',regionID,' has ', len(regionNeighbours),' neighbours'
					for x in regionNeighbours:
						neighbourCounters+=1
						neighbourID = x
						neighbourData = getRegionContour(neighbourID)
						neighbourContour = neighbourData
						
				
						pixelsNeighbouringContour = getNeighbouringPixels(neighbourContour,regionID)
						threshold = float(len(pixelsNeighbouringContour))/float(len(neighbourContour))
				
						if (threshold>0.4):
							#print 'merging region ',regionID, ' ', neighbourCounters,':', len(regionNeighbours),' with region:', neighbourID,' left:',len(regionDictionary)-len(completeList), 't:', threshold
							mergeRegions(regionID,neighbourID)
							RippleFound = True
							AllRipplesFound = False

							completeList.append(neighbourID)
			
					
					if (len(regionNeighbours)==0):
						print 'no neighbours!'
						for x in tableFull[regionID]:
								if (regionMap[x[0],x[1]]==regionID):
									regionMap[x[0],x[1]]=0	
									completeList.append(regionID)
					
					if (RippleFound==False):
						complete+=1
						print 'region ', regionID,' is complete. total:',complete
						completeList.append(regionID)
						RippleFound = True #just a lie to break

		print 'iteration:', iteration, ' Ripples found:', complete
		completeList=list()
		
		
		
	##count regions	
	regionDictionary = defaultdict(int)
	for j in range(0,map_height):
		for i in range(0,map_width):
			regionDictionary[regionMap[j,i]]+=1 
	regionDictionary.pop(0)
	print 'region dictionary:'
	print len(regionDictionary)
		
		
		
		
	#printRegionMap()
	
	
	
	
def printRegionMap():
	print 'printRegionMap'
	row = 0; col = 0; index = 0

	global map_width
	global map_height
	print map_width
	#image = []
	
	w, h = map_width, map_height
	data = np.zeros((h, w, 4), dtype=np.uint8)
	data[occMap==0]=[255,255,255,255] #freeSpace
	
	item_list=list()
	for row in range(0,map_height):
		for col in range(0,map_width):
			item_list.append(regionMap[row,col]) if regionMap[row,col] not in item_list else None
			#useful
			#https://stackoverflow.com/questions/19834806/is-there-a-more-pythonic-way-to-prevent-adding-a-duplicate-to-a-list
			
	#print 'regions in item_list', item_list
	for x in item_list:
		if x > 0:
			#color = '#%02x%02x%02x' % (randint(0,255), randint(0,255), randint(0,255))
			color = [randint(0,255), randint(0,255), randint(0,255), 255]
			#print 'region:',x
			data[regionMap==x]=color
			#for row in range(0,map_height):
			#	for col in range(0,map_width):
			#		if regionMap[row,col]==x:
			#			data[row,col] = color
	

	
	
	
	#item_list=list()
	#for row in range(0,map_height):
	#	for col in range(0,map_width):
	#		item_list.append(doorsMap[row,col]) if doorsMap[row,col] not in item_list else None
	#		#useful
	#		#https://stackoverflow.com/questions/19834806/is-there-a-more-pythonic-way-to-prevent-adding-a-duplicate-to-a-list
	#		
	##print 'regions in item_list', item_list
	#for x in item_list:
	#	if x > 0:
	#		#color = '#%02x%02x%02x' % (randint(0,255), randint(0,255), randint(0,255))
	#		color = [255,255,255, 255]
	#		#print 'region:',x
	#		data[doorsMap==x]=color
	#		#for row in range(0,map_height):
	#		#	for col in range(0,map_width):
	#		#		if regionMap[row,col]==x:
	#		#			data[row,col] = color
	






	
	#for y in table:
	#	for z in table[y]:
	#		data[z[0],z[1]]=[255,255,255,255]
						

	data[occMap==100]=[0,0,0,255] #draw walls
	
	
	#draw doors
	#for x in points_doors:
	#	data[x[0],x[1]]=[0,0,255,255]

				
	img = Image.fromarray(data, 'RGBA')
	img.save('my.png')
	root.image=img
	#img.show()
	#img = img.resize((map_width*zoom_factor,map_height*zoom_factor))


	global map_image
	map_image = ImageTk.PhotoImage(img)
	label3.configure(image=map_image)
	label3.image = map_image


 
 
 
 
 #https://en.wikipedia.org/wiki/Connected-component_labeling
#connect neighbouring pixels
def connectNeighbours():
	print 'conecting'
	global map_width
	global map_height
	global regionMap
	regionMap = np.zeros((map_height, map_width), dtype=np.uint32)
	sameRegion=defaultdict(int)
	#sameRegion=65536*[None] ##change to defaultdict(int)
	#regions[0].append((0,1))
	k=0

 
	for row in range(0,map_height):
		for col in range(0,map_width):
			#regionMap[i,j]=(fsiMap[i,j],0)
   
			if fsiMap[row,col]>0:
    
	##check behind
				if ( (col-1>0) & (fsiMap[row, col-1]==fsiMap[row,col]) & ( (row-1<0) | (fsiMap[row-1,col]!=fsiMap[row,col]) ) ):
					regionMap[row,col]=regionMap[row,col-1]
	##check above
				elif ( ( (col-1<0) | (fsiMap[row,col-1]!=fsiMap[row,col]) ) & (col-1>0) & (fsiMap[row,col-1]==fsiMap[row,col]) ):
					#print 'above'
					regionMap[row,col]=regionMap[row,col-1]
					#print 'ok'
	##check both
				elif ( (col-1>0) & (row-1>0) & (fsiMap[row-1,col]==fsiMap[row,col-1]) & (fsiMap[row,col-1]==fsiMap[row,col]) ):
					#print 'both'
					if (regionMap[row,col-1]!=regionMap[row-1,col]):	#conflict
						#print regionMap[i-1,j][1], ' ', regionMap[i,j-1][1]
						minK = min(regionMap[row,col-1],regionMap[row-1,col])
						maxK = max(regionMap[row,col-1],regionMap[row-1,col])
						#newNode = Node(maxK)
						#newNode.next=sameRegion[minK]
						#print 'conflict:',minK,' ', maxK
						#print 'min:', minK, ' max:',maxK
						sameRegion[maxK]=minK
						regionMap[row,col]=minK
						None
					else: #no conflict
						#print 'no conflict'
						regionMap[row,col]=regionMap[row,col-1]
						None
				else: #new label
					#print 'new label'
					k+=1
					regionMap[row,col]=k
					#print 'k:',k
					sameRegion[k]= 0
			else: #wall, i
				None
	 
     
	print k
	print len(sameRegion)
	
	for index, e in sorted(sameRegion.iteritems(), key=lambda (k,v): (v,k), reverse=True):
	#for index, e in reversed(sameRegion).iteritems():
		if e != None:
			if e != 0:
				#print 'should switch ', index, '->', e
				regionMap[regionMap==index]=e
				print e, ':', k
	
	
 
 
 
 
 	regionDictionary = defaultdict(int)
	#sortedRegions = defaultdict(int)
	for row in range(0,map_height):
		for col in range(0,map_width):
			regionDictionary[regionMap[row,col]]+=1 
 
 
	print 'regions:', len(regionDictionary)

 
 
 
 
 
 
 

#https://stackoverflow.com/questions/10818546/finding-index-of-nearest-point-in-numpy-arrays-of-x-and-y-coordinates
def createFSImap():
	print 'FSI'
	global occMap
	global distanceMap
	global fsiMap
	
	
	distanceMap = np.zeros((map_height, map_width), dtype=np.uint32)
	#max_distance = 0
	#z = np.zeros((map_height,map_width))
	print 'distance shape:', distanceMap.shape
	#print 'z shape:', z.shape
	#mask = occMap > 0	

	# Coordinates of non-black pixels.
	#walls = np.argwhere(mask)

	
	distanceMap[occMap==0]=1 #free space
	distanceMap[occMap!=0]=0 #occupied
	#for row in range(0,map_height):
	#	for col in range(0,map_width):
	#		pt = [row, col]
	#		if (occMap[row, col]==0):
	#			distanceMap[row,col]=1
	#		else:
	#			distanceMap[row,col]=0
	#		#nearest_point = walls[spatial.KDTree(walls).query(pt)[1]]
	#		#print 'nearest point:', nearest_point
	#			#distance,index = spatial.KDTree(walls).query(pt)
	#			#distanceMap[row,col]=int(math.floor(distance))
	#			#max_distance = max(max_distance,distance)
	#	print row	
	#https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.ndimage.morphology.distance_transform_edt.html
	#https://stackoverflow.com/questions/34681631/distance-transform-algorithms-in-scipy?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	distanceMap = ndimage.distance_transform_edt(distanceMap)	
	
	
	#distanceMap[occMap==-1]=-1
	
	
	saveDistanceMap()
	root.update_idletasks()
	#time.sleep(2)
	
	z=distanceMap.copy()
	
	for row in range(0,map_height):
		for col in range(0,map_width):
			#pt = [row, col]
			if (distanceMap[row, col]>0):
				# specify circle parameters: centre ij and radius
				distance = distanceMap[row, col]
				#ci,cj=row,col
				#cr=distance

				# Create index arrays to z > maybe move outside
				#I,J=np.meshgrid(np.arange(z.shape[1]),np.arange(z.shape[0]))
				

				# calculate distance of all points to centre
				#dist_formula= np.sqrt((J-cj)**2+(I-ci)**2)<distance
				#form2 = [(dist_formula<cr) & (z<distance)]
				#print 'dist_formula shape:', dist_formula.shape
				# Assign value of 1 to those points where dist<cr:
				
				#mask2 = np.logical_and(dist_formula, distanceMap<distance, z<distance)
				
				
				y,x = np.ogrid[-row:map_height-row, -col:map_width-col]
				r = distance
				mask3 = y*y + x*x <= r*r
				
				z[np.logical_and(mask3, np.logical_and( z<distance, z>0) ) ]=distance
			

				#fsiMap = z

				#print 'distance:', distance
				#print 'index:', index
			
			
		print row
	fsiMap = z
	#occMap = distanceMap

	#https://stackoverflow.com/questions/23667646/python-replace-zeros-in-matrix-with-circle-of-ones?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	

	# specify circle parameters: centre ij and radius




	global doorsMap
	doorsMap = np.zeros((map_height, map_width), dtype=np.uint32)
	
	saveFSI()
	root.update_idletasks()
	root.update()
	time.sleep(1)
	connectNeighbours()
	#printRegionMap()
	root.update_idletasks()
	root.update()
	time.sleep(1)
	generateHash()
	mergeRipples()
	#printRegionMap()
	root.update_idletasks()
	root.update()
	generateHash()
	lastMerge()
	#printRegionMap()
	root.update_idletasks()
	root.update()
	time.sleep(3)
	generateHash2()
	detectDoors()
	connectDoors()
	#printRegionMap()
	generateHash2()
	root.update_idletasks()
	root.update()
	time.sleep(3)
	saveImage()
	endMaoris()
	root.update_idletasks()
	root.update()
	time.sleep(3)
	#printRegionMap()
	saveImage()
	
	
	
	print 'done'


def saveDistanceMap():
	print 'printMap'
	row = 0; col = 0; index = 0

	global map_width
	global map_height
	print map_width
	#image = []
	
	w, h = map_width, map_height
	data = np.zeros((h, w, 4), dtype=np.uint8)
	
	color = [0,0,0]
	
	for row in range (0,map_height):
		for col in range (0,map_width):
			#print col,row
			pixel = occMap[row,col]
 
		#FREE RASTER 
			if pixel == 0:   
				data[row,col]=[0,0,0,distanceMap[row,col]*5]#(0xffAA0000)
				



				
	img = Image.fromarray(data, 'RGBA')
	img.save('my.png')
	root.image=img
	#img.show()
	global map_image
	map_image = ImageTk.PhotoImage(img)
	label3.configure(image=map_image)
	label3.image = map_image
	
	
	

def saveFSI():
	print 'printMap'
	row = 0; col = 0; index = 0

	global map_width
	global map_height
	print map_width
	#image = []
	
	w, h = map_width, map_height
	data = np.zeros((h, w, 4), dtype=np.uint8)
	
	color = [0,0,0]
	
	for row in range (0,map_height):
		for col in range (0,map_width):
			#print col,row
			pixel = occMap[row,col]
 
		#FREE RASTER 
			if pixel == 0:   
				data[row,col]=[0,0,0,fsiMap[row,col]*5]#(0xffAA0000)
				



				
	img = Image.fromarray(data, 'RGBA')
	img.save('my.png')
	root.image=img
	#img.show()
	global map_image
	map_image = ImageTk.PhotoImage(img)
	label3.configure(image=map_image)
	label3.image = map_image







def createDistanceMapOld():
#fill DistMap
 global distMap
 global occMap
 print 'calculating distance'
 for j in range(0,map_height):
  print j, '/', map_height
  for i in range(0,map_width):
   if occMap[i,j]==0:
    distMap[i,j]=getObstacleDistance(i,j)
   else:
    distMap[i,j]=-1       








def getObstacleDistance(row,col):
 r=1
 while obstacleInDistance(row,col, r)!=True:
  r+=1
 return r
    


def obstacleInDistance(x0,y0,radius):
 if (x0-radius<0) or (x0+radius>map_width-1) or (y0-radius<0) or (y0+radius>map_height-1):
  return True
 y=-radius
 x=-radius
 while y<=radius:
  while x<=radius:
   if (x*x+y*y <= radius*radius
   and (occMap[x0 + x, y0 + y]==100
   or occMap[x0 + x, y0 + y]==100
   or occMap[x0 - y, y0 + x]==100
   or occMap[x0 - y, y0 + x]==100
   or occMap[x0 - x, y0 - y]==100
   or occMap[x0 - y, y0 - x]==100
   or occMap[x0 + y, y0 - x]==100
   or occMap[x0 + x, y0 - y]==100)):
    #print 'x:',x0,' y:', y0,' radius:',radius
    return True
   x+=1
  y+=1
  #print y, '/', width_map
  x=-radius

 
 return False












	

def sendGoal(x,y,w):
	print 'going to (',x,',',y,',',w,')'
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	goal = MoveBaseGoal()
	#goal.target_pose.pose = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
	
	goal.target_pose.pose = Pose(Point(y, x, 0.000), Quaternion(0.000, 0.000, w, 0.975))
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	move_base.send_goal(goal)





def send_checkpoint(x, y, w):
	print 'x:', x
	print 'y:', y
	print 'x0:', intialState[1]
	print 'y0:', intialState[0]
	x=float(-x+intialState[1])/20
	y=float(y-intialState[0])/20
	w=0
	sendGoal(x, y, w)


		
	



def createMapNumpy(path):
	global occMap
	global map_height
	global map_width	
	print 'loading occMap'
	##open files
	#lines = open(path).read().splitlines()
	
	mymap = np
	#https://stackoverflow.com/questions/7368739/numpy-and-16-bit-pgm?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	f =  open(path, 'r')
	header = f.readline()
	print header
	print '1', f.readline()
	dimentions =  f.readline().strip().split(' ')
	map_width = int(dimentions[0])
	map_height = int(dimentions[1])
	print dimentions[0],'x',dimentions[1]
	print '3', f.readline()
	#map_width, map_height= 4000, 4000 #, maxval = [int(item) for item in header.split()[1:]]
	
	data_size = map_height * map_width * 1
	f.seek(0, 2)
	filesize = f.tell()
	f.close()
	i_header_end = filesize - (data_size)

	f = open(path, 'rb')
	f.seek(i_header_end)
	buffer = f.read()
	f.close()
	
	# convert binary data to an array of the right shape
	mymap = np.frombuffer(buffer, dtype=np.uint8).reshape((map_height, map_width)).astype(np.int16)
	
	mymap[mymap==0]=100 #wall
	mymap[mymap==254]=0 #free
	mymap[mymap==205]=-1 #unknown
	print mymap
	#unique, counts = np.unique(mymap, return_counts=True)
	#print dict(zip(unique, counts))
	#mymap = np.fromfile(f, dtype=np.uint16).reshape((map_height, map_width))
	
	
	
	
	print 'lines open'
	#map_width =  int(lines[11].split(":")[1])
	#map_height = int(lines[12].split(":")[1])
	##set full_resolution
	print 'resolution =', map_width,"x", map_height
	#array_map =  lines[23].split(",")##CREATE MAP
	#max = int((map_width*map_height) - 1)
	#array_map[0]=-1
	#array_map[max]=-1
	

	#w, h = map_width, map_height
	#data = np.zeros((h, w), dtype=np.int8)
	
	print 'parsing from str to int'
	#mymap = __builtin__.map(int, array_map)
	#mymap_np = np.array(array_map)
	#mymap = mymap_np.astype(np.int16)
	#mymap = mymap.reshape((map_height, map_width))
	#data = np.flipud(mymap) #flip_xy
	data = mymap
	print 'array rdy'

	
	
	#print 'creating Occ map'
	#row = 0; col = 0; index = 0
	#for occ in mymap:
	#	data[col,row]=mymap[index]
    #
	#	index+=1
    #
	#		
    #
	#	col += 1
	#	if col == map_width:
	#		row +=1; col = 0
    #
    #
    #
    #
	##numpy_array = np.array(array_map)
	print 'new numpy', data.shape


	#data = np.rot90(data)

	mask = data == 100	

		# Coordinates of non-black pixels.
	coords = np.argwhere(mask)
	print 'coords = np.argwhere(mask)'
		# Bounding box of non-black pixels.
	y0, x0 = coords.min(axis=0)
	y1, x1 = coords.max(axis=0) + 1   # slices are exclusive at the top
	
		# Get the contents of the bounding box.
	
		#print 'point1', x0, y0
		#print 'point2', x1, y1 
	
		#print x0,':',x1,' ',y0,':',y1
		#global scale
		#scale = (x0,x1,y0,y1)




	Margin = 10
	x0=x0-Margin
	y0=y0-Margin
	x1=x1+Margin
	y1=y1+Margin



	cropped = data[y0:y1, x0:x1]

	
	print 'cropped', cropped.shape

	#data2 = np.rot90(cropped)
	#occMap=data2
	occMap=cropped


	global map_data
	map_data = cropped 
	initialX = map_height/2
	initialY = map_width/2
		
	initialX= initialX - x0
	initialY= initialY - y0
	global intialState
	intialState = (initialY,initialX)
	print 'initial state:', intialState
	#print 'done cropped'
	#print intialState
		
	map_height=y1-y0
		
	map_width=x1-x0







			
def createMapMatrix(path):
	global occMap
	global map_height
	global map_width	
	print 'loading occMap'
	##open files
	lines = open(path).read().splitlines()
	print 'lines open'
	map_width =  int(lines[11].split(":")[1])
	map_height = int(lines[12].split(":")[1])
	##set full_resolution
	print 'resolution =', map_width,"x", map_height
	array_map =  lines[23].split(",")##CREATE MAP
	max = int((map_width*map_height) - 1)
	array_map[0]=-1
	array_map[max]=-1
	

	
	#numpy_array = np.array(array_map)
	#https://stackoverflow.com/questions/12575421/convert-a-1d-array-to-a-2d-array-in-numpy
	#int_array = numpy_array.astype(np.int)
	#matrix = np.reshape(int_array, (-1, map_width))
	
	
	
	
	
	
	
	
	
	print 'parsing from str to int'
	mymap = __builtin__.map(int, array_map)

	print 'array rdy'

	
	
	print 'creating Occ map'
	row = 0; col = 0; index = 0
	for occ in mymap:
		pixel = mymap[index]
		#print pixel
		index+=1
		#UNKNOWN RASTER
		if pixel == -1:
			occMap[col,row]=-1
		#FREE RASTER 
		elif pixel == 0:
			occMap[col,row]=0;
		#WALL OCCUPIED RASTER
		else:
			occMap[col,row]=100
		col += 1
		if col == map_width:
			row +=1; col = 0
	

	resizeMap()
	
	
	
def resizeMap(): #crop
	print 'resizing'
	global map_height
	global map_width 
	global occMap
	smallMap={}
	x1=map_width
	x2=0
	y1=map_height
	y2=0
	
	initialX = map_height/2
	initialY = map_width/2
	
	print 'creating matrix'
	for j in range(0,map_height):
		for i in range(0,map_width):
			#array_num=int(j*map_width+i)
			#print array_num
			if occMap[i,j]==100:
				x1=min(x1,i)
				y1=min(y1,j)
				x2=max(x2,i)
				y2=max(y2,j)
				#print 'yes', x1, x2, y1, y2
	
	
	#exclusive at top
	y2+=1
	x2+=1

	margin = 10 #padding

	x1-=margin
	y1-=margin
	x2+=margin
	y2+=margin



	initialX= initialX - x1
	initialY= initialY - y1
	global intialState
	intialState = (initialX,initialY)
	#y = "Those who know %s and those who %s." % (binary, do_not)
	text_location = 'initial location: (%s,%s) ' %(initialX,initialY)
	label.config(text= text_location)
	

	
	map_width =  x2 - x1 #-1	
	map_height = y2 - y1 #-1 
	print 'new resolution:', map_width,'x',map_height
	
	counter=0
	print 'creating map'
	for j in range(0,map_height):
		for i in range(0,map_width):
			smallMap[i,map_height-j-1]=occMap[i+x1,j+y1] #flipped y
			counter+=1
	print 'duplicating'
	occMap=smallMap	
	print 'done'

def applyRooms():
	back = root.image #Image.open(root.filename).convert("RGBA")
	blend = back
	print 'iterating rooms:', len(rooms)
	for x in rooms:
		layer = x[4]
		blend = Image.alpha_composite(blend, layer)
	global map_with_rooms
	map_with_rooms = blend
	tmp = ImageTk.PhotoImage(blend)
	label3.configure(image=tmp)
	label3.image = tmp

	
def saveImage():
	print 'printMap'
	row = 0; col = 0; index = 0

	global map_width
	global map_height
	print map_width
	#image = []
	
	w, h = map_width, map_height
	data = np.zeros((h, w, 4), dtype=np.uint8)
	
	color = [0,0,0]
	
	for row in range (0,map_height):
		for col in range (0,map_width):
			#print col,row
			pixel = occMap[row,col]
	
		#UNKNOWN RASTER
			if pixel == -1:
				data[row,col]=[255,204,153,0] #(0xffFFCC99) 
		#FREE RASTER 
			elif pixel == 0:   
				data[row,col]=[255,255,255,255]#(0xffAA0000)
				
		#WALL OCCUPIED RASTER
			elif pixel > 0:
				data[row,col]=[0,0,0,255]#(0xffFFFF00)


				
	img = Image.fromarray(data, 'RGBA')
	img.save('my.png')
	root.image=img
	#img.show()
	global map_image
	map_image = ImageTk.PhotoImage(img)
	label3.configure(image=map_image)
	label3.image = map_image
	
	
def createRegionMap():
	#create regionmap
	global regionMap
	regionMap = np.zeros((map_height, map_width), dtype=np.uint32) #, dtype=np.uint32

	counter=1
	for x in rooms:
		regionPixels = x[5]
		for y in regionPixels:
			regionMap[y[0],y[1]]=counter
		counter+=1
			

def createDistanceMap():
	#create distance map




	global distanceMap

	
	
	distanceMap = np.zeros((map_height, map_width), dtype=np.uint32)

	print 'distance shape:', distanceMap.shape

	distanceMap[occMap==0]=1 #free space
	distanceMap[occMap!=0]=0 #occupied	
	#https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.ndimage.morphology.distance_transform_edt.html
	#https://stackoverflow.com/questions/34681631/distance-transform-algorithms-in-scipy?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	distanceMap = ndimage.distance_transform_edt(distanceMap)	
	



			
def loadMap():
	try:
		global map_loaded
		global shader
		if map_loaded == False:
			root.filename = tkFileDialog.askopenfilename(title = "Select file",defaultextension='.map', filetypes = (("ROS map files","*.pgm"),("all files","*.*")))
		print (root.filename)
		if root.filename != "":

			
			#createMapMatrix(root.filename)
			createMapNumpy(root.filename)
			print 'height:', map_height
			saveImage()
			
			global room_counter
			
			map_loaded=True
			print 'map loaded'
			#rooms
			global rooms
			rooms = list()
			inoPath = root.filename.rsplit('.', 1)[0] + str('.ino')
			print os.path.isfile(inoPath)
			ino_exist = os.path.isfile(inoPath)
			if ino_exist==True:
				lines = open(inoPath).read().splitlines()
				for z in lines[1:]:
					#print x
					line = z.split(';')
					print '0', line[0]
					print '1', line[1]
					print '2', line[2]
					print '3', line[3]
					#print '4', line[4]
					mytup = line[4].split(',')
					newlist = list()
					mytup2 = list()
					x=0
					while x<len(mytup):
						mytup2.append((int(mytup[x].replace("(","").replace(")","").replace("[","").replace("]","").replace(" ","")) ,int(mytup[x+1].replace("(","").replace(")","").replace("[","").replace("]","").replace(" ","")) ))
						x+=2

				
					rgb = line[1].split(',')
					#if str(rgb[3])!='gray':
					#	print rgb[3]
					#	rgb[3] = hex(rgb[3])
					print rgb
					color = ((rgb[0],rgb[1],rgb[2]),rgb[3])
					myroom_image = np.zeros((map_height, map_width, 4), dtype=np.uint8)
					for x in mytup2: #regionPixels
						print rgb[0]
						print rgb[1]
						print rgb[2]
						print x[0]
						print x[1]
					
						myroom_image[x[0],x[1]]=[rgb[0], rgb[1], rgb[2], shader]
					roomimg = Image.fromarray(myroom_image, 'RGBA')
			
			
					#myroom = (room_title, room_color, room_type, room_image_path, roomimg, regionPixels)
					rooms.append(  (line[0],color,line[2],line[3], roomimg,mytup2) )
					room_counter+=1
				createRegionMap()
				createDistanceMap()
				applyRooms()

	
	except:
		print "Unexpected error:", sys.exc_info()[0]
		raise
	
	
	
	
	
def loadMapPNG():
	try:
		root.filename = tkFileDialog.askopenfilename(title = "Select file",defaultextension='.png', filetypes = (("ROS map files","*.png"),("all files","*.*")))
		print (root.filename)
		if root.filename != "":
			global map_image
			map_image = ImageTk.PhotoImage(file = root.filename)
			#tmp_map_image=Image.open(root.filename).convert("RGBA")
			label3.configure(image=map_image)
			label3.image = map_image
			global map_height
			map_height=map_image.height()
			global map_width
			map_width = map_image.width() 
	except:
		print "Unexpected error:", sys.exc_info()[0]
		raise
	
	
	
		
	
	
def saveMap():
	print 'saving map'
	print 'num of rooms:', len(rooms)
	if map_loaded:
		print 'map loaded ', root.filename
	else:
		print 'prompt new file'
		root.filename= tkFileDialog.asksaveasfilename(title='SVG export filename',filetypes=[("ROS map","*.pgm")], defaultextension = '.pgm')
		if root.filename=="":
			return
		else:
			print 'saved to', root.filename
			print root.filename.rsplit('.', 1)[0]
		

	print 	'saving to files'
	#cmd = 'rosrun map_server map_saver -f /tmp/my_map'
	#cmd = 'rosrun map_server map_saver -f '+ str(root.filename)
	global clock_state
	if (clock_state == True):
		#cmd = 'rostopic echo -n1 /map > ' + str(root.filename)	
		#print cmd

		#tmp_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
	
		#for line in tmp_proc.stdout:
		#	print '#',line
	
		#for line in tmp_proc.stderr:
		#	print '#',line

		cmd = 'rosrun map_server map_saver -f ' + str(root.filename.rsplit('.', 1)[0])	
		print cmd

		tmp_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
	
		for line in tmp_proc.stdout:
			print '#',line
	
		for line in tmp_proc.stderr:
			print '#',line

		
		




	else:
		print 'ros server down'

	filepath = root.filename.rsplit('.', 1)[0] + str('.ino')
	with open(filepath, 'w') as the_file:
    		the_file.write('Hello\n')
		for x in rooms:
			print x[0]
			print x[1][0]
			print x[1][1]
			color = '{},{},{},{}'.format(x[1][0][0],x[1][0][1],x[1][0][2],x[1][1])
			line = x[0]+';'+str(color)+';'+x[2]+';'+x[3]+';'+str(x[5])+'\n'
			#new_room = (room_title, room_color, room_type ,image_path, roomimg, regionPixels)
			the_file.write(line)


	#update gui choices
	#global choices_map


	global menu3
	title = root.filename.rsplit('.', 1)[0]
	#choices_map.append(root.filename.rsplit('.', 1)[0])
	name = title.rsplit('/', 1)[1]
	menu3['menu'].add_command(label=name, command=lambda t=name : changeMap(t))
	
	

	print 'saved'
	
	
	
	
	
	
	
			
def OptionMenu_SelectionEvent(event): # I'm not sure on the arguments here, it works though
    ## do something
	print event
	if event=='off':
		menu2.config(state='disabled')
	else:
		menu2.config(state='normal')

    #pass
			
		
def clock():
	if clock_state==False:
		return
	tend = datetime.datetime.now()
	elapsed=tend-start_time
	avgString ='up time:', str(elapsed).split(".")[0] #remove ms
	
	status.config(text=avgString)
	root.after(1000, clock) # run itself again after 1000 ms		


def run_ROS():
	print 'running ros'
	global start_time
	start_time = datetime.datetime.now()
	global clock_state
	clock_state = True	
	clock()
	command1_ROS()
	global robot_status
	robot_status = 'online'


def command1_ROS():
	print 'command1'
	global world_path
	global world_loaded
	global map_path
	global map_loaded
	cmd = 'roslaunch robotican_armadillo armadillo.launch lidar:=true gazebo:=true move_base:=true' # amcl:=true
	global map_loaded
	if map_loaded==True:
		map_file = map_path.rsplit('.', 1)[0] + '.yaml'
		cmd = cmd + ' amcl:=true have_map_file:=true map_file:="{}"'.format(map_file)
	else:
		cmd = cmd + ' gmapping:=true'
	if world_loaded==True:
		cmd = cmd + ' world_name:="{}"'.format(world_path)


	#cmd = 'roslaunch robotican_armadillo armadillo.launch lidar:=true gazebo:=true gmapping:=true move_base:=true world_name:="`rospack find robotican_common`/worlds/testing_room2.sdf"'
	#cmd = 'roslaunch robotican_armadillo armadillo.launch lidar:=true gazebo:=true gmapping:=true move_base:=true world_name:="worlds/testing_room.sdf"'
	print 'cmd = ', cmd
	outfile = open("outfile", "w")
	#global proc_gazebo
	#proc = subprocess.Popen(cmd, stdout=outfile, stderr=outfile, shell=True, preexec_fn=os.setsid)
	#root.proc_gazebo = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
	root.proc_gazebo = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setpgrp)
	print 'pid:', os.getpid()
	#global gazebo_pid
	#gazebo_pid=os.getpid()


	time.sleep(10)

	rospy.init_node('test_node')

	#button_stop_ROS.config(state='normal')
	#button1.config(state='disabled')
	#rospy.sleep(8.0)
	if map_loaded==True:
		return
	#command1_FE()

	

def command2_ROS():
	print 'command2'
	#global gazebo_pid
	#global proc_gazebo
	print 'killing:', root.proc_gazebo.pid
	#proc.terminate()
	#os.killpg(os.getpgid(root.proc_gazebo.pid), signal.SIGTERM)
	os.kill(-root.proc_gazebo.pid, 	signal.SIGTERM)
	for line in root.proc_gazebo.stdout:
		print '#',line
	for line in root.proc_gazebo.stderr:
		print '#',line
	root.proc_gazebo.wait()
	#command2_FE()


def stop_ROS():
	print 'stopping ros'
	command2_ROS()
	#button_stop_ROS.config(state='disabled')
	#button1.config(state='normal')
	global clock_state
	clock_state = False
	status.config(text='offline.')
	global robot_status
	robot_status = 'offline'








def command1_FE():
	print 'command1'
	#cmd = "./myexe arg1 arg2 >& outfile"
	cmd = "roslaunch husky_navigation exploration_demo_noserver.launch"
	print 'cmd = ', cmd
	root.proc_FE = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
	print 'pid:', os.getpid()
	#global pid
	#pid=os.getpid()

def command2_FE():
	print 'command2'
	#os.kill(pid, signal.SIGTERM)
	os.kill(-root.proc_FE.pid, signal.SIGTERM)
	for line in root.proc_FE.stdout:
		print '#',line
	for line in root.proc_FE.stderr:
		print '#',line
	root.proc_FE.wait()














def stop():
	print 'stop'
	#cmd = "./myexe arg1 arg2 >& outfile"
	cmd = "rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}"
	print 'cmd = ', cmd
	#proc = subprocess.Popen(command, shell=True)
	outfile = open("outfile", "w")

	#tmp_proc = subprocess.Popen(cmd, stdout=outfile, stderr=outfile, shell=True, preexec_fn=os.setsid)


	tmp_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
	
	for line in tmp_proc.stdout:
		print '#',line
	
	for line in tmp_proc.stderr:
		print '#',line


	#print 'pid:', os.getpid()
	#rospy.sleep(3.0)
	#os.killpg(os.getpgid(tmp_proc.pid), signal.SIGTERM)








#EXPLORE#######################################################################################################	
def explore_window():
	image_fromarray = Image
	#allow_map=False



	def back():
		#command2_FE()
		global allow_map

		allow_map=False

		global end_it
		end_it=True

		map_sub.unregister()

		odom_sub.unregister()
		path_sub.unregister()

		


		answer = tkMessageBox.askyesno("Map not saved","Save map?")
		print answer
		if answer==True:
			saveMap()






		global map_image
		#map_image = ImageTk.PhotoImage(img)
		map1 = Image.open("images/nomap.png")

		small_map = map1.resize((150,150)) #better switch to fill
		root.image = small_map
		map2 = ImageTk.PhotoImage(small_map)

		#global map_image
		map_image = map2



		label3.configure(image=map_image)
		label3.image = map_image




		global occMap
		global map_image
		global map_width
		global map_height
		global intialState
		global map_data



		rospy.sleep(1.5)

		
		#occMap={};
		#print 'creating matrix'
		#for j in range(0,map_height):
		#	for i in range(0,map_width):
			#array_num=int(j*map_width+i)
			#print array_num
		#		occMap[i,j]=map_data[j,i]

		occMap = map_data

		


		stop_FE()

		#rospy.sleep(3.0)

		stop_ROS()




		#rospy.sleep(3.0)

		root.deiconify()
		root.geometry('%dx%d+%d+%d' % (1024, 600, root.explore_window.winfo_x(), root.explore_window.winfo_y()))
		root.explore_window.destroy()

	#https://stackoverflow.com/questions/3221655/python-threading-string-arguments
	def callback_map_thread(data):
		processThread = threading.Thread(target=callback_map, args=(data,))  # <- note extra ','
		#processThread = multiprocessing.Process(target=callback_map, args=(data,))
		#processThread.setDaemon(True)
		processThread.start()
		#processThread.join()




	def callback_map(data):
		global allow_map
		print 'got map', allow_map
		if allow_map==False:
			print 'False'
			return
		allow_map=False
		#print 'length:', len(data.data)
		#print data.header
		#print '##########'
		#print data.info
		#getMap(data.data)
		mymap = occupancygrid_to_numpy(data)
		print 'occupancygrid_to_numpy(data)'
		mymap = np.flipud(mymap)
		print 'mymap = np.flipud(mymap)'
		#img5 = Image.fromarray(mymap, 'L')
		#img5.save('mymap5.png')
		#print 'done numpy'
		
		global map_height
		map_height=data.info.height
		global map_width
		map_width=data.info.width
	
		initialX = map_height/2
		initialY = map_width/2
		global intialState
		#intialState = (initialX,initialY)
		
	
	#https://codereview.stackexchange.com/questions/132914/crop-black-border-of-image-using-numpy
		# Mask of non-black pixels (assuming image has a single channel).
		#mask = mymap > -1
		mask = mymap == 100	

		# Coordinates of non-black pixels.
		coords = np.argwhere(mask)
		print 'coords = np.argwhere(mask)'
		# Bounding box of non-black pixels.
		y0, x0 = coords.min(axis=0)
		y1, x1 = coords.max(axis=0) + 1   # slices are exclusive at the top
	
		# Get the contents of the bounding box.
	
		#print 'point1', x0, y0
		#print 'point2', x1, y1 
	
		#print x0,':',x1,' ',y0,':',y1
		#global scale
		#scale = (x0,x1,y0,y1)




		Margin = 10
		x0=x0-Margin
		y0=y0-Margin
		x1=x1+Margin
		y1=y1+Margin



		cropped = mymap[y0:y1, x0:x1]
		print 'cropped = mymap[y0:y1, x0:x1]'
		#img5 = Image.fromarray(cropped, 'L')
		#img5.save('cropped.png')

		print 'shape:', cropped.shape

		global map_data
		map_data = cropped 

		
		initialX= initialX - x0
		initialY= initialY - y0
		intialState = (initialX,initialY)
		print 'initial state:', intialState
		#print 'done cropped'
		#print intialState
		
		map_height=y1-y0
		
		map_width=x1-x0
	
		#print 'resolution:', map_width, map_height
	
		#canvas2.config(width=map_width*zoom_factor, height=map_height*zoom_factor)
	
	
		w, h = map_width, map_height
		data2 = np.zeros((h, w, 4), dtype=np.uint8)
		
		color = [0,0,0]
		
		for row in range (0,map_height):
			for col in range (0,map_width):
				#print col,row
				pixel = cropped[row,col]
		
			#UNKNOWN RASTER
				if pixel == -1:
					data2[row,col]=[255,204,153,0] #(0xffFFCC99)
			#WALL OCCUPIED RASTER
				elif pixel > 1:   
					data2[row,col]=[0,0,0,255]#(0xffAA0000)BLACK
			#FREE RASTER 
				else:
					data2[row,col]=[255,255,255,255]#(0xffFFFF00)OFFWHITE
	
		#data2[intialState[1],intialState[0]]=[0,255,0,255]


		global flip_xy
		if flip_xy==True:
			data2 = np.rot90(data2)
			tmp = map_width
			map_width=map_height
			map_height=tmp
			intialState=(intialState[1],intialState[0])

		global initial_oval
		#if initial_oval is not None:
		#try: 
		#	canvas2.delete(initial_oval)
		#except:
		#	None
		radius = 10*zoom_factor
		#initial_oval = canvas2.create_oval(intialState[0]-radius, intialState[1]-radius, intialState[0]+radius, intialState[1]+radius, outline="red", width=2*zoom_factor)
	
	
		#print cropped
	#https://stackoverflow.com/questions/34324958/trying-to-create-noise-image-with-noise-numpy-and-image
		#img = Image.fromarray(cropped, 'L')

		
		img = Image.fromarray(data2, 'RGBA')
		global image_fromarray
		image_fromarray = img
		#img.save('mymap.png')
		root.image=img
		big = img.resize((map_width*zoom_factor,map_height*zoom_factor))
		#img.show()
		#big.show()
	
		global map_image
		map_image = ImageTk.PhotoImage(img)


		global map_image_factor
		map_image_factor = ImageTk.PhotoImage(big)
		
		canvas2.image = map_image_factor
		canvas2.itemconfig(canvas2_image_widget, image = map_image_factor)
	
	
	
		#refresh path
		global global_path_msg
		if global_path_msg!= None:
			callback_path(global_path_msg)
	
	
	
	
		#print 'pic rdy'
		#global allow_map
		global end_it
		if end_it==False:
			allow_map=True
		#print 'allow map=', allow_map
		#rospy.sleep(0.0)
		#allow_map=True
		return None




		


	
	
	#http://docs.ros.org/jade/api/ros_numpy/html/occupancy__grid_8py_source.html
	#converts_to_numpy(OccupancyGrid)
	def occupancygrid_to_numpy(msg):
		print 'got it!'
		data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
		print	'data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)'
		#return np.ma.array(data, mask=data==-1, fill_value=-1)
		return np.ma.array(data)
	





	def allow_sample():
		global allow_map
		allow_map=True








	def callback_path(msg):
		global end_it
		if end_it==True:
			return
		global mutex_path
		if mutex_path==True:
			return
		mutex_path = True
		global global_path
		global global_path_msg
		global_path_msg = msg
	
		print 'got path!'

			
		for x in global_path:
			canvas2.delete(x)
	
		global_path = list()
		if len( msg.poses)==0:
			mutex_path=False
			return
		print msg.poses[0].pose.position.x
	
		start_x = currentState[0]
		start_y = currentState[1]
		
		counter=len(msg.poses)-2
	
		start_x = int((msg.poses[0].pose.position.x*20+intialState[0])*zoom_factor)
		start_y = int((-msg.poses[0].pose.position.y*20+intialState[1])*zoom_factor)
		#skip first
		iterator_poses = iter(msg.poses)
		next(iterator_poses)
		for t in iterator_poses:
			counter-=1
			x=int((intialState[0]+20*t.pose.position.x)*zoom_factor)
			y=int((intialState[1]+20*-t.pose.position.y)*zoom_factor)
			if counter == 0:
				arrow = canvas2.create_line(start_x,start_y,x,y, tags=("line",), arrow="last", width=2*zoom_factor, fill="green")
			else:
				arrow = canvas2.create_line(start_x,start_y,x,y, tags=("line",), width=2*zoom_factor, fill="green")
			start_x = x
			start_y = y
			global_path.append(arrow)
	
	
			
	
	
	
		#print intialState[0], '+20*', msg.poses[lastone].pose.position.x,'=', x 
		#print intialState[1], '+20*', msg.poses[lastone].pose.position.y,'=', y 
		#print x,y
		#arrow1 = canvas.create_line(currentState[0],currentState[1],x,y, tags=("line",), arrow="last")
	
		mutex_path=False
	
	
	
	
	
	
	
	
	
	
	def callback_odom(msg):
		global end_it
		if end_it==True:
			return
		#print msg.pose.pose
		#print msg.pose.pose.position.x
		global currentState
		global intialState
		#intialState = (initialX,initialY)
		#print 'position:', msg.pose.pose.position.x, msg.pose.pose.position.y
	
		pos_x = float(msg.pose.pose.position.x)
		pos_y = float(msg.pose.pose.position.y)	
	
		angle_in_radians =0.0
	
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			#print trans[0], trans[1]
			#print 'success'
			pos_x = float(trans[0])
			pos_y = float(trans[1])
			#print rot
			roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
			#print roll, pitch, yaw
		
			#print float(yaw/math.pi), ' pi'
			#print float(yaw*180/math.pi), ' degrees'
	
			angle_in_radians = yaw
			
	
	
	
	
			
			
	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			None
		
	
		
	
		currentState=(int((intialState[0]+pos_x*20)*zoom_factor), int((intialState[1]-pos_y*20)*zoom_factor))
		#print currentState
		global current_oval
		canvas2.delete(current_oval)
		radius = 8*zoom_factor
		current_oval = canvas2.create_oval(currentState[0]-radius, currentState[1]-radius, currentState[0]+radius, currentState[1]+radius, outline="blue", width=2*zoom_factor)
		txt = '(',currentState[0],',',currentState[1],')'	
		label_current_pose.config(text=txt)
		
		line_length = 15
		#angle_in_radians
	
		end_x = currentState[0] + line_length * math.cos(angle_in_radians)
		end_y = currentState[1] - line_length * math.sin(angle_in_radians)
	
		canvas2.coords(canvas_arrow, int(currentState[0]),int(currentState[1]),end_x,end_y)
	
	
	


	def reset_gmapping():
		command2_FE()
		command1_FE()
		allow_sample()






	def explore():
		print 'explore'
		getNextPoint()
		

	def moved(event):
		#canvas2.itemconfigure(tag, text="(%r, %r)" % (event.x-intialState[0], event.y-intialState[1]))
		#canvas2.itemconfigure(meter_tag, text="(%r, %r)" % (float(event.x-intialState[0])/20, float(event.y-intialState[1])/20 ) )
		pixel_pos.config(text="(%r, %r)" % (event.x, event.y))
		tag.config(text="(%r, %r)" % (event.x-intialState[0], event.y-intialState[1]))
		meter_tag.config(text="(%r, %r)" % (float(event.x-intialState[0])/20, float(event.y-intialState[1])/20 ) )








	def send_polygon():

		print boundedPolygon


		triplePoints = []

		p1= Point32()
		p2= Point32()
		p3= Point32()
		p4= Point32()
		p5= Point32()
		p6= Point32()

		p1.x=85.5;	p1.y=85.0;	p1.z=0;
		p2.x=85.5;	p2.y=-85.0;	p2.z=0;
		p3.x=-85.5;	p3.y=-85.0;	p3.z=0
		p4.x=-85.5;	p4.y=85.0;	p4.z=0
		p5.x=85.5;	p5.y=85.0;	p5.z=0
		p6.x=0.0;	p6.y=0.0;	p6.z=0;
	
	
		triplePoints.append(p1)
		triplePoints.append(p2)
		triplePoints.append(p3)
		triplePoints.append(p4)
		triplePoints.append(p5)
		triplePoints.append(p6)
	

		for point in triplePoints:	
	
			pt = PointStamped()
			pt.header.stamp = rospy.Time.now()
			pt.header.frame_id = "map"
			pt.point.x = point.x
			pt.point.y = point.y
			pt.point.z = point.z

			print pt

			boundedPolygon.publish(pt)
			rospy.sleep(0.3)




	def start_FE():
		#stop_FE()
		#rospy.sleep(5)
		cmd = "roslaunch husky_navigation exploration.launch"
		print 'cmd = ', cmd
		#outfile = open("outfile", "w")
		#global proc_gazebo
		#proc = subprocess.Popen(cmd, stdout=outfile, stderr=outfile, shell=True, preexec_fn=os.setsid)
		root.proc_FE_server = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
		

	def stop_FE():
		os.killpg(os.getpgid(root.proc_FE_server.pid), signal.SIGTERM)
		for line in root.proc_FE_server.stdout:
			print '#',line
		for line in root.proc_FE_server.stderr:
			print '#',line


	def refresh_FE():
		stop_FE()
		stop()
		start_FE()


	def start_drive(x,y):
		global driving
		driving = True
		drive(x, y) ##update location
		command_drive()	##start loop

	def stop_drive(e):
		print 'stop!'
		global driving
		driving = False
		move_cmd=Twist()
		speed=0.0
		angular=0.0
		move_cmd.linear.x=speed
		move_cmd.angular.z=angular
		cmd_vel.publish(move_cmd)



	def drive(x, y):
		print x,y
		global driving_x, driving_y
		speed= (50.0-y)/50.0
		angular = (50.0-x)/50.0
		driving_x=speed
		driving_y=angular
		
	def command_drive():
		global driving, driving_x, driving_y
		if driving==True:
			print 'inloop'
			
			move_cmd=Twist()
			#speed=0.0
			#angular=-0.5

		
			move_cmd.linear.x=driving_x
			move_cmd.angular.z=driving_y
			cmd_vel.publish(move_cmd)
			#rospy.sleep(2.0)
			root.after(100, command_drive)


	########################################




	run_ROS()
	root.after(5000, None)




	#https://stackoverflow.com/questions/47829049/tkinter-deiconify-method-does-not-draw-window-in-the-same-location-on-linux
	explore_window = tk.Toplevel(root)
	explore_window.title("Explore")
	#explore_window.geometry('1024x600+200+200')
	explore_window.geometry('%dx%d+%d+%d' % (1024, 600, root.winfo_x(), root.winfo_y()))
	#explore_window.overrideredirect(True)
	
	root.explore_window = explore_window

	root.withdraw()

	global map_image
	global map_height
	global map_width

	topframe = tk.Frame(explore_window, borderwidth=3, relief=tk.RIDGE)
	topframe.grid(row=0, column=0)




	label_Explore_Title = tk.Label(topframe, text='Explore')
	label_Explore_Title.grid(row=0,column=0)
	label_Explore_Button = tk.Button(topframe, text = 'back', command=back)
	label_Explore_Button.grid(row=0, column=1)


	reset_Button = tk.Button(topframe, text = 'clear map', command=reset_gmapping)

	reset_Button.grid(row=0, column=3)

	

	explore_Button = tk.Button(topframe, text = 'explore', command=send_polygon)

	explore_Button.grid(row=0, column=4)


	stop_explore_Button = tk.Button(topframe, text = 'stop', command=refresh_FE)

	stop_explore_Button.grid(row=0, column=5)
 	


	default_profile = "images/control3.png"
	image_path = default_profile
	img_control = ImageTk.PhotoImage(Image.open(image_path).resize((100,100)))

	explore_window.image = img_control

	control_canvas = tk.Canvas(topframe, cursor="arrow",  width=100, height=100)
	control_canvas.grid(row=0, column=6)
	control_canvas_widget = control_canvas.create_image(50, 50, image=explore_window.image)

	control_canvas.bind("<Button-1>", lambda e : start_drive(e.x, e.y))
	control_canvas.bind("<B1-Motion>", lambda e : drive(e.x, e.y))
	control_canvas.bind("<ButtonRelease-1>", stop_drive)

	midframe = tk.Frame(explore_window)
	midframe.grid(row=1, column=0)


	
	#width=map_width, height=map_height
	canvas2 = tk.Canvas(midframe, cursor="arrow",  width=860, height=475, borderwidth=10, relief=tk.RIDGE)
	canvas2.grid(row=0, column=0)
	canvas2_image_widget = canvas2.create_image(0, 0, image=map_image, anchor=tk.NW)


	#tag = canvas2.create_text(100, 50, text="", anchor="ne") 
	#meter_tag = canvas2.create_text(100, 50, text="", anchor="se")


	botframe = tk.Frame(explore_window)
	botframe.grid(row=2, column=0)


	pixel_pos = tk.Label(botframe, text="")
	tag = tk.Label(botframe, text="")
	meter_tag = tk.Label(botframe, text="")
	pixel_pos.grid(row=0, column=0)
	tag.grid(row=1, column=0)
	meter_tag.grid(row=2, column=0)

	canvas2.bind("<Motion>", moved)
	canvas2.bind("<ButtonRelease-1>", lambda e, z=zoom_factor : send_checkpoint(int(e.y/z), int(e.x/z), 0))







	global global_path
	global_path=list()

	global global_path_msg
	global_path_msg = None

	global mutex_path
	mutex_path = False


	global initial_oval
	#initial_oval = canvas2.create_oval(20-10, 20-10, 20+10, 20+10, outline="red", width=0)
	global current_oval
	current_oval = canvas2.create_oval(20-10, 20-10, 20+10, 20+10, outline="blue", width=0)

	global canvas_arrow
	canvas_arrow = canvas2.create_line(0,0, 0,0, tags=("line",), arrow="last",  width=2*zoom_factor)





	label_current_pose = tk.Label(botframe, text='(0,0)')
	label_current_pose.grid(row=0, column=1)








	allow_sample()
	global end_it
	end_it=False

	#rospy.init_node('test_node2')
	#costmap_sub = rospy.Subscriber("/explore_server/explore_costmap/costmap", OccupancyGrid, callback_costmap)
	#costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback_costmap)
	map_sub = rospy.Subscriber("/map", OccupancyGrid, callback_map_thread)
	#map_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback_map)
	listener = tf.TransformListener()
	odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, callback_odom)
	path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, callback_path)

	#command1_FE()
	
	start_FE()

	boundedPolygon = rospy.Publisher('clicked_point', PointStamped , queue_size=50)




	#can = tk.Canvas(explore_window)
	#can.pack()




	cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=50)




















#EDIT MAP#######################################################################################################
def editMap():
	points = list(); ovals = list();mylist=[];

	
	def save_editor():
		print 'saving'
		global prompt_cancel
		prompt_cancel = False
		update_canvas_overlays()


		#create regionmap
		createRegionMap()
		#create distance map


		createDistanceMap()



		global map_image
		global map_with_rooms
		map_image = ImageTk.PhotoImage(map_with_rooms)
		#map_image = ImageTk.PhotoImage(img)
		label3.configure(image=map_image)
		label3.image = map_image

		saveMap()


		root.deiconify()
		root.geometry('%dx%d+%d+%d' % (1024, 600, root.edit_window.winfo_x(), root.edit_window.winfo_y()))	
		root.edit_window.destroy()	
	
	
	def cancel_editor():
		global prompt_cancel
		print 'prompt_cancel is ', prompt_cancel

		global rooms_backup
		global rooms

		if (prompt_cancel):
			answer = tkMessageBox.askokcancel("Changes not saved","Are you sure you want to cancel?")
			print answer
			if answer==False:
				return
		
		prompt_cancel = False
		rooms=list()
		for x in rooms_backup:
			room_title  =copy.deepcopy(x[0])
			room_color  =copy.deepcopy(x[1])
			room_type   =copy.deepcopy(x[2])
			image_path  =copy.deepcopy(x[3])
			roomimg     =x[4].copy() 
			regionPixels=copy.deepcopy(x[5]);
			new_room = (room_title, room_color, room_type ,image_path, roomimg, regionPixels)
			rooms.append(new_room)
		#rooms=copy.deepcopy(rooms_backup)
		update_canvas_overlays()
		root.deiconify()
		root.geometry('%dx%d+%d+%d' % (1024, 600, root.edit_window.winfo_x(), root.edit_window.winfo_y()))
		root.edit_window.destroy()
	
	
	def draged(event):
		#global ovals
		canvas.itemconfigure(tag, text="(%r, %r)" % (event.x, event.y))
		if not create_new_mode:
			return
		if (event.x<0)or(event.y<0):
			return
		if (event.x>=map_width) or (event.y >= map_height) or (occMap[event.y,event.x]!=0):
			return
		points.pop(vertex_index)
		oval2 = ovals.pop(vertex_index)
		canvas.delete(oval2[1])
		points.insert(vertex_index, (event.x,event.y))#reversed
		ovals.insert( vertex_index, ( (event.x,event.y) ,(canvas.create_oval(event.x-10, event.y-10, event.x+10, event.y+10, outline="#DDD", width=1)) ))
		#print ' ovals :', len(ovals)
		edit_color = button_color2["bg"]
		global polygon
		if len(points)>=3:
			canvas.delete(polygon)
			polygon = canvas.create_polygon(points, outline='gray', fill=edit_color, width=2, stipple="gray50")
		
	def release(event):
		None
	
	def moved(event):
		canvas.itemconfigure(tag, text="(%r, %r)" % (event.x, event.y))
	
	##Save polygon- ROOM or Accept
	def add_polygon(room_color, room_type):
		
		#accept changes
		global edit_mode
		global color
		global image_path
		global prompt_cancel
		global shader
		prompt_cancel=True
		button_save_exit.config(state='normal')
		button_color2.config(relief=tk.GROOVE)
		label_profile_picture_Image.config(relief=tk.GROOVE)
		button_color2.config(state='disabled')
		if edit_mode:
			
			lb.config(state='normal')
			edit_mode = False
			entry_title.config(state='disabled')
			button_New_Room.config(state='normal')
			button_Edit_Room.config(state='normal')		
			button_DelRoom.config(state='normal')

			button2_MW.config(state='disabled')

			
			menu2.config(state='disabled')
			
			#myroom = (room_title, room_color, room_type, room_image_path, roomimg, regionPixels)
			myroom = rooms[selected_room]

			
			room_type = selectedVar2.get()
			rgb = myroom[1][0]
		
			print rgb
			myroom_image = np.zeros((map_height, map_width, 4), dtype=np.uint8)
			for x in myroom[5]: #regionPixels
				if (occMap[x[0],x[1]]==0):#avoid walls
					myroom_image[x[0],x[1]]=[rgb[0], rgb[1], rgb[2], shader]
			roomimg = Image.fromarray(myroom_image, 'RGBA')
			
			
			#myroom = (room_title, room_color, room_type, room_image_path, roomimg, regionPixels)
			rooms[selected_room] = (title.get(),myroom[1],room_type,image_path, roomimg,myroom[5])
			
			update_canvas_overlays()
			print 'accept changes'
			cancel_button.config(state='normal')
			index = lb.curselection()
			lb.insert(selected_room,title.get())
			lb.delete(selected_room+1)
			
			return
		

		
		button2_MW.config(state='disabled')
		button_New_Room.config(state='normal')
		global create_new_mode
		create_new_mode=False
		canvas.config(cursor="arrow")
		lb.config(state='normal')
		entry_title.config(state='disabled')

		label_profile_picture_Image.config(relief=tk.GROOVE)
		
		#data = np.zeros((len(points), 1, 2), dtype=np.uint8)
		#
		#x=0
		#for y in points:
		#	data[x]=[y[0],y[1]]
		#	x+=1
		
		
		
		#http://matplotlib.1069221.n5.nabble.com/how-to-tell-if-a-point-is-inside-a-polygon-td17339.html
		#p = path.Path([[0,0], [42, 3], [45, 23], [1, 32]])
		p = path.Path(points)
		
		regionPixels=list()
		for row in range(0,map_height):
			for col in range(0,map_width):
				if p.contains_point([col,row]) and occMap[row,col]==0:#avoid walls
					regionPixels.append((row,col)) #REVERSED!
					
		print len(regionPixels), 'pixels selected'
		
		#for x in regionPixels:
		#	map_image[x[0],x[1]]=('#004cff')
		
		myroom = np.zeros((map_height, map_width, 4), dtype=np.uint8)
		print room_color
		rgb = room_color[0]
		
		print rgb
		for x in regionPixels:
			#myroom[x[0],x[1]]=[0,0,170,255]
			myroom[x[0],x[1]]=[rgb[0], rgb[1], rgb[2], shader]
			
		roomimg = Image.fromarray(myroom, 'RGBA')
		roomimg.save('myroom.png')
		#roomimg.show()
		#eyes = Image.open('myroom.png').convert("RGBA")
		#back = Image.open('C:\Users\Nir\OneDrive\Documents\ROS\code\map2png\my.png').convert("RGBA")
		back = root.image #Image.open(root.filename).convert("RGBA")
		#iwork = ImageTk.PhotoImage(file = 'myroom.png')
	
		#mylist.append(canvas.create_image(50,50, image=iwork))
		
		#blended = Image.blend(roomimg, back, alpha=0.5)
		#blended.save("blended.png")
		
		
		#IMPORTANT
		#final= Image.alpha_composite(back, roomimg)
		#final.save("final.png")
		
		#c = Image.alpha_composite(tmp_map_image, roomimg)
		#global map_image
		#map_image = ImageTk.PhotoImage(c)

		#map_image.paste(eyes, (0,0))
		
		print p.contains_point([5,5])
		print p.contains_point([72, 3])
	
		global polygon
		canvas.delete(polygon)
		#global points
		while len(points) > 0 : points.pop()
		print 'deleteing ovals:', len(ovals)
		for x , y in ovals:
			canvas.delete(y)
			
		for x in range(0,len(ovals)):
			ovals.pop()
		print 'remaining ovals:', len(ovals)	
			
			
			
		#SAVE ROOM         [0]rbg, [1]hex
		#room_title, room_color, room_type, room_image_path, room_image, room_pixels
		global room_counter
		if title.get()=="":
			title.set('room #' + str(room_counter))

		room_counter+=1

		
		room_title=title.get()

		myroom = (room_title, room_color, room_type, image_path, roomimg, regionPixels)
		print room_title, room_color, room_type, image_path
		rooms.append(myroom)
		lb.insert(tk.END,room_title)
		
		
		update_canvas_overlays()
		#global color
		color = ( (160, 160, 160), 'gray' )
		button_color2.config(bg=color[1])
		
		menu2.config(state='disabled')
		selectedVar2.set(choices2[0]) ##utility back to no choice
		title.set("")
		cancel_button.config(state='normal')
		
		#final2 = ImageTk.PhotoImage(final)
		#canvas.image = final2
		#canvas.itemconfig(canvas_image_widget, image = final2)
		
	def update_canvas_overlays():
		back = root.image #Image.open(root.filename).convert("RGBA")
		blend = back
		print 'iterating rooms:', len(rooms)
		for x in rooms:
			layer = x[4]
			blend = Image.alpha_composite(blend, layer)
		global map_with_rooms
		map_with_rooms = blend
		tmp = ImageTk.PhotoImage(blend)
		canvas.image = tmp
		canvas.itemconfig(canvas_image_widget, image = tmp)
			
		
		
	#touch on screen	
	def on_button_press(event):
    #    # save mouse drag start position
    #    #self.start_x = self.canvas.canvasx(event.x)
    #    #self.start_y = self.canvas.canvasy(event.y)
	#	
		if not create_new_mode:
			return
		if (event.x<0)or(event.y<0):
			return
		if (event.x>=map_width) or (event.y >= map_height) or (occMap[event.y,event.x]!=0):
			return
			
		
		print 'x',event.x, ' row', event.y
		#global points
		global vertex_index
		#https://stackoverflow.com/questions/5228383/how-do-i-find-the-distance-between-two-points
		dist=100000.0
		for x in points:
			dist = math.hypot(event.y - x[1], event.x - x[0])
			print dist
			if dist <=10:
				##select exisitng oval 
				print 'select'
				vertex_index = points.index(x)
				print vertex_index
				return
		
		vertex_index = len(points)
		points.append((event.x,event.y)) #reversed numpy from tkinter 
		
		print points
		
		edit_color = button_color2["bg"]
		

		ovals.append( ( (event.y,event.x) ,(canvas.create_oval(event.x-10, event.y-10, event.x+10, event.y+10, outline="#DDD", width=1)) )) #, fill="blue"
		global polygon
		if len(points)==3:
			polygon = canvas.create_polygon(points, outline='gray', fill=edit_color, width=2, stipple="gray50")
			button2_MW.config(state='normal')
		elif len(points)>3:
			canvas.delete(polygon)
			polygon = canvas.create_polygon(points, outline='gray', fill=edit_color, width=2, stipple="gray50")

	
    #    # create rectangle if not yet exist
    #    #if not self.rect:
    #    #    if self.out_of_scope == 1:
    #    #        self.rect = self.canvas.create_rectangle(self.x, self.y, 1, 1, outline='blue', fill='yellow') #since it's only created once it always remains at the bottom
	#	
	#	

	
	##select room from listbox
	def onselect(event):
	
		if len(rooms)==0 or create_new_mode or edit_mode:
			return
		w = event.widget
		index = int(w.curselection()[0])
		value = w.get(index)
		print 'You selected item %d: "%s"' % (index, value)
		print lb.curselection()
		button_DelRoom.config(state='normal')
		button_Edit_Room.config(state='normal')
		global selected_room
		selected_room = index
		
		#back = Image.open(root.filename).convert("RGBA")
		back = map_with_rooms
		room_png = rooms[selected_room][4]
		final= Image.alpha_composite(back, room_png)
		final3 = ImageTk.PhotoImage(final)
		canvas.image = final3
		canvas.itemconfig(canvas_image_widget, image = final3)
		#final.show()
		
		title.set(rooms[selected_room][0])
		color = rooms[selected_room][1]
		button_color2.config(bg=color[1])
		
		#change rooms utility
		selectedVar2.set(rooms[selected_room][2])
		
		#change profile picture
		global image_path
		image_path = rooms[selected_room][3]
		tmp_image = ImageTk.PhotoImage(Image.open(image_path).resize((100,100)))
		label_profile_picture_Image.configure(image=tmp_image)
		label_profile_picture_Image.image=tmp_image


		
		
	def OptionMenu2_SelectionEvent(event): # I'm not sure on the arguments here, it works though
		## do something
		print event
		
		
	def change_color():
		global color
		color=askcolor()
		button_color2.config(bg=color[1])
		
		
		if edit_mode:
			global rooms
			global shader
			myroom = rooms[selected_room]
			#myroom = (room_title, room_color, room_type, room_image_path, roomimg, regionPixels)
			myroom_image = np.zeros((map_height, map_width, 4), dtype=np.uint8)
			rgb = color[0]
			for x in myroom[5]:
				#myroom[x[0],x[1]]=[0,0,170,255]
				myroom_image[x[0],x[1]]=[rgb[0], rgb[1], rgb[2], shader]
			roomimg = Image.fromarray(myroom_image, 'RGBA')
			
			rooms[selected_room] = (myroom[0],color,myroom[2],myroom[3],roomimg,myroom[5])
		
		
		#color polygon
		if len(points)>=3:
			global polygon
			canvas.delete(polygon)
			polygon = canvas.create_polygon(points, outline='gray', fill=color[1], width=2, stipple="gray50")
			
		update_canvas_overlays()
		
		
	def edit_room():
		print 'edititng room', selected_room
		lb.config(state='disabled')
		button_save_exit.config(state='disabled')
		global edit_mode
		edit_mode = True
		entry_title.config(state='normal')
		button_New_Room.config(state='disabled')
		button_Edit_Room.config(state='disabled')		
		button_DelRoom.config(state='disabled')
		cancel_button.config(state='disabled')
		button2_MW.config(state='normal')
		button_color2.config(state='normal')
		menu2.config(state='normal')
		button_color2.config(relief=tk.RAISED)
		label_profile_picture_Image.config(relief=tk.RAISED)
		
	def delete_room():
		print 'deleteing room', selected_room
		answer = tkMessageBox.askyesno("Delete Room","Are you sure?")
		print answer
		if answer==False:
			return
		global prompt_cancel
		prompt_cancel=True
		global rooms
		rooms.pop(selected_room)
		lb.delete(selected_room)
		print 'total rooms:', len(rooms)
		update_canvas_overlays()
		button_DelRoom.config(state='disabled')
		button_Edit_Room.config(state='disabled')
		global color
		color = ( (160, 160, 160), 'gray' )
		button_color2.config(bg=color[1])
		selectedVar2.set(choices2[0])
		title.set("")
		#global room_counter
		#room_counter-=1
		change_profile_picture(default_profile)
		button_save_exit.config(state='normal')

	def delete_all_rooms():
		print 'deleteing all rooms'
		answer = tkMessageBox.askyesno("Delete Room","Are you sure?")
		print answer
		if answer==False:
			return
		global prompt_cancel
		prompt_cancel=True
		global rooms
		for x in rooms:
			lb.delete(0)
		rooms = list()
		print 'total rooms:', len(rooms)
		update_canvas_overlays()
		button_DelRoom.config(state='disabled')
		button_Edit_Room.config(state='disabled')
		global color
		color = ( (160, 160, 160), 'gray' )
		button_color2.config(bg=color[1])
		selectedVar2.set(choices2[0])
		title.set("")
		#global room_counter
		#room_counter-=1
		change_profile_picture(default_profile)
		button_save_exit.config(state='normal')




		
	def create_room():
		print 'create room'
		global create_new_mode
		create_new_mode = True
		change_profile_picture(default_profile)
		entry_title.config(state='normal')
		title.set("")
		button_New_Room.config(state='disabled')
		canvas.config(cursor="target")
		lb.config(state='disabled')
		button_DelRoom.config(state='disabled')
		button_Edit_Room.config(state='disabled')
		cancel_button.config(state='disabled')
		button_color2.config(state='normal')
		global color
		color = ( (160, 160, 160), 'gray' )
		button_color2.config(bg=color[1])
		menu2.config(state='normal')
		selectedVar2.set(choices2[0])
		update_canvas_overlays()
		title.set('room #' + str(room_counter))
		button_color2.config(relief=tk.RAISED)
		label_profile_picture_Image.config(relief=tk.RAISED)
		button_save_exit.config(state='disabled')
		
	def change_picture():
		print 'change_picture'
		global image_path
		if not (create_new_mode or edit_mode):
			return
		tmp_path = ""
		tmp_path = tkFileDialog.askopenfilename(title = "Select file",defaultextension='.*', filetypes = (("all files","*.*"),("JPEG files","*.jpg"),("PNG files","*.png")))
		if tmp_path!="":
			change_profile_picture(tmp_path)

	def change_profile_picture(path):
		print 'change_profile_picture', path
		global image_path
		image_path = path
		img4 = ImageTk.PhotoImage(Image.open(image_path).resize((100,100)))
		label_profile_picture_Image.configure(image=img4)
		label_profile_picture_Image.image = img4
	
	
	def autoRooms():
		createFSImap()
		for x in rooms:
			lb.insert(tk.END,x[0])
		update_canvas_overlays()
	
	#def leave_lb(event):
	#	print 'leave'
	#	update_canvas_overlays()
	
	editMapWindow = tk.Toplevel(root)
	editMapWindow.title("Edit Map")
	#editMapWindow.geometry('1024x600+200+200')
	editMapWindow.geometry('%dx%d+%d+%d' % (1024, 600, root.winfo_x(), root.winfo_y()))

	#editMapWindow.protocol("WM_DELETE_WINDOW", None)
	
	root.edit_window = editMapWindow
	
	#editMapWindow.after(1, lambda: editMapWindow.focus_force())
	#https://stackoverflow.com/questions/1406145/how-do-i-get-rid-of-python-tkinter-root-window
	#https://stackoverflow.com/questions/45214662/tkinter-toplevel-always-in-front
	root.withdraw()
	
	
	global color
	color = ( (160, 160, 160), 'gray' )
	
	
	
	endscreen_frame = tk.Frame(editMapWindow)
	endscreen_frame.grid(row=3, column=3)
	
	button_save_exit = tk.Button(endscreen_frame, text='Save & Exit', width=10, command=save_editor)
	button_save_exit.grid(row=0, column=1)
	button_save_exit.config(state='disabled')
	
	
	cancel_button = tk.Button(endscreen_frame, text='Cancel', width=10, command=cancel_editor)
	cancel_button.grid(row=0, column=0)

	

	
	
	frame1=tk.Frame(editMapWindow)
	frame1.grid(row=1, column=3)
	
	#Name, Colour, Type, 
	#http://www.mypythonadventure.com/category/tkinter/
	
	frame2 = tk.Frame(editMapWindow)
	frame2.grid(row=2, column=3)
	lb = tk.Listbox(frame2, height=7)
	#lb.config(state='disabled')
	
	#lb.insert(END,"first entry")
	#lb.insert(END,"second entry")
	#lb.insert(END,"third entry")
	#lb.insert(END,"fourth entry")
	lb.grid(row=0, column=0)
	
	sb = tk.Scrollbar(frame2,orient=tk.VERTICAL)
	sb.grid(row=0, column=1, sticky='ns')
	
	sb.configure(command=lb.yview)
	lb.configure(yscrollcommand=sb.set)
	
	lb.bind('<<ListboxSelect>>', onselect)
	#lb.bind('<Leave>', leave_lb)
	
	
	
	
	room_option_frame = tk.Frame(frame1)
	room_option_frame.grid(row=0, column=0)
	
	
	button_New_Room = tk.Button(room_option_frame, text = 'New', command=create_room)
	button_New_Room.pack(side=tk.LEFT)
	
	button_Edit_Room = tk.Button(room_option_frame, text = 'Edit', command=edit_room)
	button_Edit_Room.pack(side=tk.LEFT)
	button_Edit_Room.config(state='disabled')
	
	button_DelRoom = tk.Button(room_option_frame, text='Delete',  command=delete_room)
	button_DelRoom.pack(side=tk.LEFT)
	button_DelRoom.config(state='disabled')
	

	
	button2_MW = tk.Button(room_option_frame, text='Accept', command= lambda: add_polygon(color, selectedVar2.get()))
	button2_MW.pack(side=tk.LEFT)
	button2_MW.config(state='disabled')
	
	
	
	button7 = tk.Button(room_option_frame, text='Auto',  command=autoRooms)
	button7.pack(side=tk.LEFT)

	
	button8 = tk.Button(room_option_frame, text='delete all',  command=delete_all_rooms)
	button8.pack(side=tk.LEFT)	



	#label4 = tk.Label(editMapWindow,text= "initial location: (0,0)", fg="dark green")
	#label.grid(row=0, column=1)
	
	#label2.configure(image=map_image)
	#label2.image = map_image
	
	
	
	
	
	#label_picture = tk.Label(editMapWindow, text='Image', width=25)
	#label_picture.grid(row=3, column=3)
	
	#button_picture = tk.Button(editMapWindow, text='Change Image', width=25,  command=None)
	#button_picture.grid(row=4, column=3)
	
	
	profile_frame = tk.Frame(editMapWindow, height=2, bd=1, relief=tk.RAISED)
	profile_frame.config(highlightbackground='black')
	profile_frame.grid(row=0, column=0)
	
	title_frame = tk.Frame(editMapWindow)
	title_frame.grid(row=0, column=1)
	
	label_title = tk.Label(title_frame, text='Room Title')
	label_title.grid(row=0, column=0)
	
	title = tk.StringVar()
	entry_title = tk.Entry(title_frame, textvariable=title)
	entry_title.config(state='disabled')
	entry_title.grid(row=0, column=1)
	title.set("")
	s = title.get()
	
	
	global image_path
	global default_profile
	default_profile = "images/noprofile.png"
	image_path = default_profile
	img4 = ImageTk.PhotoImage(Image.open(image_path).resize((100,100)))

	
	
	label_profile_picture_Image = tk.Label(profile_frame, image=img4)
	label_profile_picture_Image.image = img4
	
	
	
	label_profile_picture_Image.bind("<Button-1>",lambda e : change_picture())
	label_profile_picture_Image.config(relief=tk.GROOVE)
	label_profile_picture_Image.pack()
	

	
	#utility_frame = tk.Frame(title_frame)
	#utility_frame.grid(row=1, column=0)
	
	
	label_type = tk.Label(title_frame, text='Utility')
	label_type.grid(row=1, column=0)

	choices2 = []
	choices2.append("not selected")
	choices2.append("Office")
	choices2.append("Class Room")
	choices2.append("Lavoratory")
	
	
	
	selectedVar2 = tk.StringVar()
	selectedVar2.set(choices2[0])
	
	menu2 = tk.OptionMenu(title_frame, selectedVar2, *choices2, command = OptionMenu2_SelectionEvent)
	menu2.config(width=20)
	menu2.config(state='disabled')
	
	menu2.grid(row=1, column=1)
	
	
	label_color = tk.Label(title_frame, text='Room color', width=10)
	
	label_color.grid(row=2, column=0)
	
	
	button_color2 = tk.Button(title_frame, bg=color[1], width=20, relief=tk.GROOVE, command= lambda: change_color())
	button_color2.config(state='disabled')
	button_color2.config(relief=tk.GROOVE)
	button_color2.grid(row=2, column = 1)
	
	
	
	
	
	
	canvas = tk.Canvas(editMapWindow, cursor="arrow",  width=map_width, height=map_height)
	canvas.grid(row=1, column=0, columnspan=3, rowspan=2)
	canvas_image_widget = canvas.create_image(0, 0, image=map_image, anchor=tk.NW)
	
	canvas.bind("<ButtonPress-1>",   on_button_press)
	canvas.bind("<Motion>", moved)
	canvas.bind("<B1-Motion>",  draged)
	canvas.bind("<ButtonRelease-1>", release)
	
	
	
	#canvas.create_polygon(points, outline='gray', fill='gray', width=2)
    #
	#polygon.pack()
	
	tag = canvas.create_text(50, 10, text="", anchor="nw") 
	
	global rooms_backup
	#https://stackoverflow.com/questions/2612802/how-to-clone-or-copy-a-list?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
	#print rooms
	
	rooms_backup=list()
	for x in rooms:
		room_title  =copy.deepcopy(x[0])
		room_color  =copy.deepcopy(x[1])
		room_type   =copy.deepcopy(x[2])
		image_path  =copy.deepcopy(x[3])
		roomimg     =x[4].copy() 
		regionPixels=copy.deepcopy(x[5]);
		new_room = (room_title, room_color, room_type ,image_path, roomimg, regionPixels)
		rooms_backup.append(new_room)
	# = copy.deepcopy(rooms)
	#print rooms
	#print rooms_backup
	global prompt_cancel
	prompt_cancel = False
	
	for x in rooms:
		lb.insert(tk.END,x[0])
		

	update_canvas_overlays()
	####################################################################################################3





















#NAVIGATE#######################################################################################################	
def navigate_window():

	profile_list = list()
	def back():


		global occMap
		global map_image
		global map_width
		global map_height
		global intialState
		global map_data
		global distanceMap

		#occMap={};
		#print 'creating matrix'
		#for j in range(0,map_height):
		#	for i in range(0,map_width):
		#	#array_num=int(j*map_width+i)
		#	#print array_num
		#		occMap[i,j]=map_data[i,j]




		if robot_status == 'online':
			odom_sub.unregister()
			path_sub.unregister()
		else:
			print 'offline'

		stop_ROS()


		#rospy.sleep(3.0)

		root.deiconify()
		root.geometry('%dx%d+%d+%d' % (1024, 600, root.navigate_window.winfo_x(), root.navigate_window.winfo_y()))
		root.navigate_window.destroy()





	def show_rooms(page):
		print 'there are ', len(rooms),' rooms'
		i = page*8
		room_num = i
		for x in rooms[i:i+4]:
		
		#new_room = (room_title, room_color, room_type ,image_path, roomimg, regionPixels)
			this_num = copy.deepcopy(room_num)
			print this_num
			room_frame = tk.Frame(rooms_frame_top, borderwidth=5, relief=tk.RIDGE, pady=5, padx=5)
			room_frame.bind("<Button-1>",lambda event, num=this_num : pressed_room(event, num))
			room_frame.pack(side=tk.LEFT, padx=5, pady=5)
			profile_list.append(room_frame)
	
			room_title = tk.Label(room_frame, text=x[0])
			room_title.pack()


			room_image = ImageTk.PhotoImage(Image.open(x[3]).resize((150,150)))

	
			label_profile_picture_Image = tk.Label(room_frame, image=room_image, relief=tk.RAISED)
			label_profile_picture_Image.image = room_image



			#label_profile_picture_Image.bind("<Button-1>",lambda event, title= x[0] : pressed_room(event, title))
			label_profile_picture_Image.config(relief=tk.RAISED)
			label_profile_picture_Image.pack(padx=5, pady=5)
			room_num += 1




		for x in rooms[i+4:i+8]:
		
		#new_room = (room_title, room_color, room_type ,image_path, roomimg, regionPixels)
			this_num = copy.deepcopy(room_num)
			print this_num
			room_frame = tk.Frame(rooms_frame_bot, borderwidth=5, relief=tk.RIDGE, pady=5, padx=5)
			room_frame.bind("<Button-1>",lambda event, num= this_num : pressed_room(event, num))
			room_frame.pack(side=tk.LEFT, padx=5, pady=5)
			profile_list.append(room_frame)
	
			room_title = tk.Label(room_frame, text=x[0])
			room_title.pack()


			room_image = ImageTk.PhotoImage(Image.open(x[3]).resize((150,150)))

	
			label_profile_picture_Image = tk.Label(room_frame, image=room_image, relief=tk.RAISED)
			label_profile_picture_Image.image = room_image



			#label_profile_picture_Image.bind("<Button-1>",lambda event, title= x[0] : pressed_room(event, title))
			label_profile_picture_Image.config(relief=tk.RAISED)
			label_profile_picture_Image.pack(padx=5, pady=5)

			room_num += 1








	def next_page():
		print 'next'
		global profile_list
		for x in profile_list:
			print 'forget', x
			x.pack_forget()
		global current_page
		current_page+=1
		show_rooms(current_page)

	def prev():
		print 'prev'
		global profile_list
		for x in profile_list:
			print 'forget', x
			x.pack_forget()
		global current_page
		current_page-=1
		show_rooms(current_page)

	def pressed_room(event, i):
		navigate_window.allow_goto = False
		print 'room num:', i 
		print 'title:', rooms[i][0]
		room_region = rooms[i][5]
		print 'rooms num of pixels:', len(room_region)
		#new_room = (room_title, room_color, room_type ,image_path, roomimg, regionPixels)
		
		max =0
		region = regionMap[room_region[0][0],room_region[0][1]]
		print 'region number is:', region
		print 'at ', room_region[0]

		#for x in room_region:
		#	if regionMap[x[0],x[1]]!=region:
		#		print 'WTF'

		tmp_map = np.zeros((map_height, map_width), dtype=np.uint32)
		print 'shape: tmp_map', tmp_map.shape
		print 'region: ', regionMap.shape
		print 'distance ', distanceMap.shape
		tmp_map[regionMap==region]=distanceMap[regionMap==region]

		ind = np.unravel_index(np.argmax(tmp_map, axis=None), tmp_map.shape)
		#max = np.amax(distanceMap[regionMap==region])   
		
		print 'ind ', ind

		#location = (room_region[0][0],room_region[0][1])
		navigate_window.location = (ind[0],ind[1])
		
		#for x in room_region:
		#	value = distMap[x[0],x[1]]
		#	#print distMap[x[0],x[1]]
		#	if value > max:
		#		max=value
		#		location = (x[0],x[1])

		print 'going to ', navigate_window.location[0], navigate_window.location[1]
		#global zoom_factor
		send_checkpoint(navigate_window.location[0], navigate_window.location[1], 0)
		
		#.height



		back = root.image #Image.open(root.filename).convert("RGBA")
		blend = back
		print 'iterating rooms:', len(rooms)
		for x in rooms:
			layer = rooms[i][4]
			blend = Image.alpha_composite(blend, layer)
		global map_with_rooms
		#map_with_rooms = blend
		scale = 0.5
		width = int(map_width*scale)
		height = int(map_height*scale)
		navigate_window.image = ImageTk.PhotoImage(blend.resize((width,height), Image.ANTIALIAS))

		#tmp = ImageTk.PhotoImage(blend)
		
		canvas.image = navigate_window.image
		canvas.itemconfig(canvas_image_widget, image = navigate_window.image)






	def callback_odom(msg):
		#global end_it
		#if end_it==True:
		#	return
		#print msg.pose.pose
		#print msg.pose.pose.position.x
		global currentState
		global intialState
		#intialState = (initialX,initialY)
		#print 'position:', msg.pose.pose.position.x, msg.pose.pose.position.y
	
		pos_x = float(msg.pose.pose.position.x)
		pos_y = float(msg.pose.pose.position.y)	
	
		angle_in_radians =0.0
	
		try:
			(trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
			#print trans[0], trans[1]
			#print 'success'
			pos_x = float(trans[0])
			pos_y = float(trans[1])
			#print rot
			roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
			#print roll, pitch, yaw
		
			#print float(yaw/math.pi), ' pi'
			#print float(yaw*180/math.pi), ' degrees'
	
			angle_in_radians = yaw
			
	
	
	
	
			
			
	
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			None
		
	
		scale = 0.5		
	
		currentState=(intialState[0]+int(pos_x*20),(intialState[1]- int(pos_y*20)))
		scaledCurrentState = (int(currentState[0]*scale),int(currentState[1]*scale))
		#print currentState
		global current_oval
		canvas.delete(navigate_window.current_oval)
		radius = 8



		#width = int(map_width*scale)
		#height = int(map_height*scale)



		navigate_window.current_oval = canvas.create_oval(scaledCurrentState[0]-radius, scaledCurrentState[1]-radius, scaledCurrentState[0]+radius, scaledCurrentState[1]+radius, outline="blue", width=2*scale)
		txt = '(',scaledCurrentState[0],',',scaledCurrentState[1],')'	
		#label_current_pose.config(text=txt)
		
		line_length = int(15*scale)
		#angle_in_radians
	
		end_x = scaledCurrentState[0] + line_length * math.cos(angle_in_radians)
		end_y = scaledCurrentState[1] - line_length * math.sin(angle_in_radians)
	
		canvas.coords(navigate_window.current_arrow, int(scaledCurrentState[0]),int(scaledCurrentState[1]),end_x,end_y)
	
	
	








	def callback_path(msg):
	
		#global end_it
		#if end_it==True:
		#	return
		#global mutex_path
		#if mutex_path==True:
		#	return
		#mutex_path = True
		#global global_path
		#global global_path_msg
		#global_path_msg = msg
	
		#print 'got path!'

			
		for x in navigate_window.global_path:
			canvas.delete(x)
	
		navigate_window.global_path = list()
		if len( msg.poses)==0:
			#mutex_path=False
			return
		print msg.poses[0].pose.position.x
	
		start_x = currentState[0]
		start_y = currentState[1]
		
		counter=len(msg.poses)-2

		scale = 0.5
	
		start_x = int((msg.poses[0].pose.position.x*20+intialState[0])*scale)
		start_y = int((-msg.poses[0].pose.position.y*20+intialState[1])*scale)
		#skip first
		iterator_poses = iter(msg.poses)
		next(iterator_poses)
		for t in iterator_poses:
			counter-=1
			x=int((intialState[0]+20*t.pose.position.x)*scale)
			y=int((intialState[1]+20*-t.pose.position.y)*scale)
			#if counter == 0:
			#	arrow = canvas.create_line(start_x,start_y,x,y, tags=("line",), arrow="last", width=2*zoom_factor, fill="green")
			#else:
			arrow = canvas.create_line(start_x,start_y,x,y, tags=("line",), width=2, fill="green")
			start_x = x
			start_y = y
			navigate_window.global_path.append(arrow)
	
	
			
	
	
	
		#print intialState[0], '+20*', msg.poses[lastone].pose.position.x,'=', x 
		#print intialState[1], '+20*', msg.poses[lastone].pose.position.y,'=', y 
		#print x,y
		#arrow1 = canvas.create_line(currentState[0],currentState[1],x,y, tags=("line",), arrow="last")
	
		#mutex_path=False
	
	

		if (navigate_window.allow_goto==False):
			stop()
			goto_button.config(state='normal')
		else:
			goto_button.config(state='disabled')
	



	def ready_goto():
		navigate_window.allow_goto = True
		send_checkpoint(navigate_window.location[0], navigate_window.location[1], 0)
		goto_button.config(state='disabled')



	run_ROS()
	

	navigate_window = tk.Toplevel(root)
	navigate_window.title("Navigate")
	navigate_window.geometry('%dx%d+%d+%d' % (1024, 600, root.winfo_x(), root.winfo_y()))
	navigate_window.grid_columnconfigure(4, weight=1)
	navigate_window.grid_rowconfigure(4, weight=1)

	
	root.navigate_window = navigate_window

	root.withdraw()

	global map_image
	global map_height
	global map_width
	global profile_list



	label_Navigate_Title = tk.Label(navigate_window, text='Navigate')
	label_Navigate_Title.grid(row=0,column=0)
	label_Navigate_Button = tk.Button(navigate_window, text = 'back', command=back)
	label_Navigate_Button.grid(row=0, column=1)



	label_Navigate_Button = tk.Button(navigate_window, text = 'prev', command=prev)
	label_Navigate_Button.grid(row=0, column=2)


	label_Navigate_Button = tk.Button(navigate_window, text = 'next', command=next_page)
	label_Navigate_Button.grid(row=0, column=3)




	rooms_frame_top = tk.Frame(navigate_window)
	rooms_frame_top.grid(row=1,column=0, columnspan=3, sticky=tk.W)

	rooms_frame_bot = tk.Frame(navigate_window)
	rooms_frame_bot.grid(row=2,column=0, columnspan=3, sticky=tk.W)



	pages = len(rooms)//8
	print 'num of pages', pages
	
	global current_page
	current_page = 0
	show_rooms(current_page)


	#detail frame w/ map

	detail_frame = tk.Frame(navigate_window, borderwidth=3, relief=tk.RIDGE)
	detail_frame.grid(row=0, rowspan=5, column = 4, columnspan = 2, sticky=(tk.N + tk.S + tk.E + tk.W))
	myLabel = tk.Label(detail_frame,text= "robot status ")
	myLabel.pack()

	#default_profile = "images/control3.png"
	#image_path = default_profile
	#global img_small
	


	global default_profile
	default_profile = "images/noprofile.png"
	#image_path = default_profile
	img4 = ImageTk.PhotoImage(Image.open(default_profile).resize((100,100)))

	
	
	label_profile_picture_Image = tk.Label(detail_frame, image=img4)
	label_profile_picture_Image.image = img4
	label_profile_picture_Image.pack()



	myText = tk.Text(detail_frame, width = 100, height = 17)
	myText.insert(tk.END, "Lorem ipsum dolor sit amet, consectetur adipiscing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.")
	myText.config(state='disabled')
	myText.pack()




	scale = 0.5
	width = int(map_width*scale)
	height = int(map_height*scale)
	navigate_window.image = ImageTk.PhotoImage(root.image.resize((width,height), Image.ANTIALIAS))


	canvas = tk.Canvas(detail_frame,   width=width, height=height)
	canvas.pack()
	canvas_image_widget = canvas.create_image(0, 0, image=navigate_window.image, anchor=tk.NW)
	
		

	#canvas.bind("<ButtonPress-1>",   on_button_press)
	#canvas.bind("<Motion>", moved)
	#canvas.bind("<B1-Motion>",  draged)
	#canvas.bind("<ButtonRelease-1>", release)


	radius = int(10*scale)
	y = int(intialState[0]*scale)
	x = int(intialState[1]*scale)
	#initial_oval = canvas.create_oval(y-radius, x-radius, y+radius, x+radius, outline="red", width=2)



	#global current_oval
	navigate_window.current_oval = canvas.create_oval(20-10, 20-10, 20+10, 20+10, outline="blue", width=0)

	#global canvas_arrow
	navigate_window.current_arrow = canvas.create_line(0,0, 0,0, tags=("line",), arrow="last",  width=2)



	navigate_window.allow_goto = False
	navigate_window.location = (0.0,0.0)

	
	goto_button = tk.Button(detail_frame, text='goto', width=25,  command=ready_goto)
	goto_button.config(state='disabled')
	goto_button.pack()


	listener = tf.TransformListener
	odom_sub = rospy.Subscriber
	path_sub = rospy.Subscriber

	navigate_window.global_path = list()

	if robot_status == 'online':
		None
		listener = tf.TransformListener()
		odom_sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, callback_odom)
		path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, callback_path)
	else:
		print 'offline'




	





			
start_time = datetime.datetime.now()	
boot_time = datetime.datetime.now()
root = tk.Tk()
root.title("GUI")

frame1=tk.Frame(root)
frame1.pack(side=tk.TOP,fill=tk.X)

label = tk.Label(frame1,text= "robot status \n initial location: (0,0)", fg="dark red") #fg="dark green"
label.grid(row=0,column=2)




#button1 = tk.Button(frame1, text='Run ROS Core', width=25, command=run_ROS)
#button1.grid(row=1, column=0)
#button1.config(state='disabled')


#localization

label = tk.Label(frame1, text='Simulator', width=25)
label.grid(row=1, column=2)

#button_stop_ROS = tk.Button(frame1, text='Stop ROS Core', width=25, command=stop_ROS)
#button_stop_ROS.grid(row=1, column=3)
#button_stop_ROS.config(state='disabled')

choices = []
choices.append("off")
choices.append("Gazebo")
#choices.append("")



selectedVar = tk.StringVar()
selectedVar.set(choices[0])

menu = tk.OptionMenu(frame1, selectedVar, *choices, command = OptionMenu_SelectionEvent)
menu.config(width=25)


menu.grid(row=1, column=3)



label_URDF = tk.Label(frame1, text='URDF', width=25)
label_URDF.grid(row=1, column=0)





choices_URDF = []
choices_URDF.append("ARMadillo")
choices_URDF.append("Huskey")
choices_URDF.append("Turtlebot")



selectedVar_URDF = tk.StringVar()
selectedVar_URDF.set(choices_URDF[0])

menu_URDF = tk.OptionMenu(frame1, selectedVar_URDF, *choices_URDF, command = None)
menu_URDF.config(width=25)


menu_URDF.grid(row=1, column=1)










label_world = tk.Label(frame1, text='World', width=25)
label_world.grid(row=2, column=2)



choices_world = []
choices_world.append("empty world")
#choices_world.append("Building")
#choices_world.append("Testing room")



for file in os.listdir("/home/zeged/Documents/gui/worlds"):
	if "{}".format(file).rsplit('.')[0] not in choices_world:
		choices_world.append("{}".format(file))




selectedVar2 = tk.StringVar()
selectedVar2.set(choices_world[0])

menu2 = tk.OptionMenu(frame1, selectedVar2, *choices_world, command = changeWorld)
menu2.config(width=25)
menu2.config(state='disabled')


menu2.grid(row=2, column=3)




label_map = tk.Label(frame1, text='Map', width=25)
label_map.grid(row=2, column=0)


global choices_map
choices_map = []
choices_map.append("new map")
#choices_world.append("Building")
#choices_world.append("Testing room")



for file in os.listdir("/home/zeged/Documents/gui/maps"):
	if "{}".format(file).rsplit('.')[1] == "pgm":
		choices_map.append("{}".format(file).rsplit('.')[0])



global selectedVar3
selectedVar3 = tk.StringVar()
selectedVar3.set(choices_map[0])

global menu3
menu3 = tk.OptionMenu(frame1, selectedVar3, *choices_map, command = changeMap)
menu3.config(width=25)


menu3.grid(row=2, column=1)








button2 = tk.Button(frame1, text='Launch explorer', width=25, command=explore_window)
button2.grid(row=4,column=0, columnspan=2)

#button3 = tk.Button(frame1, text='Load Map', width=25, command=loadMap)
#button3.grid(row=3,column=0)


label_rooms = tk.Label(frame1, text='Rooms editor', width=25)
label_rooms.grid(row=3, column=0)




button4 = tk.Button(frame1, text='Edit rooms', width=25, command=editMap)
button4.grid(row=3, column=1)
button4.config(state='disabled')

#button5 = tk.Button(frame1, text='Save Map', width=25, command=saveMap)
#button5.grid(row=3, column=2)


button6 = tk.Button(frame1, text='Launch navigator', width=25, command=navigate_window)
button6.grid(row=5,column=0, columnspan=2)
button6.config(state='disabled')



#image1 = PhotoImage(file="images\logo.png")

img = ImageTk.PhotoImage(Image.open("images/logo.png").resize((288,76)))


label2 = tk.Label(frame1,image=img)
label2.image = img
label2.grid(row=0,columnspan=2)


map1 = Image.open("images/nomap.png")

small_map = map1.resize((150,150)) #better switch to fill
root.image = small_map
map2 = ImageTk.PhotoImage(small_map)

#global map_image
map_image = map2

global no_map_image
no_map_image = map2

#global cost_map_image
#cost_map_image = map2

#global map_height
map_height=map_image.height()
#global map_width
map_width = map_image.width() 






label3 = tk.Label(frame1,image=map2)
label3.image = small_map
label3.grid(row=6,column=2, rowspan=2, columnspan=2)



status = tk.Label(root, text = 'offline.', bd=1, relief=tk.SUNKEN, anchor=tk.W)
status.pack(side=tk.BOTTOM, fill=tk.X)

#print root.cget("bg")


screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# calculate position x and y coordinates
x = (screen_width/2) - (1024/2)
y = (screen_height/2) - (600/2)
root.geometry('%dx%d+%d+%d' % (1024, 600, x, y))

#test()

root.mainloop()










