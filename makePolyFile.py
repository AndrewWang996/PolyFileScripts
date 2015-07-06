# -*- coding: utf-8 -*-
'''
Written by: Andy Wang
Date: June 23rd 2015
Email: andrew.wang@inria.fr

Use this Python program to create a couches5.poly file with
arbitrary width + depth, with arbitrary interface between 
water layer (acoustic) and wet sand layer (elastic).

Instructions: 
(1) Change the parameters below to what is desired
(2) Enter command 'python makePolyFile.py' into terminal
'''

### Dimensions
sizeX = 3000
sizeY = 3000

### Function that represent the interface between water layer + wet sand layer
### Let function be 0 or any constant if flat interface is desired.
mountainFunc = lambda x: sqrt(x/10.)*(sqrt(cos(x/2.)*cos(x/2.))-cos(x/2.)+0.8*cos(2.*x/2.)+0.6*cos(4.*x/2.)+0.4*cos(8.*x/2.))
#mountainFunc = lambda x: exp(1.1*x) * (math.sin(4.*x) + 0.7*math.sin(8.*x) + 0.5*math.sin(16.*x))
#mountainFunc = None       # no interface
#mountainFunc = 0          # flat interface
#mountainFunc = lambda x: abs(x) - 1

### Range of the function that defines the interface.
### For example, if our function is sin(x) and our range is [0,pi] we get the positive section of sin(x)
### If our range is [end1, end2] = [pi,2*pi] we get the negative section
###  |   ___
###  |  /   \pi   2*pi  
###  |-/-----\-----/---------------------> x
###  | 0      \___/
###  |
end1, end2 = 0, 10

### The desired value (maxY - minY) of the function that defines the interface.
###     --note: may differ from traditional definitions of amplitude
### For example if our function is sin(x) and our range is [0,2*pi]:
###  1|   ___
###   |  /   \     2*pi  
###   |-/-----\-----/---------------------> x
###   | 0      \___/
### -1|
###      and we let desiredAmplitude be 8, the interface is then:
###  4|   ___
###   |  /   \     2*pi  
###   |-/-----\-----/---------------------> x
###   | 0      \___/
### -4|
desiredAmplitude = sizeY / 25.
 
### The number of points required to define the interface.
### Increase this in order to get better resolution
nSteps = 100

### For example, if our function is |x| on [-1,1]:

### Number of repeats of the range
###  |\  /
###  |_\/__________> x
###  |  
### and we have nSegs = 3, then our interface will look like this:
###  |\  /\  /\  /
###  |_\/__\/__\/______> x
###  |     
nSegs = 1

### where to write the poly file
writePath = './couches5.poly'



baseString = """
#VERTICES
55 2 0 1
 
 -5  22 -1 0
 -4  24 -2.2 0 
 -3  26 -3 0
 -2  28 -4.2 0   
 -1  30 -5 3
 0   0 -5 3

#1   0 0 3  
#2   30 0 3 
 3   0 9 4
 4   0 15 1
 5   30 15 1
 6   30 9 2
 7   30 3 2
 8   30 2 2
 9   30 1 2

# 10  0.5 0.2 0
# 11  1 0.4 0
# 12  1.5 0.6 0
# 13  2 0.8 0
# 14  2.5 1 0
# 15  3 1.2 0
# 16  3.5 1.4 0
# 17  4 1.6 0
# 18  4.5 1.8 0
# 19  5 2 0
 20  5.5 3 0 
 21  6 4 0
 22  6.5 4.5 0
 23  7 5 0
 24  7.5 5.5 0
 25  8 5.8 0
 26  8.5 6 0
 27  9 6.2 0
 28  9.5 6.4 0
 29  10 6.5 0

 30  10.5 6.4 0
 31  11 6.2 0
 32  11.5 6 0
 33  12 5.8 0
 34  12.5 5.5 0
 35  13 5 0
 36  13.5 4.5 0
 37  14 4 0
 38  14.5 3 0
 39  15 2 0

 40  15.5 1.5 0
 41  16.5 1.2 0
 42  17.5 1 0
 43  18 0.8 0
 44  18.5 0.6 0
 45  19 0.4 0
 46  19.5 0.2 0
 47  20 0 0
 
 48 15.5 2.3 0
 49 16 2.4 0
 50 16.5 2.1 0
 51 17 2 0
 
 52 14.5 4.5 0
 53 15 4.8 0
 54 15.5 4.8 0
 55 16 4.5 0
 56 17 4 0 

 57 12 6 0
 58 12.5 6.1 0
 59 13 6.1 0
 60 13.5 6 0
 61 14 5.9 0
 
#EDGES
60 1
 -8 0 20 0
 -7 -5 47 0
 -6 -4 -5 0
 -5 -3 -4 0
 -4 -2 -3 0
 -3 -1 -2 0
#-2 -1 2 3
 -1 0 -1 3
#0  1 0 3

 0 0 3 4
 1 -1 9 2

#1 1 3 3
 2 3 4 4
 3 4 5 1
 4 5 6 2
 5 6 7 2
 6 7 8 2
 7 8 9 2
#8 9 2 3
#9 1 10 0
#10 10 11 0
#11 11 12 0
#12 12 13 0
#13 13 14 0
#14 14 15 0
#15 15 16 0
#16 16 17 0
#17 17 18 0
#18 18 19 0
#19 19 20 0
 20 20 21 0
 21 21 22 0
 22 22 23 0
 23 23 24 0
 24 24 25 0
 25 25 26 0
 26 26 27 0
 27 27 28 0
 28 28 29 0
 29 29 30 0
 30 30 31 0
 31 31 32 0
 32 32 33 0
 33 33 34 0
 34 34 35 0
 35 35 36 0
 36 36 37 0
 37 37 38 0
 38 38 39 0
 39 39 40 0
 40 40 41 0
 41 41 42 0
 42 42 43 0
 43 43 44 0
 44 44 45 0
 45 45 46 0
 46 46 47 0
#47 47 2 3
#48 1 47 3

 49 3 6 0

 50 39 48 0
 51 48 49 0
 52 49 50 0
 53 50 51 0 
 54 51 9 0

 55 37 52 0
 56 52 53 0
 57 53 54 0
 58 54 55 0 
 59 55 56 0
 60 56 8 0 

 61 32 57 0
 62 57 58 0
 63 58 59 0
 64 59 60 0
 65 60 61 0
 66 61 7 0


#HOLES
0

#REGIONS
6
1 29 14 1 2
2 29 0.1 2 1
3 29 1.2 3 2
4 29 2.2 4 1
5 29 6 5 2
6 10 6 6 1
"""
import re
import sys
from math import sqrt, cos, sin, exp
import math
from copy import deepcopy as COPY

interfaceTag = 10

def parseBaseString():
	'''
	parsing the text from string

	valuesVert is a list containing the vertices
	valuesEdge is a list containing the edges
	valuesHole is a list containing the holes (in this case, assuming no holes exist)
	valuesRegion is a list containing the points marking the separate regions 
									+ the upper bound on the area of the mesh triangles in each region
	'''
	global baseString
	string = baseString.split("\n")
	string = [tuple(re.split('\s+', x.strip())) for x in string if len(x.strip()) > 0 and x[0] != '#']
	 
	sVert = string[0]
	nVert = int(sVert[0])
	stringVert = string[1:nVert+1]

	sEdge = string[1+nVert]
	nEdge = int(sEdge[0])
	stringEdge = string[nVert+2:nEdge+nVert+2]

	sHole = string[nEdge+nVert+2]
	nHole = int(sHole[0])
	stringHole = string[nEdge+nVert+3:nHole+nEdge+nVert+3]

	sRegion = string[nHole+nEdge+nVert+3]
	nRegion = int(sRegion[0])
	stringRegion = string[nHole+nEdge+nVert+4:nRegion+nHole+nEdge+nVert+4]

	'''
	in more detail: reindexing arrays
	'''
	mountA, mountB, mountH = None, None, None
	getVertInd = dict()
	valuesVert = []
	for i in range(len(stringVert)):
		n,x,y,bc = stringVert[i]
		getVertInd[n] = i+1
		if int(n) == 3:
			mountA = i+1
			mountH = float(y)
		if int(n) == 6:
			mountB = i+1
			mountH = float(y)
		valuesVert.append( (float(x),float(y),int(bc)) )
	valuesVert = [valuesVert[i] if not i+1 in (mountA, mountB) else valuesVert[i][:2] + (interfaceTag,) for i in range(len(valuesVert))]
 
	valuesEdge = []
	for n,v1,v2,bc in stringEdge:
		valuesEdge.append((getVertInd[v1],getVertInd[v2],int(bc)))

	#assuming no holes created
	valuesHole = []

	valuesRegion = []
	for n,x,y,t,a in stringRegion:
		valuesRegion.append( (float(x),float(y),int(t),float(a)) )

	return (sVert, sEdge, sHole, sRegion, valuesVert, valuesEdge, valuesHole, valuesRegion, mountA, mountB, mountH)


def resize(values, valuesVert, sX=sizeX, sY=sizeY):
	'''
	resizes a list values according to sizeX, sizeY
	'''
	maxX, minX = max(x for (x,y,bc) in valuesVert), min(x for (x,y,bc) in valuesVert)
	maxY, minY = max(y for (x,y,bc) in valuesVert), min(y for (x,y,bc) in valuesVert)
	multX = float(sX) / (maxX - minX)
	multY = float(sY) / (maxY - minY)

	if type(values) is float:
		return multY * (values - minY)

	return [(multX * (tup[0] - minX), multY * (tup[1] - minY)) + tuple(tup[2:]) for tup in values]


def resizeRegions(valuesRegion, estimateNumTriangles=10000, sX=sizeX, sY=sizeY):
	'''
	resizes the upper bound on the area of the mesh triangles in each region
	in order to obtain the Estimated Number of Triangles
	'''
	maxArea = sum(a for x,y,t,a in valuesRegion)
	desiredArea = sX * sY / float(estimateNumTriangles) * 10
	mult = desiredArea / maxArea
	return [(x,y,t, a * mult) for x,y,t,a in valuesRegion]


def insertAndDelete(position, value, values):
	'''
	This is a helper method for addMountain
	inserts the element value into the list values at index position
	'''
	position -= 1
	return values[:position] + [value] + values[position+1:]

def addMountain(valuesVert, valuesEdge, mountA, mountB, mountH, function, end1, end2, desiredAmplitude, nSteps, nSegs):
	'''
	adds the interface which hopefully looks like some small mountains
	'''
	# If function is bad :: if the function is constant, or reaches a discontinuity
	try:
		function(0)
	except:
		# Give the interface a tag of 10
		valuesEdgeP = [t if t != (mountA, mountB, 0) else (mountA, mountB, interfaceTag) for t in valuesEdge]
		return valuesVert, valuesEdgeP

	#remove line
	valuesEdgeP = [t for t in valuesEdge if t != (mountA,mountB,0) ]

	if function == None:
		return valuesVert, valuesEdgeP

	n = int(round(float(nSteps) / nSegs))		
	heights = [function(end1 + float(end2 - end1) * x / n) for x in range(n + 1)]
	amplitude = max(heights) - min(heights)

	heights = [mountH + x * float(desiredAmplitude) / amplitude for x in heights]
#	heights = heights[::-1] + heights
	heights *= nSegs

	H = len(heights)
	segLength = float(sizeX) / (H - 1)
	values = [(i * segLength, heights[i], interfaceTag) for i in range(H)]
	
	valuesVertP = COPY(valuesVert)
	valuesVertP = insertAndDelete(mountA, values[0], valuesVertP)
	valuesVertP = insertAndDelete(mountB, values[-1], valuesVertP)
	values = values[1:-1]
	V = len(values)
	
	#change valuesVert + valuesEdge
	L = len(valuesVertP)
	valuesVertP.extend( values )
	valuesEdgeP.extend( [(mountA, L+1, interfaceTag), (L+V, mountB, interfaceTag)] )
	valuesEdgeP.extend( (L+i, L+i+1, interfaceTag) for i in range(1, V) )
	return valuesVertP, valuesEdgeP



'''
These 3 methods process the arrays to output in a .poly format
https://www.cs.cmu.edu/~quake/triangle.poly.html
'''
def addIndices(values):
	return [(i+1,) + values[i] for i in range(len(values))]

def formatSection(s, values):
	sp = (len(values),) + s[1:]
	return [sp] + addIndices(values)

def toFinalStr(tup):
	return " ".join(map(str,tup))

def toFinalArray(values):
	return map(toFinalStr, values)






def create(sX=sizeX, sY=sizeY, mF=mountainFunc, e1=end1, e2=end2, dA=desiredAmplitude, nSt=nSteps, nSe=nSegs):
	'''
### THIS IS THE MAIN METHOD
### if other modules import this module, they can run this module by running this method.
### returns the output as a string
	'''

#   Parse text
	sVert, sEdge, sHole, sRegion, valuesVert, valuesEdge, valuesHole, valuesRegion, mountA, mountB, mountH = parseBaseString()
	
	
#   resize the entire mesh to desired sizeX, sizeY by linearly transforming all the points
	mountH = resize(mountH, valuesVert, sX, sY)
	valuesRegion = resize(valuesRegion, valuesVert, sX, sY)
	valuesVert = resize(valuesVert, valuesVert, sX, sY)

	valuesRegion = resizeRegions(valuesRegion, sX=sX, sY=sY)


#   add the interface between the water and the wet sand
	valuesVert, valuesEdge = addMountain(valuesVert, valuesEdge, mountA, mountB, mountH, mF, e1, e2, dA, nSt, nSe)


#	process the data for output in the form of a .poly file
#	structure of a .poly file:
#	https://www.cs.cmu.edu/~quake/triangle.poly.html
	finalArray = formatSection(sVert, valuesVert) + \
				 formatSection(sEdge, valuesEdge) + \
				 formatSection(sHole, valuesHole) + \
				 formatSection(sRegion, valuesRegion)

	finalArray = toFinalArray(finalArray)
	return "\n".join(finalArray) + "\n"



def write(s):
	f = open(writePath, 'w')
	f.write(s)
	f.close()


write(create())





