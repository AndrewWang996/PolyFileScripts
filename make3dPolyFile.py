# -*- coding: utf-8 -*-
'''
Written by: Andy Wang
Date: July 6th 2015
Email: andrew.wang@inria.fr

Use this Python program to create a couches3D.poly file with
arbitrary width + depth, with arbitrary interface between 
water layer (acoustic) and wet sand layer (elastic).

Instructions: 
(1) Change the parameters below to what is desired
(2) Enter command 'python make3dPolyFile.py' into terminal
'''
# import + run the 2D makePolyFile.py
import makePolyFile
from makePolyFile import interfaceTag
from copy import deepcopy as COPY
from math import sqrt, atan2, pi, exp, sin
import math
import re
import sys

### Dimensions
sizeX = 3000
sizeY = 3000
sizeZ = 3000

### Function that represent the interface between water layer + wet sand layer
### Let function be None if flat interface is desired.
mountainFunc = lambda x, y: sin(x-y) * sin(x+y)

### Range of the function that defines the interface.
### For example, if our function is sin(x) and our range is [0,pi] we get the positive section of sin(x)
### If our range is [end1, end2] = [pi,2*pi] we get the negative section
###  |   ___
###  |  /   \pi   2*pi  
###  |-/-----\-----/---------------------> x
###  | 0      \___/
###  |
mountEnd1, mountEnd2 = (-3,-3), (3,3)

### The desired value (maxY - minY) of the function that defines the interface.
###     --note: may differ from traditional definitions of amplitude
### For example, if our function is sin(x) and our range is [0,2*pi]:
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
desiredAmplitude = sizeZ / 10.

### The number of points required to define the interface.
### Increase this in order to get better resolution.
nStepsX, nStepsY = 10, 10

### Number of repeats of the range
### For example, if our function is |x| on [-1,1]:
###  |\  /
###  |_\/__________> x
###  |  
### and we have nSegs = 3, then our interface will look like this:
###  |\  /\  /\  /
###  |_\/__\/__\/______> x
###  |     
nSegsX, nSegsY = 1, 1

### Function that diminishes to zero
### Describes the growth of the salt dome
dimFunc = lambda x: exp(-x*x)
#dimFunc = lambda x: sqrt(6-x)
#dimFunc = lambda x: 1

dimEnd1, dimEnd2 = -0.5,0.5

### where to write the poly file
writePath = './couches3D.poly'



def parseString(baseString):
	'''
	parsing the text from string to create lists of data [in 2D initial yStep]

	valuesVert is a list containing the vertices
	valuesEdge is a list containing the edges
	valuesHole is a list containing the holes (in this case, assuming no holes exist)
	valuesRegion is a list containing the points marking the separate regions 
									+ the upper bound on the area of the mesh triangles in each region
	'''
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

	getVertInd = dict()
	valuesVert = []
	for i in range(len(stringVert)):
		n,x,y,bc = stringVert[i]
		getVertInd[n] = i+1
		valuesVert.append( (float(x),float(y),int(bc)) )
 
	valuesEdge = []
	for n,v1,v2,bc in stringEdge:
		valuesEdge.append((getVertInd[v1],getVertInd[v2],int(bc)))

	#assuming no holes created
	valuesHole = []

	valuesRegion = []
	for n,x,y,t,a in stringRegion:
		valuesRegion.append( (float(x),float(y),int(t),float(a)) )

	return (sVert, sEdge, sHole, sRegion, valuesVert, valuesEdge, valuesHole, valuesRegion)

def properAngle(pA, pB):
	return atan2(pB[1] - pA[1], pB[0] - pA[0])

def getAngle(valuesVert, prev, curr, next):
	pPrev = valuesVert[prev]
	pCurr = valuesVert[curr]
	pNext = valuesVert[next]
	return ( properAngle(pCurr, pNext) - properAngle(pCurr, pPrev) ) % (2 * pi)

def getPolygon(valuesVert, neighbors, prev, curr):
	'''
	helper method for "polygonize" method
	'''
	visited = []
	while not curr in visited:
		visited.append(curr)
		neigh = COPY(neighbors[curr])
		neigh.remove(prev)
		if len(neigh) == 1:
			prev, curr = curr, neigh.pop()
		elif len(neigh) == 2:
			nA, nB = neigh.pop(), neigh.pop()
			if getAngle(valuesVert, prev, curr, nA) > getAngle(valuesVert, prev, curr, nB):
				prev, curr = curr, nA
			else:
				prev, curr = curr, nB
	return (len(visited),) + tuple(map(lambda i: i+1, visited))


def polygonize(valuesVert, valuesEdge, sX=sizeX):
	'''
	returns list of planes that cuts the 2D plane into its different regions
	the corners of each plane are returned in counterclockwise order
	'''
	neighbors = [set() for i in range(len(valuesVert))]
	for (u, v, bc) in valuesEdge:
		neighbors[u-1].add(v-1)
		neighbors[v-1].add(u-1)

	rightSide = []
	botSide = []
	for i in range(len(valuesVert)):
		x,z,bc = valuesVert[i]
		if abs(sX - x) < 1e-10:
			rightSide.append(i)
		if abs(z) < 1e-10:
			botSide.append(i)

	rightSide.sort(key=lambda i: valuesVert[i][1])	# has 6 elements
	botSide.sort(key=lambda i: valuesVert[i][0])		# has 2 elements

	valuesPlaneSide = []
	for i in range(len(botSide) - 1):
		valuesPlaneSide.append( getPolygon(valuesVert, neighbors, botSide[i], botSide[i+1]) )

	for i in range(len(rightSide) - 1):
		valuesPlaneSide.append( getPolygon(valuesVert, neighbors, rightSide[i], rightSide[i+1]) )

	return valuesPlaneSide



def makeInterface(valuesVert, mountainFunc=mountainFunc, nStepsX=nStepsX, nStepsY=nStepsY, mountEnd1=mountEnd1, mountEnd2=mountEnd2):
	'''
	adds the interface which hopefully looks like some small mountains
	'''
	interface2D = filter(lambda (x,z,bc): bc == interfaceTag, valuesVert)
	mountH = sum(z for (x,z,bc) in interface2D) / float(len(interface2D))

	interface = [[0] * (nStepsY + 1) for step in range(nStepsX + 1)]
	for stepX in range(nStepsX + 1):
		for stepY in range(nStepsY + 1):
			interface[stepX][stepY] = mountainFunc(stepX / float(nStepsX) * (mountEnd2[0] - mountEnd1[0]) + mountEnd1[0],
																					   stepY / float(nStepsY) * (mountEnd2[1] - mountEnd1[1]) + mountEnd1[1])
	amplitude = float(max(max(e) for e in interface) - min(min(e) for e in interface))
	interface = [[i * desiredAmplitude / amplitude for i in j] for j in interface]
	
	average = sum(sum(e) for e in interface) / float((nStepsX+1) * (nStepsY+1))
	interface = [[i - average + mountH for i in j] for j in interface]
	
	def reshapeInitialInterface((x,z,bc)):
		'''
		helper method for makeInterface
		'''
		if bc == interfaceTag:
			stepX = int(0.5 + float(x) / sizeX * nStepsX)
			return (x, interface[stepX][0], bc)
		return (x,z,bc)

	return map(reshapeInitialInterface, valuesVert), interface


def VertEdgeTo3D(valuesVert, valuesEdge, interface, sizeX=sizeX, dimFunc=dimFunc, dE1=dimEnd1, dE2=dimEnd2):
	'''
	Starting with the 2D vertices and edges, create the vertices and planes in 3D
	'''
	dim = [dimFunc(dE1 + (dE2 - dE1) * step / float(nStepsY)) for step in range(nStepsY + 1)]
	maxDim = float(max(dim))
	dim = [e / maxDim for e in dim]

	print dim

	v3vert = []
	valuesVertP = COPY(valuesVert)
	
	for step in range(nStepsY + 1):
		vVP = COPY(valuesVertP)
		for i in range(len(valuesVertP)):
			(x,z,bc) = valuesVertP[i]
			if bc in (0,2):
				vVP[i] = (x, valuesVert[i][1] * dim[step], bc)
			if bc == interfaceTag:
				stepX = int(0.5 + float(x) / sizeX * nStepsX)
				stepY = step
				vVP[i] = (x, interface[stepX][stepY], bc)
		valuesVertP = vVP
		v3vert.append(valuesVertP)

	valuesVert3D = []
	dY = float(sizeY) / nStepsY
	for yStep in range(nStepsY + 1):
		for (x, z, bc) in v3vert[yStep]:
			valuesVert3D.append((x, yStep * dY, z, bc))

	valuesPlane = []
	L = len(valuesVert)
	for (u, v, bc) in valuesEdge:
		for yStep in range(nStepsY):
			u1, v1 = yStep * L + u, yStep * L + v
			u2, v2 = u1 + L, v1 + L
			valuesPlane.append( (4, u2, v2, v1, u1) )

	valuesPlaneSide = polygonize(valuesVert, valuesEdge, sX=sizeX)
	valuesPlane.extend( valuesPlaneSide )
	toLastSide = lambda i: i + L * nStepsY
	valuesPlane.extend( (poly[0],) + tuple(map(toLastSide, poly[1:])) for poly in valuesPlaneSide )

	return valuesVert3D, valuesPlane


def regionsTo3D(valuesRegion, dF=dimFunc, dE1=dimEnd1, dE2=dimEnd2):
	'''
	Converts the region values to 3D form
	'''
	valuesRegion3D = []
	for x, z, bc, A in valuesRegion:
		newY = sizeY / (nStepsY * 2.0)
		newZ = dF(dE1 + newY / sizeY * (dE2 - dE1)) * z
		valuesRegion3D.append( (x, newY, newZ, bc, A * sizeY / 10) )
	return valuesRegion3D


def formatPlanes(s, valuesPlane):
	'''
	Due to requirements on the .poly files for 3D, edges are now planes (facets)
	and have different formats.
	'''
	sp = (len(valuesPlane),) + s[1:]
	valuesPlane = [(1,) if i % 2 == 0 else valuesPlane[i/2] for i in range(2 * len(valuesPlane))]
	return [sp] + valuesPlane


def create(sX=sizeX, sY=sizeY, sZ=sizeZ, mF=mountainFunc, 
					 mE1=mountEnd1, mE2=mountEnd2, dA=desiredAmplitude, 
					 nStX=nStepsX, nStY=nStepsY, nSeX=nSegsX, nSeY=nSegsY,
					 dF=dimFunc, dE1=dimEnd1, dE2=dimEnd2):
	'''
	THIS IS THE MAIN METHOD
	'''

	baseString = makePolyFile.create(sX=sX, sY=sZ, mF=lambda x: mF(x,0), e1=mE1[0], e2=mE2[0], dA=dA, nSt=nStX, nSe=nSeX)
	
	sVert, sEdge, sHole, sRegion, valuesVert, valuesEdge, valuesHole, valuesRegion = parseString(baseString)

	valuesVert, interface = makeInterface(valuesVert, mountainFunc=mF, nStepsX=nStX, nStepsY=nStY, mountEnd1=mE1, mountEnd2=mE2)

	valuesVert3D, valuesPlane = VertEdgeTo3D(valuesVert, valuesEdge, interface, dimFunc=dF, dE1=dimEnd1, dE2=dimEnd2)

	''' 
	print out mathematica format data for "ListPointPlot3D" commmand. For example:
	http://2.bp.blogspot.com/-Sm9CNAqm97I/VZR7LyBLThI/AAAAAAAAQxc/9NWt1uulN3Q/s1600/nodes.png
	'''
	# mathematica = map(lambda (a,b,c,d): "{%f,%f,%f}" %(a,b,c), valuesVert3D)
	# mathematica = ",".join(mathematica)
	# print(mathematica)

	valuesRegion3D = regionsTo3D(valuesRegion, dF)

# process the data for output in the form of a .poly file
# structure of a .poly file:
# http://wias-berlin.de/software/tetgen/1.5/doc/manual/manual006.html#sec66
	finalArray = makePolyFile.formatSection(('zzz', '3', '0', '1'), valuesVert3D) + \
				 				formatPlanes(sEdge, valuesPlane) + \
				 				makePolyFile.formatSection(sHole, valuesHole) + \
				 				makePolyFile.formatSection(sRegion, valuesRegion3D)

	finalArray = makePolyFile.toFinalArray(finalArray)
	return "\n".join(finalArray) + "\n"


h = open(writePath, 'w')
h.write(create())
h.close()


