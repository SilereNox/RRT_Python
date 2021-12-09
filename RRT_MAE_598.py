# -*- coding: utf-8 -*-
"""
Created on Sat Dec  4 13:39:05 2021

@author: Silent
"""

import numpy as np
import cv2 as cv
from numpy import random
import matplotlib.pyplot as plt
import math

class RRTMap:
    def __init__(self,start,goal,MapDimensions):
        self.start=start
        self.goal=goal
        self.MapDimension=MapDimensions
        self.Mapw,self.Maph=self.MapDimension[0],self.MapDimension[1]
        self.MapImg=np.ones([self.Maph,self.Mapw,3],np.uint8)*255 # white background
        self.MapWindowName="RRT path planning with Voronoi Cells"
        self.nodeRad=0
        self.nodeThickness=-1
        self.edgeThickness=1
        #Colors
        self.Black = (20, 20, 20)
        self.Blue = (255, 0, 0)
        self.Green = (0, 255, 0)
        self.Red = (0, 0, 255)
        self.white = (255, 255, 255)

        self.obstacles=[]



    def drawMap(self,obstacles):
        self.drawNode(None,nodeType='G')
        self.drawNode(None,nodeType='S')
        self.drawObs(obstacles)
        cv.imshow(self.MapWindowName,self.MapImg)
        cv.waitKey(1)

    def refreshMap(self):
        cv.imshow(self.MapWindowName, self.MapImg)
        cv.waitKey(1)

    def drawNode(self,coords,nodeType):
        if nodeType=='G': # draw goal node
            cv.circle(self.MapImg,
                      (self.goal[0],self.goal[1]),
                      self.nodeRad,
                      self.Blue,
                      10)
        if nodeType=='S': # draw start node
            cv.circle(self.MapImg,
                      (self.start[0], self.start[1]),
                      self.nodeRad,
                      self.Green,
                      10)
        if nodeType=='N': # draw normal node
            cv.circle(self.MapImg,
                      (coords[0], coords[1]),
                      self.nodeRad,
                      self.Blue,
                      self.nodeThickness)
        if nodeType=='P': # draw path node
            cv.circle(self.MapImg,
                      (coords[0], coords[1]),
                      self.nodeRad,
                      self.Red,
                      4)

    # draw the edge between two nodes
    def drawEdge(self,node1,node2):
        cv.line(self.MapImg,
                (node1[0],node1[1]),
                (node2[0],node2[1]),
                self.Blue,
                self.edgeThickness)

    # draw the pah given the path nodes coords
    def drawPath(self,path):
        for node in path:
            self.drawNode(coords=node,nodeType='P')
            cv.imshow(self.MapWindowName,self.MapImg)
            cv.waitKey(1)

    # draw obstacles given the obstacle list as a param
    def drawObs(self,obstacles):
        obstaclesList=obstacles.copy()
        while(len(obstaclesList)>0):
            upper=obstaclesList.pop(0) # upper corner of the  obstacle
            lower=obstaclesList.pop(0) # lower corner of the obstacle
            cv.rectangle(self.MapImg,
                         (upper[0],upper[1]),
                         (lower[0],lower[1]),
                          self.Black,
                         -1)
    
    def rect_contains(self, rect, point) :
        if point[0] < rect[0] :
            return False
        elif point[1] < rect[1] :
            return False
        elif point[0] > rect[2] :
            return False
        elif point[1] > rect[3] :
            return False
        return True
        
    def draw_delaunay(self, subdiv, delaunay_color ) :
        triangleList = subdiv.getTriangleList();
        size = self.MapImg.shape
        r = (0, 0, size[1], size[0])
    
        for t in triangleList :
    
            pt1 = (t[0], t[1])
            pt2 = (t[2], t[3])
            pt3 = (t[4], t[5])
    
            if self.rect_contains(r, pt1) and self.rect_contains(r, pt2) and self.rect_contains(r, pt3) :
    
                cv.line(self.MapImg, pt1, pt2, delaunay_color, 1, 0)
                cv.line(self.MapImg, pt2, pt3, delaunay_color, 1, 0)
                cv.line(self.MapImg, pt3, pt1, delaunay_color, 1, 0)
    
    def draw_voronoi(self, points):
        size = self.MapImg.shape
        rect = (0,0,size[0], size[1])
        subdiv = cv.Subdiv2D(rect)
        subdiv.insert(points)
        ( facets, centers) = subdiv.getVoronoiFacetList([])
    
        for i in range(0,len(facets)) :
            ifacet_arr = []
            for f in facets[i] :
                ifacet_arr.append(f)
    
            ifacet = np.array(ifacet_arr, int)
            ifacets = np.array([ifacet])
            cv.polylines(self.MapImg, ifacets, True, (0, 0, 0), 1)
            cv.circle(self.MapImg, (centers[i][0], centers[i][1]), 3, (0, 0, 0), 0)


class RRT:
    
    def __init__(self, root, goal, mapDimensions):
        self.start = root
        self.goal = goal
        self.goalFlag = False
        self.x = []
        self.y = []
        self.parentNode = []
        self.x.append(self.start[0])
        self.y.append(self.start[1])
        self.parentNode.append(0)
        self.mapHeight,self.mapWidth = mapDimensions
        self.goalClosestNode = None
        self.path = []
        self.obstacles = []
        self.obstacleDimension = 15
        self.numberObstacles = 100
    
    def cost(self, node):
        startNode = 0
        node = node
        parentNode = self.parentNode[node]
        cost = 0
        while node is not startNode:
            cost = cost + self.calcDistance(node, parentNode)
            node = parentNode
            if node is not startNode:
                parentNode = self.parentNode(node)
        return cost
    
    def makeRandomRect(self):
        # centers of random rects
        centerX=int(random.uniform(self.obstacleDimension/2,self.mapWidth-self.obstacleDimension/2))
        centerY=int(random.uniform(self.obstacleDimension/2,self.mapHeight-self.obstacleDimension/2))
        # upper and lower conrners
        upperCornerX=(centerX-int(self.obstacleDimension/2))
        upperCornerY=(centerY-int(self.obstacleDimension/2))
        return [upperCornerX,upperCornerY]

    def makeobs(self):
        obstacleArray=[]
        for i in range(0,self.numberObstacles-1):
            upper=self.makeRandomRect()
            obstacleArray.append(upper)
            obstacleArray.append([upper[0]+self.obstacleDimension,upper[1]+self.obstacleDimension])
        self.obstacles=obstacleArray.copy()
        return obstacleArray
    
    def addNode(self, node, x, y):
        self.x.insert(node, x)
        self.y.insert(node, y)
        
    def removeNode(self, node):
        self.x.pop(node)
        self.y.pop(node)
        
    def addVirtex(self, node, parentNode):
        self.parentNode.insert(parentNode,node)
        
    def removeVirtex(self, node):
        self.parentNode.pop(node)
        
    def countNodes(self):
        return len(self.x)
    
    def calcDistance(self, node, parentNode):
        (x1, y1) = (self.x[node], self.y[node])
        (x2, y2) = (self.x[parentNode], self.y[parentNode])
        x1 = float(x1)
        x2 = float(x2)
        y1 = float(y1)
        y2 = float(y2)
        px = (x1 - x2) **2
        py = (y1 - y2) **2
        distance = (px + py)**0.5
        return distance
    
    def nearest(self, node):
        distance = self.calcDistance(0, node)
        nearestNode = 0
        for i in range(0,node):
            newDistance = self.calcDistance(i, node)
            if newDistance < distance:
                distance = newDistance
                nearestNode = i
        return nearestNode
    
    def inFreeSpace(self):
        node = self.countNodes()-1
        (x,y) = (self.x[node], self.y[node])
        obstacles = self.obstacles.copy
        # lowerBound = self.mapHeight
        # rightBound = self.mapWidth
        while len(obstacles)>0: 
            upper = obstacles.pop(0)
            lower = obstacles.pop(0)
            if upper[0] < x < lower[0] and upper[1] < y < lower[1]:
                self.removeNode(node)
                return False
            # if x > lowerBound:
            #     self.removeNode(node)
            #     return False
            # if y > rightBound:
            #     self.removeNode(node)
            #     return False
        return True
    
    def randomSample(self):
        x = int(random.uniform(0, self.mapWidth))
        y = int(random.uniform(0, self.mapHeight))
        return x, y
    
    def crossObstacles(self,x1,x2,y1,y2):
        obstacles=self.obstacles.copy()
        while(len(obstacles)>0):
            upper=obstacles.pop(0)
            lower=obstacles.pop(0)
            for i in range(0,101):
                u=i/100
                x=x1*u + x2*(1-u)
                y=y1*u + y2*(1-u)
                if upper[0] < x < lower[0] and upper[1] < y < lower[1]:
                    return True
        return False
    
    def connect(self, node, parentNode):
        (x1, y1) = (self.x[node], self.y[node])
        (x2, y2) = (self.x[parentNode], self.y[parentNode])
        if self.crossObstacles(x1,x2,y1,y2):
            self.removeNode(parentNode)
            return False
        else:
            self.addVirtex(node, parentNode)
            return True
    
    def step(self, nearestNode, parentNode, stepDistance):
        distance = self.calcDistance(nearestNode, parentNode)
        if distance > stepDistance:
            # u = stepDistance/distance
            (xNearest, yNearest) = (self.x[nearestNode], self.y[nearestNode])
            (xParent, yParent) = (self.x[parentNode], self.y[parentNode])
            (px, py) = (xParent - xNearest, yParent - yNearest)
            theta = math.atan2(py, px)
            (x, y) = (int(xNearest + stepDistance * math.cos(theta)),
                      int(yNearest + stepDistance * math.sin(theta)))
            self.removeNode(parentNode)
            self.addNode(parentNode, x, y)
            if abs(x - self.goal[0])<20 and abs(y-self.goal[1])<20:
                self.goalClosestNode = parentNode
                self.goalFlag = True
                
    def finalPath(self, goal):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalClosestNode)
            newPosition = self.parentNode[self.goalClosestNode]
            while (newPosition != 0):
                self.path.append(newPosition)
                newPosition = self.parentNode[newPosition]
            self.path.append(0)
            print('Goal Reached')
        return self.goalFlag
    
    def getPathCoordinates(self):
        pathCoordinates = []
        for node in self.path:
            x,y = (self.x[node], self.y[node])
            pathCoordinates.append((x,y))
        return pathCoordinates
    
    def bias(self, goal):
        nodeCount = self.countNodes()
        self.addNode(nodeCount, goal[0], goal[1])
        nearestNode = self.nearest(nodeCount)
        self.step(nearestNode, nodeCount, 10)
        self.connect(nearestNode, nodeCount)
        return self.x, self.y, self.parentNode
    
    def expand(self):
        node = self.countNodes()
        x, y = self.randomSample()
        self.addNode(node, x, y)
        if self.inFreeSpace:
            xNearest = self.nearest(node)
            self.step(xNearest, node, 10)
            # x1 = self.x[node]
            # y1 = self.y[node]
            self.connect(xNearest, node)
        return self.x, self.y, self.parentNode
    
if __name__ == "__main__":
    start = (10,10)
    goal = (500,250)
    dimensions = (550, 550)
    rrt = RRT(start, goal, dimensions)
    map=RRTMap(start,goal ,dimensions )
    obstacles=rrt.makeobs()
    map.drawMap(obstacles)
    i = 1
    points = []
    rect = (0,0,550,300)
    map.drawNode(start, "S")
    map.drawNode(goal, "G")
    while (not rrt.finalPath(goal)):
        if i%10 == 0:
            X,Y,Parent = rrt.bias(goal)
            # print('x = ',X[-1],' and y = ',Y[-1])
            map.drawNode([X[-1],Y[-1]], nodeType="N")
            map.drawEdge( (X[-1],Y[-1]) , (X[Parent[-1]],Y[Parent[-1]])  )
            points.append((X[-1],Y[-1]))
            # map.draw_voronoi(points)
            map.refreshMap()
            cv.waitKey(0)
        else:
            X,Y,Parent = rrt.expand()
            # print('x = ',X[-1],' and y = ',Y[-1])
            map.drawNode([X[-1],Y[-1]], nodeType="N")
            map.drawEdge( (X[-1],Y[-1]) , (X[Parent[-1]],Y[Parent[-1]])  )
            points.append((X[-1],Y[-1]))
            # map.draw_voronoi(points)
            map.refreshMap()
        # if i%100 ==0:
        #     map.draw_voronoi(points)
        #     map.refreshMap()
        #     cv.waitKey(0)
        #     cv.destroyAllWindows()
        i+=1
        # print(i)
    
    # map.draw_voronoi(points)
    # map.refreshMap()
    map.drawPath(rrt.getPathCoordinates())
    # k = len(rrt.getPathCoordinates())
    # pathToGoal = rrt.getPathCoordinates()
    # i = 1
    # print('points[2] = ', points[2])
    # print('Parent:',Parent)
    # for k in range(len(pathToGoal)):
    #     print('path to goal = ', pathToGoal[k])
    #     point = pathToGoal[k]
    #     # print('Node index: ',Parent[k], pathToGoal[k])
    #     k -= 1
    cv.waitKey(0)
    cv.destroyAllWindows()


