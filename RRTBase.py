from curses.textpad import rectangle
from dis import dis
import math
from platform import node
import pygame
import random
import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

class RRTMap:
    def __init__(self,start,goal,MapDimension,ObsDimension,ObsNumber):
        # Define the start position and goal position of the swarm
        self.start = start
        self.goal = goal

        # Define map properties
        self.MapWidth,self.MapHeight = MapDimension
        self.MapDimension = MapDimension
        
        # Define the obstacle properties
        self.ObsDimension = ObsDimension
        self.ObsNumber = ObsNumber
        
        ### Inıtialize the test map window
        self.MapWindowName = 'RRT Path Planning Algorithm Test Bench'
        pygame.display.set_caption(self.MapWindowName)

        # Set map window width and height
        self.Map = pygame.display.set_mode((self.MapWidth,self.MapHeight))
        self.Map.fill((255,255,255))

        # Define the object includes map properties
        self.nodeRad = 2 # Node Radius
        self.nodeThickness = int(0.0) # Node Thickness
        self.edgeThickness = int(1.0) # Edge Thickness

        # Define the map obejct containers
        self.obstacles = list()

        # Define map objects colors
        self.grey = (70,70,70)
        self.blue = (0,0,255)
        self.green = (0,255,0)
        self.red = (255,0,0)
        self.white = (255,255,255)


    def drawMap(self,Obstacles):
        self.drawObs(Obstacles)
        pygame.draw.circle(self.Map,self.blue,self.start,self.nodeRad+5,5)
        pygame.draw.circle(self.Map,self.blue,self.goal,self.nodeRad+20,1)

    def drawPath(self,path):
        for node in path:
            pygame.draw.circle(self.Map,self.red,node,self.nodeRad+3,0)

    def drawObs(self,Obstacles):
        '''
            @function drawObs : Draw the obstacles created in RRTGraph class
        '''
        obstacles_holder = Obstacles.copy()
        while (len(obstacles_holder)>0):
            obstacle = obstacles_holder.pop(0)
            pygame.draw.rect(self.Map,self.grey,obstacle)
            

'''
    @class RRTGraph : This class is for RRT tree structure.   
'''
class RRTGraph:
    def __init__(self,start,goal,MapDimension,ObsDimension,ObsNumber):
        # Define the start position and goal position of the swarm
        (x_start,y_start) = start # Hold the coordinates separately
        self.start = start
        self.goal = goal

        self.goalFlag = False # variables to check if the algorithm finds a path to the goal
        self.MapWidth,self.MapHeight = MapDimension
        '''
            Define the containers that hold the node coordinates and node parent number
        '''
        self.x_coord = list() # Define the nodes x coordinates data holder
        self.y_coord = list() # Define the nodes y coordinates data holder
        self.parentNodes = list() # Define the parent node list

        # Initialize the data holders one line above
        self.x_coord.append(x_start) # Add the number 0 node elements to list as start node
        self.y_coord.append(y_start)

        self.parentNodes.append(0) # Start node index is zero

        '''
            Define the obstacles properties
        '''
        self.ObsDimension = ObsDimension
        self.ObsNumber = ObsNumber

        self.Obstacles = list() # Define the all obstacles holder

        '''
            Define the path properties
        '''
        self.goalState = None
        self.path = list()


    def makeRandomRect(self):
        '''
            @function makeRandomRect : Define the upper-left corner of rect randomly
        '''
        rect_upper_corner_x = int(random.uniform(0,self.MapWidth-self.ObsDimension))
        rect_upper_corner_y = int(random.uniform(0,self.MapHeight-self.ObsDimension))
        return (rect_upper_corner_x,rect_upper_corner_y)

    def makeObs(self):
        '''
            @function makeObs : To be explained
        '''
        obstacles = list()

        for i in range(0,self.ObsNumber):
            rectangle = None
            isPointsInObs = True # Flag that checks the goal and start in obstacles situations 

            while isPointsInObs:
                upper_rect_coord = self.makeRandomRect()
                rectangle = pygame.Rect(upper_rect_coord,(self.ObsDimension,self.ObsDimension))

                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal): # Check the points in obstacles
                    isPointsInObs = True
                else:
                    isPointsInObs = False
            obstacles.append(rectangle)
        self.Obstacles = obstacles.copy()
        
        return obstacles



    def addNode(self,nodeId,randomNodeCoords):
        '''
            @function addNode : RRT algorithm puts a number point on a map and
            checks the node is fit or not. Algorithm keeps going until to find a
            suitable path. "addNode" function adds a number node on a map. It takes
            the id of node and coordinates of node.

            @param randomNodeCoords : (x,y) coordinates of random point.
            @param nodeId : id number of added node.
        '''

        # Add the random node to the node coordinates data holder
        temp_x,temp_y = randomNodeCoords
        self.x_coord.insert(nodeId,temp_x)
        self.y_coord.append(temp_y)

    def removeNode(self,nodeId):
        '''
            @function removeNode : Removes node from coordinates holders.

            @param nodeId : id number of node to be removed
        '''
        self.x_coord.pop(nodeId)
        self.y_coord.pop(nodeId)

    def addEdge(self,parentNodeId,childNodeId):
        '''
            @function addEdge : This function adds the tree edges between nodes.

            @param parentNodeId : Parent node id of current child node.
            @param childNodeId  : Child node id of current child node. 
        '''
        self.parentNodes.insert(childNodeId,parentNodeId)

    def removeEdge(self,childNodeId):
        '''
            @function removeEdge : Removes the parent-child relationship.

            @param childNodeId   : Node id of current child node.
        '''
        self.parentNodes.pop(childNodeId)

    def numberOfNodes(self):
        '''
            @funciton numberOfNodes : Return total number of nodes.

            @return : total number of nodes
        '''
        return len(self.x_coord)

    def distance(self,nodeId1,nodeId2):
        '''
            @function distance : Calculate the distance between two nodes.

            @param nodeId1 : Id of first node 
            @param nodeId2 : Id of second node
            
            @return distance between two nodes.
        '''
        (x1,y1) = (self.x_coord[nodeId1],self.y_coord[nodeId1])
        (x2,y2) = (self.x_coord[nodeId2],self.y_coord[nodeId2])
        # Calculate the distance
        # Euclidian rule
        px = (float(x2)-float(x1))**2
        py = (float(y2)-float(y1))**2
        return (px+py)**(0.5)

    def sampleEnv(self):
        '''
            @function sampleEnv : Generates the random point that is used to run exploration
            algorithm later.

            @return (x_sample,y_sample) generated random node coordinates.
        '''
        x_sample = int(random.uniform(0,self.MapWidth))
        y_sample = int(random.uniform(0,self.MapHeight))
        return (x_sample,y_sample)


    def isFree(self):
        '''
            @function isFree : Check the node is obstacle free.
        '''
        idOfCurrenNode = self.numberOfNodes()-1 # ids are start from zero (-1 meaning)
        # Hold the (x,y) coordinates of last node
        (x,y) = (self.x_coord[idOfCurrenNode],self.y_coord[idOfCurrenNode])
        # Hold temporariliy the obstacles
        obstacles = self.Obstacles.copy()

        # Check algorithm run
        while len(obstacles)>0:
            rectangle = obstacles.pop(0)
            if rectangle.collidepoint(x,y):
                self.removeNode(idOfCurrenNode)
                return False # if collision occures return false
        return True # if collision does not occur return true

    def crossObstacles(self,x1,x2,y1,y2):
        '''
            @function crossObstacles : Check if any edges crosses the obstacles.

            One way to do this is interpolation method. Cut the edge line into the multiple pieces.
            And check all the point if it is collided. This Algorithm is used.

        '''
        osbtacles = self.Obstacles.copy()
        while len(osbtacles)>0:
            rectangle = osbtacles.pop(0)
            for i in range(0,101):
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                
                if rectangle.collidepoint(x,y):
                    return True
        return False # If collision does not occur in edge aspect return false
        

    def connect(self,nodeId1,nodeId2):
        '''
            @function connect : Connect two nodes with and edge if there is no
            collision for edge.

            @param nodeId1 : Id number of first node.
            @param nodeId2 : Id number of second node.
        '''
        (x1,y1) = (self.x_coord[nodeId1],self.y_coord[nodeId1])
        (x2,y2) = (self.x_coord[nodeId2],self.y_coord[nodeId2])

        if self.crossObstacles(x1,x2,y1,y2):
            self.removeNode(nodeId2)
            return False
        else:
            self.addEdge(nodeId1,nodeId2)
            return True


    def nearest(self,currentNodeId):
        '''
            @function nearest : Finds the nearest node to the current node.
        '''
        distMin = self.distance(0,currentNodeId)
        nodeNear = 0

        for i in range(0,currentNodeId):
            if self.distance(i,currentNodeId)<distMin:
                distMin = self.distance(i,currentNodeId)
                nodeNear = i
        return nodeNear


    def step(self,nodeNear,nodeRand,distMax = 50):
        # nodeRand means random node
        dist = self.distance(nodeNear,nodeRand)

        if dist>distMax:
            u = distMax/dist
            (x_near,y_near) = (self.x_coord[nodeNear],self.y_coord[nodeNear])
            (x_rand,y_rand) = (self.x_coord[nodeRand],self.y_coord[nodeRand])

            (px,py) = (x_rand-x_near,y_rand-y_near)
            theta = math.atan2(py,px)
            (x,y) = (int(x_near+distMax*math.cos(theta)),
                    int(y_near + distMax*math.sin(theta)))
            
            self.removeNode(nodeRand)

            if abs(x-self.goal[0])<distMax and abs(y-self.goal[1])<distMax:
                self.addNode(nodeRand,(self.goal[0],self.goal[1]))
                self.goalState = nodeRand
                self.goalFlag = True
            else:
                self.addNode(nodeRand,(x,y))

    def bias(self,nodeGoal):
        numOfNodes = self.numberOfNodes()
        self.addNode(numOfNodes,(nodeGoal[0],nodeGoal[1]))
        nodeNear = self.nearest(numOfNodes)
        self.step(nodeNear,numOfNodes)
        self.connect(nodeNear,numOfNodes)

        return self.x_coord,self.y_coord,self.parentNodes

    def expand(self):
        numOfNodes = self.numberOfNodes()
        (x,y) = self.sampleEnv()
        self.addNode(numOfNodes,(x,y))

        if self.isFree():
            x_nearest = self.nearest(numOfNodes)
            self.step(x_nearest,numOfNodes)
            self.connect(x_nearest,numOfNodes)

        return self.x_coord,self.y_coord,self.parentNodes


    def pathToGoal(self):
        if self.goalFlag:
            self.path = list()
            self.path.append(self.goalState)

            new_pos = self.parentNodes[self.goalState]

            while new_pos != 0:
                self.path.append(new_pos)
                new_pos = self.parentNodes[new_pos]

            self.path.append(0)
        return self.goalFlag


    def getPathCoords(self):
        pathCoords = list()
        for node in self.path:
            (x,y) = (self.x_coord[node],self.y_coord[node])
            pathCoords.append((x,y))

        return pathCoords


class PlanPath:
    def __init__(self,startPoseOfSwarm,goalPoseOfSwarm,NumberOfIteration = 500,ObsDimension=50,ObsNumber = 15,MapDimension = (1920,1080)) -> None:
        
        # Start the environment parameters
        self.startPoseOfSwarm = startPoseOfSwarm
        self.goalPoseOfSwarm = goalPoseOfSwarm

        self.ObsDimension = ObsDimension
        self.ObsNumber = ObsNumber
        self.MapDimension = MapDimension

        # Initialize the iteration parameters
        self.iteration = 0

        # Pygame init

        pygame.init()
        self.Map = RRTMap(startPoseOfSwarm,goalPoseOfSwarm,MapDimension,ObsDimension,ObsNumber)
        self.Graph = RRTGraph(startPoseOfSwarm,goalPoseOfSwarm,MapDimension,ObsDimension,ObsNumber)

        self.obstacles = self.Graph.makeObs()
        self.Map.drawMap(self.obstacles)

    def execute(self):
        while not self.Graph.pathToGoal():
            if self.iteration % 10 == 0:
                X,Y,Parent = self.Graph.bias(self.goalPoseOfSwarm)
                pygame.draw.circle(self.Map.Map,self.Map.grey,(X[-1],Y[-1]),self.Map.nodeRad+2,0)
                pygame.draw.line(self.Map.Map,self.Map.blue,(X[-1],Y[-1]),(X[Parent[-1]],Y[Parent[-1]]),self.Map.edgeThickness)

            else:
                X,Y,Parent = self.Graph.expand()
                pygame.draw.circle(self.Map.Map,self.Map.grey,(X[-1],Y[-1]),self.Map.nodeRad+2,0)
                pygame.draw.line(self.Map.Map,self.Map.blue,(X[-1],Y[-1]),(X[Parent[-1]],Y[Parent[-1]]),self.Map.edgeThickness)
 
            if self.iteration % 1 == 0:
                pygame.display.update()
            self.iteration += 1



            if self.Graph.goalFlag:
                print("Destination has been reached!!!!")
        
        self.Map.drawPath(self.Graph.getPathCoords())
        #print(self.Graph.getPathCoords())
        pygame.display.update()
        pygame.event.clear()
        self.drawSplinePath()
        pygame.event.wait(0)

    def getPath(self):
        return self.Graph.getPathCoords()

    
    '''
        Get Bezier Spline Curves
    '''
    def getSplinePath(self):
        x = list()
        y = list()

        for point in self.Graph.getPathCoords():
            x.append(point[0])
            y.append(point[1])

        tck, *rest = interpolate.splprep([x,y])
        u = np.linspace(0,1,num = len(x)*2)
        #print(f"Length of RRT based points {len(x)}")
        self.smooth = interpolate.splev(u,tck)

        return self.smooth

    def drawSplinePath(self):
        self.getSplinePath()
        print(f"Smooth Path Number of Points{len(self.smooth[0])}")
        plt.figure()
        plt.plot(self.smooth[0],-self.smooth[1])
        plt.show()