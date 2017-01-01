#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; #roslib.load_manifest('keyboard_test')
import rospy
import datetime
import numpy as np

#Library for RRT
import random
import math
import copy
import time

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from gazebo_msgs.msg import ModelStates
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class MoveController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		# Holds the value for x,y,z and x,y,z angular. 
		self.statex = 0
		self.statey = 0
		self.statez = 0
		self.x = 0
		self.y = 0
		self.z = 0
		self.statexAngle = 0
		self.stateyAngle = 0
		self.statezAngle = 0

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		self.position = rospy.Subscriber('/gazebo/model_states', ModelStates,self.ReceiveModelState)

		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty, queue_size=10)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty, queue_size=10)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty, queue_size=10)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist, queue_size=10)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		#rospy.loginfo('Hello Im status')
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		self.status = navdata.state
		

	def ReceiveModelState(self,modelstate):
		self.statex = modelstate.pose[11].position.x 
		self.statey = modelstate.pose[11].position.y
		self.statez = modelstate.pose[11].position.z

	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
			self.pubCommand.publish(self.command)
	####################################################################################################
	####################################################################################################
	####################################################################################################
	####################################################################################################
	def Move(self, x , y, z):
		
		while self.statex > x+0.4 or self.statex < x-0.4 or self.statey > y+0.4 or self.statey < y-0.4 or self.statez > z+0.4 or self.statez < z-0.4:
			if self.statex > x+1 or self.statex < x-1:
				if self.statex < x-1:
					self.SetCommand(0,1)#speed control
				elif self.statex > x+1:
					self.SetCommand(0,-1)
			if self.statey > y+1 or self.statey < y-1:
				if self.statey < y-1:
					self.SetCommand(1)#speed control
				elif self.statey > y+1:
					self.SetCommand(-1)
			if self.statez > z+1 or self.statez < z-1:
				if self.statez < z-1:
					self.SetCommand(0,0,0,1)#speed control
				elif self.statez > z+1:
					self.SetCommand(0,0,0,-1)		
	#####################################################################################################
			if self.statex > x+0.4 or self.statex < x-0.4:
				if self.statex < x-0.4:
					self.SetCommand(0,0.4)#speed control
				elif self.statex > x+0.4:
					self.SetCommand(0,-0.4)
			if self.statey > y+0.4 or self.statey < y-0.4:
				if self.statey < y-0.4:
					self.SetCommand(0.4)#speed control
				elif self.statey > y+0.4:
					self.SetCommand(-0.4)
			if self.statez > z+0.4 or self.statez < z-0.4:
				if self.statez < z-0.4:
					self.SetCommand(0,0,0,0.4)#speed control
				elif self.statez > z+0.4:
					self.SetCommand(0,0,0,-0.4)		
		print("Point Reached:", self.statex,"Final y:", self.statey)
		
	####################################################################################################
	####################################################################################################
	####################################################################################################

	def Target(self, ml, loc):
		#Args = [0]*2
		#Args = loc[ml]
		x = loc[ml][0]
		y = loc[ml][1]
		z = 2
		self.Move(x,y,z)		
		
###############################################################################################################
###############################################################################################################		
###############################################################################################################
###############################################################################################################		
###############################################################################################################

	def running(self, la):
		r = rospy.Rate(35) #rate increases speed
		LocList = list()
		LocList = la
		size = len(LocList)
		print('Initial location:')
		print("X: ",self.statex, " Y: ",self.statey)
		self.SendTakeoff()
		while not rospy.is_shutdown():
			while size > 0:	
				self.Move(LocList[size-1][0],LocList[size-1][1],2)
				size = size -1			
			if not rospy.is_shutdown():
				self.SendLand()
		r.sleep()
			
##########################################################################################
class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,randArea,expandDis=1.0,goalSampleRate=5,maxIter=500):
        u"""
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]
        """
        self.start=Node(start[0],start[1])
        self.end=Node(goal[0],goal[1])
        self.minrand = randArea[0]
        self.maxrand = randArea[1]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter

    def Planning(self,animation=True):
        u"""
        Pathplanning 
        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
        while True:
            # Random Sampling
            if random.randint(0, 100) > self.goalSampleRate:
                rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(self.minrand, self.maxrand)]
            else:
                rnd = [self.end.x, self.end.y]

            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            # print(nind)

            # expand tree
            nearestNode =self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if not self.__CollisionCheck(newNode, obstacleList):
                continue

            self.nodeList.append(newNode)

            # CHECK
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                print("Goal!!")
                break

            if animation:
                self.DrawGraph(rnd)

            
        path=[[self.end.x,self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([node.x,node.y])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])

        return path

    def DrawGraph(self,rnd=None):
        import matplotlib.pyplot as plt
        plt.clf()
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [node.y, self.nodeList[node.parent].y], "-g")
        for (x,y,size) in obstacleList:
            self.PlotCircle(x,y,size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([-20, 20, -20, 20])
        plt.grid(True)
        plt.pause(0.01)

    def PlotCircle(self,x,y,size):
        deg=range(0,360,5)
        deg.append(0)
        xl=[x+size*math.cos(math.radians(d)) for d in deg]
        yl=[y+size*math.sin(math.radians(d)) for d in deg]
        plt.plot(xl, yl, "-k")

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1]) ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

class Node():
    u"""
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def GetPathLength(path):
    l = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        l += d

    return l


def GetTargetPoint(path, targetL):
    l = 0
    ti = 0
    lastPairLen = 0
    for i in range(len(path) - 1):
        dx = path[i + 1][0] - path[i][0]
        dy = path[i + 1][1] - path[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        l += d
        if l >= targetL:
            ti = i-1
            lastPairLen = d
            break

    partRatio = (l - targetL) / lastPairLen
    #  print(partRatio)
    #  print((ti,len(path),path[ti],path[ti+1]))

    x = path[ti][0] + (path[ti + 1][0] - path[ti][0]) * partRatio
    y = path[ti][1] + (path[ti + 1][1] - path[ti][1]) * partRatio
    #  print((x,y))

    return [x, y, ti]


def LineCollisionCheck(first, second, obstacleList):
    # Line Equation

    x1=first[0]
    y1=first[1]
    x2=second[0]
    y2=second[1]

    try:
        a=y2-y1
        b=-(x2-x1)
        c=y2*(x2-x1)-x2*(y2-y1)
    except ZeroDivisionError:
        return False

    #  print(first)
    #  print(second)

    for (ox,oy,size) in obstacleList:
        d=abs(a*ox+b*oy+c)/(math.sqrt(a*a+b*b))
        #  print((ox,oy,size,d))
        if d<=(size):
            #  print("NG")
            return False

    #  print("OK")

    return True  # OK


def PathSmoothing(path, maxIter, obstacleList):

    l = GetPathLength(path)

    for i in range(maxIter):
        # Sample two points
        pickPoints = [random.uniform(0, l), random.uniform(0, l)]
        pickPoints.sort()
        #  print(pickPoints)
        first = GetTargetPoint(path, pickPoints[0])
        #  print(first)
        second = GetTargetPoint(path, pickPoints[1])
        #  print(second)

        if first[2]<=0 or second[2]<=0:
            continue

        if (second[2]+1) > len(path):
            continue

        if second[2]==first[2]:
            continue

        #CHECK COLLISON
        if not LineCollisionCheck(first, second, obstacleList):
            continue

        newPath=[]
        newPath.extend(path[:first[2]+1])
        newPath.append([first[0],first[1]])
        newPath.append([second[0],second[1]])
        newPath.extend(path[second[2]+1:])
        path=newPath
        l = GetPathLength(path)
    return path

	
	
if __name__ == '__main__':
	import matplotlib.pyplot as plt
	#######################################RAPIDLY EXPLORING RANDOM TREES###############################################
	# Circle Objects (Obstacles)
	obstacleList = [
		(-8, -1.5, 1),
		(-6, -1.5, 1),
		(-4, -1.5, 1),
		(-2, -1.5, 1),
		(0, -1.5, 1),
		(-8, -3.5, 1),
		(-8, -5.5, 1),
		(-8, -7.5, 1),
		(-6, -7.5, 1),
		(-4, -7.5, 1),
		(-2, -7.5, 1),
		(0, -7.5, 1),
		(0, -5.5, 1),
		(0, -3.5, 1),
		(0, -1.5, 1),
		(1.5, -3, 0.7),
		(2.5, -3, 0.7),
		(3.5, -3, 0.7),
		(4.5, -3, 0.7),
		(6, -2.5, 0.7),
		(6, -1.5, 0.7),
		(6, -0.5, 0.7),
		(6, 0.5, 0.7),
		(9, 3.5, 1.15),
		(7, 8, 0.7),
		(8, 8, 0.7),
		(9, 8, 0.7),
		(10, 8, 0.7),
		(11, 8, 0.7),
		(12, 8, 0.7),
		(13, 8, 0.7),
		(14, 8, 0.7),
		(15, 8, 0.7),
		(5.5, 8.5, 1),
		(3.5, 8.5, 1),
		(1.5, 8.5, 1),
		(-0.5, 8.5, 1),
		(-0.5, 10.5, 1),
		(-0.5, 12.5, 1),
		(1.5, 12.5, 1),
		(3.5, 12.5, 1),
		(5.5, 12.5, 1),
		(5.5, 10.5, 1),
		(-5.5, 12.5, 1),
		(-3.5, 8.5, 1),
		(-5.5, 8.5, 1),
		(-7.5, 8.5, 1),
		(-9.5, 8.5, 1),
		(-9.5, 10.5, 1),
		(-9.5, 12.5, 1),
		(-7.5, 12.5, 1),
		(-3.5, 12.5, 1),
		(-3.5, 10.5, 1),
	]  # [x value ,y value , size]
	rrt=RRT(start=[0,0],goal=[13,11],randArea=[-20,20],obstacleList=obstacleList)
	path=rrt.Planning(animation=True)

	# Draw Blue Path
	rrt.DrawGraph()
	plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')

	#Smooth Path
	maxIter=1500
	smoothedPath = PathSmoothing(path, maxIter, obstacleList)
	plt.plot([x for (x,y) in smoothedPath], [y for (x,y) in smoothedPath],'-b')
	plt.pause(0.01)
	print ('smoothedPath', smoothedPath)
	plt.grid(True)
	text_file = open("Output.txt", "w")
    	text_file.write("Smoothed Path: %s" % smoothedPath)
   	text_file.close()
	#######################ROSPY/GAZEBO##################################################
	rospy.init_node('ardrone_autonomy', anonymous=True)
	try:
		controller = MoveController()
		controller.SendTakeoff()
		time.sleep(5)	
		controller.running(smoothedPath)
	except rospy.ROSInterruptException:
		pass

