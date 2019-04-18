# -*- coding: utf-8 -*-
"""
Created on Mon Apr  8 11:10:57 2019

@author: joao ricardo
"""

import time 
from threading import Thread
from naoqi import ALProxy
import numpy
import matplotlib.pyplot as plt
from PIL import Image

#this implementation tittled implementation, is able to explore an area, define a map  and navigate on it in order to pick an object.
#the differente factor present in this implementation is the use of Threads to compare the path described on the map to define the points and to move the object.
#since threads are used, more than one function will be running at the same time, the response time to begin the movement gets higher.
#at this moment the script is only able to run once at a time before requiring to redo the exploration and definition of the desired points.

#Global variables are used in all of the scricpt to comunicate values of variables between threads

class Fios(Thread):
    path=''
    running = False
    a=list()
    b=list()
    c=list()
    d=list()
    e=list()
    f=list()
    
    def __init__(self,val): # __init__ is the first thing to run when the program is started
        #in the function __init__ we call every module that is going to be used throughout the present script. Since this script will use threads at some point, this should also be called here.
        Thread.__init__(self)
        self.val = val
        
        self.tts = ALProxy("ALTextToSpeech", "172.16.0.77", 9559)
        self.motion = ALProxy("ALMotion", "172.16.0.77", 9559)
        self.posture = ALProxy("ALRobotPosture", "172.16.0.77", 9559)
        self.tracker = ALProxy("ALTracker", "172.16.0.77", 9559)
        self.memory = ALProxy("ALMemory","172.16.0.77",9559)
        self.perception = ALProxy("ALPeoplePerception", "172.16.0.77", 9559)
        self.navigation = ALProxy("ALNavigation","172.16.0.77",9559)
        self.posture.goToPosture("StandInit",0.5)
        
        #The parameters corresponding to the use of the landmark are defined here. there are two ways of defining the distance to a certain landmark. the one presented here in __init__ and the one presented in qrcode2
        self.targetName = "LandMark"
        self.distanceX = 0.2  #distance pertended to the target
        self.distanceY = 0.0
        self.angleWz = 0.0
        self.thresholdX = 0.1 #threshold used in order to break the loop of qrcode measurements
        self.thresholdY = 0.1
        self.thresholdWz = 0.3
        self.subscribeDone = False
        self.effector = "None"
        self.isRunning = False
        self.sizeMark = 0.09 #the sizeMark should be close to the diameter of the actual physical landmarks. the diameter will be used by the robot to calculate the distance to the target. 
        self.markIds = [68, 85, 84, 204, 145, 76, 115, 153, 112, 11, 135, 127, 170, 123, 114, 121]
        
        #posture standinit gives a more natural position for the robot. instead of being looking to the sky it looks in the direction of the horizon
        self.posture.goToPosture("StandInit",0.5)

        #Since in this script we making a movement with the actuators, it is good practice to set the stiffness to the maximum in order for the robot to keep the final position of the actuators in the end of the movement.        
        stiffnesses  = 1.0
        names ="Body"
        self.motion.setStiffnesses(names, stiffnesses)
        
        self.names = list()
        self.times = list()
        self.keys = list()
    
        
    def qrco1_box(self):  
        self.motion.setAngles("HeadPitch",0.1,0.1) #it lowers the head to better search for a landmark that might be close

        mode = "Move" #The mode determines the way that the robot follows the stimuli from the target. the Move mode makes the robot move towards the target.
        self.tracker.setMode(mode)
        
        #the effector its used when it is pretended that a certain member of Pepper's body follow the object beeing tracked, can be set to "Arms" or "head" for instance. in ths case it is not used, however it is kept since it is part of the method
        self.tracker.setEffector(self.effector)
        
        self.tracker.registerTarget(self.targetName,  [self.sizeMark, self.markIds])
        
        #the tracking parameters associated to the tracking with landmarks are the ones defined in __init__. Define the distance to the target
        self.tracker.setRelativePosition([-self.distanceX, self.distanceY, self.angleWz,
                                           self.thresholdX, self.thresholdY, self.thresholdWz])
        
        self.tracker.track(self.targetName) #Start tracker
        self.isRunning = True  
  
        try:
            while True:
                time.sleep(0.5)
                self.tracker.setRelativePosition([-0.29, self.distanceY, self.angleWz, self.thresholdX, self.thresholdY, self.thresholdWz])
                
                relative_position = self.tracker.getRelativePosition() #it takes the coordinates set in setRelativePosition and fixed the distance pretended to the target.
                print relative_position
     
                target_position = self.tracker.getTargetPosition(2) #calculates the distance form the robot to the target, it is updated every second.
                print target_position
   
                if abs(target_position[0] + relative_position[0]) < 0.1: #when the distance to the target is smaller than 0.1 of the defined position, there is a break in the loop making the sript jump to the next step.
                    break

        except KeyboardInterrupt:
            pass
        
        finally:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()   #unregister target is necessary in order for the robot to be able toregister a new target. it is always necessary when there is a break in the tracking
            print "tracker stopped"
            
  
    def qrcode2_target(self):
        #it follows the same structure of the tracking used for the box thge difference is that in here the distance is sat to be 65cm.
        self.motion.setAngles("HeadPitch",0.1,0.1) #it lowers the head to better search for a landmark that might be close

        mode = "Move"
        self.tracker.setMode(mode)
       
        self.tracker.setEffector(self.effector)
        self.tracker.registerTarget(self.targetName,  [self.sizeMark, self.markIds])
    
        self.tracker.setRelativePosition([-self.distanceX, self.distanceY, self.angleWz,
                                           self.thresholdX, self.thresholdY, self.thresholdWz])

        self.tracker.track(self.targetName) #Start tracker
        self.isRunning = True  
         
        try:
            while True:
                time.sleep(1.)
                self.tracker.setRelativePosition([-0.65, 0.0, 0.0, 0.1, 0.1, 0.3]) #for the second detected landmark it is pretended a different distance compared to the first. instead of defining all the values in different variables we can just replace the name of the vairables in the method with the desired values.
                
                relative_position = self.tracker.getRelativePosition() #it takes the coordinates set in setRelativePosition and fixed the distance pretended to the target.
 
                target_position = self.tracker.getTargetPosition(2) #calculates the distance form the robot to the target, it is updated every second.
                print target_position
                       
                if abs(target_position[0] + relative_position[0]) < 0.1: #when the distance to the target is smaller than 0.1 of the defined position, there is a break theat terminates the loop making the sript jump to the next step.
                    break
                
        except KeyboardInterrupt:
            pass
        
        finally:
            self.tracker.stopTracker()
            self.tracker.unregisterAllTargets()  #unregister target is necessary in order for the robot to be able toregister a new target. it is always necessary when there is a break in the tracking
            print "tracker stopped"
    
    def explore(self, force=False):
        global path
        self.motion.wakeUp() #it is only used if the robot is in rest position.
        
        # Explore the environement, in a radius of 2 m.
        radius = 2.0
        error = self.navigation.explore(radius) 
        if error != 0:
            print "Exploration failed."
            return
        # Saves the exploration on the robots disk
        self.path = self.navigation.saveExploration()
        path=self.path
       
        print "Exploration saved at path: \"" + self.path + "\""
        # Start localization to navigate in map
        self.navigation.startLocalization()
        # Come back to initial position
        self.navigation.navigateToInMap([0., 0., 0.])
        # Stop localization
        self.navigation.stopLocalization()
        # Retrieve and display the map built by the robot
        result_map = self.navigation.getMetricalMap()
        map_width = result_map[1]
        map_height = result_map[2]
        img = numpy.array(result_map[4]).reshape(map_width, map_height)
        img = (100 - img) * 2.55 # from 0..100 to 255..0
        img = numpy.array(img, numpy.uint8)
        Image.frombuffer('L',  (map_width, map_height), img, 'raw', 'L', 0, 1).show()
        self.posture.goToPosture("StandInit",0.5)
        
    def def_point(self): #in def_point it is defined the movements needed in order to retrieve the desired coordinates of the map
        #if this funtion is to be run for any reason more then once in a row the location needs to be stopped 
        global a, b, c, d, e ,f, h
        
        self.navigation.stopLocalization()
        
        #two lists are created empty to later append the coordinates relative to X and Y of the coordinate system of the robot
        xlstpoints=[]
        ylstpoints=[]
        
        #loads the map from the directory where it was saved during exploration
        self.navigation.loadExploration(str(path))
        self.navigation.startLocalization()
        
        #the map made by the robot has associated a coordinate system relative to the robot. the place where the robot starts the movement is the [0.0,0.0,0.0]. we define this position as our home
        self.home = self.navigation.getRobotPositionInMap()
        print "saved home position"
        
        self.motion.moveTo(0.0,0.0,0.0)
       
        self.a = self.navigation.getRobotPositionInMap() #a is defined in the same position as home. the reason for this is to follow a logic when it is called in the steps.
        a=self.a 
        
        ax=self.a[0][0] #gets the x coordinate of the coordinate system of the map
        xlstpoints.append(ax) #appends the coordinate to the empty list, saving it as the first value
        ay=self.a[0][1] #gets the y coordinate of the coordinate system of the map
        ylstpoints.append(ay) #appends the coordinate to the empty list, saving it as the first value
        print "Saved position of A in map"
        time.sleep(2)
        
        #since the navigation method as a deprecated coordinate Z we are not able to use navigation to define the points themselfs. we use the ALMotion module instead that give us high precision of movement to retrieve the coordinates.
        self.motion.moveTo(0.2,0.0,0.0) #20 centimeters in front
        self.b = self.navigation.getRobotPositionInMap()
        b=self.b
        
        bx=self.b[0][0]
        xlstpoints.append(bx) #apppend the value of x coordinate in point b as the second value of the list
        by=self.b[0][1]
        ylstpoints.append(by)
        print "Saved position of B in map"
        time.sleep(2)
    
        self.motion.moveTo(0.2,0.0,0.0)
        self.c = self.navigation.getRobotPositionInMap()
        c=self.c
        
        cx=self.c[0][0]
        xlstpoints.append(cx) 
        cy=self.c[0][1]
        ylstpoints.append(cy)
        print "Saved position of C in map"
        time.sleep(2)
        
        self.motion.moveTo(0.0,0.0,-3.1415)#rotation of 180º in radians the minus sine make the rotation movement clockwise
        self.motion.moveTo(1.3,0.0,0.0)#after the rotation move 1.3m in front to save the position in the other direction
        self.d = self.navigation.getRobotPositionInMap()
        d=self.d
        
        dx=self.d[0][0]
        xlstpoints.append(dx)
        dy=self.d[0][1]
        ylstpoints.append(dy)
        print "Saved position of D in map"
        time.sleep(2)
        
        self.motion.moveTo(0.2,0.0,0.0)
        self.e = self.navigation.getRobotPositionInMap()
        e=self.e
        
        ex=self.e[0][0]
        xlstpoints.append(ex)
        ey=self.e[0][1]
        ylstpoints.append(ey)
        print "Saved position of E in map"
        time.sleep(2)
      
        self.motion.moveTo(0.2,0.0,0.0)
        self.f = self.navigation.getRobotPositionInMap()
        f=self.f
        
        fx=self.f[0][0]
        xlstpoints.append(fx)
        fy=self.f[0][1]
        ylstpoints.append(fy)
        print "Saved position of F in map"
        time.sleep(2)
        
        self.navigation.navigateToInMap([self.home[0][0],self.home[0][1],0.0])  #after the definition of all the points it is used the navigation method for pepper to reach the home position. the orientation on arrival is not under control
        self.h = self.navigation.getRobotPositionInMap()
        h=self.h
        
        hx=self.h[0][0]
        xlstpoints.append(hx)
        hy=self.h[0][1]
        ylstpoints.append(hy)
        print "home \n" 
        
        print "lista de pontos x"
        print (xlstpoints)
        print "lista de pontos y"
        print (ylstpoints)
        
        #plot the coordinates in a graphic that shows the path taken in relation to the coordinates of the robot
        plt.plot(xlstpoints, ylstpoints, 'r')
        plt.axis([-2.0, 1.5, -0.8, 0.5])
        plt.show()
        
        self.posture.goToPosture("StandInit",0.5)
        return self.home, self.a, self.b, self.c, self.d, self.e, self.f 
  

    def box_pick(self):
        global names, times, keys
        # Choregraphe simplified export in Python. The behaviour with the corresponding timeline used in Chorgraphe to create this will also be available on github.
        
        self.motion.setExternalCollisionProtectionEnabled("All", False) #disables the security regarding the colisions with the robot, makes it more efficient to pick the object
        names = list() #name of the joints
        times = list() #time in seconds taken for the movement
        keys = list() #angular values that the joints assume with time
        #the conjugation of this lists for every joint make it possible to create a movement. the values are retrieved after creating a timeline in Choregraphe and exporting it to python simplified
        
        leftArmEnable  = False
        rightArmEnable  = False
        self.motion.setMoveArmsEnabled(leftArmEnable, rightArmEnable) #disables autonomous motions for the robot's arms

        names.append("HeadPitch")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-0.197884, -0.197884, -0.201921, 0.0273087, -0.188725, -0.191747])
        
        names.append("HeadYaw")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([0.177942, -0.00153399, 0.000437361, -0.00734707, -0.00197911, 0.0460193])
        
        names.append("HipPitch")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-0.085903, -0.075165, -0.267778, -0.405169, -0.112075, -0.107379])
        
        names.append("HipRoll")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-0.0122719, -0.0122719, -0.00920421, -0.00622665, -0.00622665, -0.00920391])
        
        names.append("KneePitch")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([0.0352817, 0.0659611, 0.102488, 0.169177, 0.0625379, 0.0598252])
        
        names.append("LElbowRoll")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-0.605922, -0.946466, -0.829175, -1.11201, -1.12558, -1.12748])
        
        names.append("LElbowYaw")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-2.07394, -2.09694, 0.115263, 0.0368566, 0.166225, 0.0720971])
        
        names.append("LHand")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([0.59051, 0.59051, 0.973251, 0.971906, 0.840422, 0.818102])
        
        names.append("LShoulderPitch")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([1.70272, 0.329806, -0.244712, -0.25418, -0.385139, -0.408039])
        
        names.append("LShoulderRoll")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([0.268447, 1.20264, 0.371723, 0.338789, 0.3182, 0.322135])
        
        names.append("LWristYaw")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-0.0353239, 0.00149202, -1.81919, -1.77485, -1.72769, -1.70125])
        
        names.append("RElbowRoll")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([0.592117, 0.935729, 0.829169, 1.11125, 1.13531, 1.12748])
        
        names.append("RElbowYaw")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([2.0724, 2.09542, -0.115263, -0.0368501, -0.166221, -0.076699])
        
        names.append("RHand")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([0.594903, 0.594903, 0.97386, 0.970296, 0.820624, 0.789982])
        
        names.append("RShoulderPitch")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([1.71346, 0.322135, -0.234208, -0.257077, -0.39748, -0.406505])
        
        names.append("RShoulderRoll")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-0.260777, -1.19804, -0.386175, -0.337755, -0.325495, -0.311398])
        
        names.append("RWristYaw")
        times.append([1.52, 3.56, 5.96, 7.92, 9.6, 10.92])
        keys.append([-4.19617e-05, -0.06447, 1.81919, 1.77916, 1.71414, 1.67202])
        
        try:
            self.motion.angleInterpolation(names, keys, times, True) #angle intespolation is the method that will give life to the movement
            
            stiffnesses  = 1.0
            self.motion.setStiffnesses(names, stiffnesses) #stiffness make the robot keep the last position achieved without any unvoluntary autonomous movement

            #angles = [element[-1] for element in keys]
            #fractionMaxSpeed  = 0.001
            #self.motion.setAngles(names, angles, fractionMaxSpeed) 

        except BaseException, err:
            print err
        
    def box_move(self):
        # Choregraphe simplified export in Python. The behaviour with the corresponding timeline used in Chorgraphe to create this will also be available on github.
        #this function follows the same structure showed above. The robot lowers the box to be able to move without blocking the depth sensor
        
        global names, times, keys
        
        names = list()
        times = list()
        keys = list()
        
        names.append("LElbowRoll")
        times.append([0.36, 2.64])
        keys.append([-1.06587, -1.03851])
        
        names.append("LElbowYaw")
        times.append([0.36, 2.64])
        keys.append([0.195775, 0.130388])
        
        names.append("LHand")
        times.append([0.36, 2.64])
        keys.append([0.801525, 0.913005])
        
        names.append("LShoulderPitch")
        times.append([0.36, 2.64])
        keys.append([-0.368357, 0.599787])
        
        names.append("LShoulderRoll")
        times.append([0.36, 2.64])
        keys.append([0.290244, 0.271515])
        
        names.append("LWristYaw")
        times.append([0.36, 2.64])
        keys.append([-1.664, -1.76568])
        
        names.append("RElbowRoll")
        times.append([0.36, 2.64])
        keys.append([1.06672, 1.01703])
        
        names.append("RElbowYaw")
        times.append([0.36, 2.64])
        keys.append([-0.189732, -0.138058])
        
        names.append("RHand")
        times.append([0.36, 2.64])
        keys.append([0.802087, 0.891916])
        
        names.append("RShoulderPitch")
        times.append([0.36, 2.64])
        keys.append([-0.368027, 0.592117])
        
        names.append("RShoulderRoll")
        times.append([0.36, 2.64])
        keys.append([-0.287276, -0.251573])
        
        names.append("RWristYaw")
        times.append([0.36, 2.64])
        keys.append([1.664, 1.75792])
        
        try:
          self.motion.angleInterpolation(names, keys, times, True)
          stiffnesses  = 1.0
          self.motion.setStiffnesses(names, stiffnesses) #stiffness make the robot keep the last position achieved without any unvoluntary autonomous movement
        except BaseException, err:
          print err
    
    def box_drop(self):
        # Choregraphe simplified export in Python. The behaviour with the corresponding timeline used in Chorgraphe to create this will also be available on github.
        # this function follows the same structure as the one showed above. it makes the robot drop the object in his hands.
        global names, times, keys
        names = list()
        times = list()
        keys = list()
        
        names.append("HeadPitch")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.194815, -0.196191, -0.196191, -0.191747, -0.191747])
        
        names.append("HeadYaw")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.101243, -0.101219, -0.101219, -0.101243, -0.101243])
        
        names.append("HipPitch")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.0690292, -0.047628, -0.541503, -0.0690292, -0.0690292])
        
        names.append("HipRoll")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.00920391, -0.00921134, -0.00921134, -0.00920391, -0.00920391])
        
        names.append("KneePitch")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.0582912, -0.0117972, 0.204936, -0.0690292, -0.0690292])
        
        names.append("LElbowRoll")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-1.01554, -0.737834, -0.991477, -0.492408, -0.61666])
        
        names.append("LElbowYaw")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([0.103891, 0.0707059, 0.0707059, 1.85612, 0.36202])
        
        names.append("LHand")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([0.976795, 0.976734, 0.828987, 0.79174, 0.598418])
        
        names.append("LShoulderPitch")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.0737358, -0.138431, -0.144499, -0.374291, 1.32536])
        
        names.append("LShoulderRoll")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([0.262136, 0.110632, 0.641998, 1.39746, 0.371223])
        
        names.append("LWristYaw")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-1.82022, -1.81984, -1.71439, -1.71199, -1.70125])
        
        names.append("RElbowRoll")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([1.01778, 0.737834, 0.99739, 0.506214, 0.605922])
        
        names.append("RElbowYaw")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.103891, -0.0707057, -0.0707057, -1.86839, -0.365088])
        
        names.append("RHand")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([0.976795, 0.970481, 0.821113, 0.796134, 0.599297])
        
        names.append("RShoulderPitch")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.0737358, -0.138432, -0.144498, -0.516951, 1.31769])
        
        names.append("RShoulderRoll")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([-0.262138, -0.110637, -0.641999, -1.40819, -0.380428])
        
        names.append("RWristYaw")
        times.append([0.52, 1.84, 4.12, 7.2, 9.64])
        keys.append([1.81919, 1.81139, 1.68777, 1.68429, 1.67355])
        
        try:
          self.motion.angleInterpolation(names, keys, times, True)
          stiffnesses  = 1.0
          self.motion.setStiffnesses(names, stiffnesses) #stiffness make the robot keep the last position achieved without any unvoluntary autonomous movement
        except BaseException, err:
          print err
    
    def steps(self):
        
        global path
        global running
        global a, b, c, d, e ,f, h
        global head_pos
        global names, times, keys
        
        running = False #makes thread2 start working, gathering coordinates each 0.5s
        print "initiated behaviour"
        
        time.sleep(5)
        #the steps followed to make the behaviour. first the robot goes to 3 defined points close to each other to make adjustments to the orientation
        
        self.navigation.navigateToInMap([d[0][0],d[0][1],0.0])
        self.navigation.navigateToInMap([e[0][0],e[0][1],0.0])
        self.navigation.navigateToInMap([f[0][0],f[0][1],0.0])
        
        #when the robot arrives the third point, hopefully facing the object, it initiates the landmarkdetection 
        self.qrco1_box()
        time.sleep(0)
        
        #after arriving to the pretended distance it makes the movement to pick up the box
        #self.box_pick()
        time.sleep(10)
        #when the box is picked the robot slowly moves back to a safe distance where it can lower its arms
        self.motion.moveTo(-0.45,0.0,0.0,10)
        #the robot procedes to lower the arms
        #self.box_move()
        
        #then the robot starts moving autonomously to the target
        self.navigation.navigateToInMap([a[0][0],a[0][1],0.0])
        self.navigation.navigateToInMap([b[0][0],b[0][1],0.0])
        self.navigation.navigateToInMap([c[0][0],c[0][1],0.0])    

        #when the position in map is achieved the landmark tracking is once again used to look for the target.
        self.qrcode2_target()
        time.sleep(1) 
        #after reaching the target, the robot procides to droping the object
        #self.box_drop()
        #after dropping the object, the robot returns home in the initial pose
        self.navigation.navigateToInMap([a[0][0],a[0][1],0.0])  
        self.posture.goToPosture("StandInit",0.5)
        running = True #to stop the second thread
    
    def live_exp(self):     
        #function used to retrieve the path made live by the robot when its picking the object
        global path
        global running
        
        #two lists are created to append values of coordinates
        self.xlivepoints=[]
        self.ylivepoints=[]
        
        self.navigation.stopLocalization()
        print 'start retrieving coordinates'
        self.navigation.loadExploration(str(path)) #loads the path and starts localizing in the current path
        self.navigation.startLocalization()
        
        while True:    
            time.sleep(1.0)
            coord=self.navigation.getRobotPositionInMap() #gets the coordinates and retrieves the value of x and y 
            posx=coord[0][0] 
            posy=coord[0][1]
            
            self.xlivepoints.append(posx)
            self.ylivepoints.append(posy)
            
            if running == True:
                #when the funtion steps reaches the end of the behaviour running becomes true which will stop the collection of points and will plot the path taken
                plt.plot(self.xlivepoints, self.ylivepoints,)
                plt.axis([-2.0, 1.5, -0.8, 0.5])
                plt.show()
                break

    def run(self): #function related to threads. the lines inside the function run() are the ones runned when Thread is started (mythread.start())
        if self.val == 1:
            self.steps()
        
        elif self.val == 2: 
            self.live_exp()                
  

    def princ(self):      
        global path
        global a, b, c, d, e ,f, h
        
        # Declare objects of MyThread class
        myThreadOb1 = Fios(1)
        myThreadOb1.setName('Thread 1') 
        
        myThreadOb2 = Fios(2)
        myThreadOb2.setName('Thread 2')
        myThreadOb2.daemon = True #daemon will make sure that thread 2 stop when thread 1 ends
        
        # starts running both threads at the same time.
        print 'Initiating Threads'
        myThreadOb1.start()
        myThreadOb2.start()
    
        # Wait for the threads to finish...
        myThreadOb1.join()
        myThreadOb2.join()
        
        print 'Main Terminating...'