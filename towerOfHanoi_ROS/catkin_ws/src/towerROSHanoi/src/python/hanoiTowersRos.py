#!/usr/bin/env python
# this python manage the tower of hanoi problem using ROS
# created by Frederic Starnecker
# Student of HTWG-University of applied science Constance
# Master degree
# Date 2016

import numpy as np
import std_msgs.msg
import tf
from tf import TransformListener
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import rospy
import roslib

#from vrep_common import simRosStartSimulation
import math
import time
#import vrep
import vrep_common
from vrep_common.srv import simRosStartSimulation
from vrep_common.srv import simRosStopSimulation
from vrep_common.srv import simRosGetObjectPose
from vrep_common.srv import simRosSetObjectPose
from vrep_common.srv import simRosGetObjectHandle
from vrep_common.srv import simRosSetObjectPosition
from vrep_common.srv import simRosGetJointMatrix




#def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

#simGetObjectMatrix
def getTransformationMatrix(boxHandle):
    # epsilon for testing whether a number is close to zero
    EPS = np.finfo(float).eps * 4.0
    quaternion = srvPose(boxHandle,-1).pose.pose.orientation
    quaternion = [quaternion.x,quaternion.y,quaternion.z,quaternion.w]
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

# Param: listner all tfs, boxHandle searching
def getBoxAdjustedMatrixAndFacingAngle2(listener,boxHandle):       
    #lua: p2=simGetObjectPosition(boxHandle,-1)
    t = listener.getLatestCommonTime(boxHandle, "/map")
    p2, quaternion2 = listener.lookupTransform(boxHandle, "/map", t)
    print 'P2-vektor2: ',p2
     
    #lua: p1=simGetObjectPosition(vehicleReference,-1)
    t = listener.getLatestCommonTime("/base_link", "/map")
    p1, quaternion1 = listener.lookupTransform("/base_link", "/map", t)
    print 'P1-vektor2: ',p1

    return p2


# Param: listner all tfs, boxHandle searching
#def getBoxAdjustedMatrixAndFacingAngle(listener,boxHandle):
def getBoxAdjustedMatrixAndFacingAngle(boxHandle):       
    #lua: p2=simGetObjectPosition(boxHandle,-1)
    p2 = srvPose(boxHandle,-1).pose.pose.position
    p2=[p2.x,p2.y,p2.z]
    print 'P2-vektor: ',p2
    #t = listener.getLatestCommonTime(boxHandle, "/map")
    #p2, quaternion2 = listener.lookupTransform(boxHandle, "/map", t)
     
    #lua: p1=simGetObjectPosition(vehicleReference,-1)
    vehicle_handle = srvObHandle('youBot_vehicleReference').handle
    p1 = srvPose(vehicle_handle,-1).pose.pose.position
    p1=[p1.x,p1.y,p1.z]
    print 'P1-vektor: ',p1

    #t = listener.getLatestCommonTime("/base_link", "/map")
    #p1, quaternion1 = listener.lookupTransform("/base_link", "/map", t)

    #go on calulating the dirction and normalize
    p=[p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]]
    pl=math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])
    p[0]=p[0]/pl
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    
    print '	P-vektor: ', p

    #m=simGetObjectMatrix(boxHandle,-1)  or   m=GetObjectMatrix(boxHandle,"/map")
    m=getTransformationMatrix(boxHandle)
    
    print 'm getobj: \n',m
    #m = listener.fromTranslationRotation(position2,quaternion2)
    matchingScore=0
    for i in xrange(0,2):
        v=[m[0][i],m[1][i],m[2][i]] # go x,y,z ache
        print 'v: ',v,'\n'
        score=v[0]*p[0]+v[1]*p[1]+v[2]*p[2]
        print 'score: ',score, '\n'
        if (abs(score)>matchingScore):
            s=1
            if (score<0):
                s=-1
            matchingScore=abs(score)
            bestMatch=[v[0]*s,v[1]*s,v[2]*s]
    angle=math.atan2(bestMatch[1],bestMatch[0])
    print 'anle=math.atan2:', angle
    # m=simBuildMatrix(p2,{0,0,angle})
    #m = listener.fromTranslationRotation(p2,[0,0,angle,0])
    m[0][0] =  math.cos(angle)
    m[1][0] =  math.sin(angle)
    m[0][1] = -math.sin(angle)
    m[1][1] =  math.cos(angle)
    m[0][2] = 0
    m[2][0] = 0
    m[1][2] = 0
    m[2][1] = 0
    m[2][2] = 1
    m[3][3] = 1
    m[2][3] = 0
    m[3][2] = 0
    print 'm \n',m,'\n'
    return m, angle-math.pi/2

if __name__ == '__main__':
    print('Program started')
    ## Define Node and Topics give them time to get ready
    rospy.init_node("hanoi")
    
    # wait for important sevices of V-REP to come up
    rospy.wait_for_service("/vrep/simRosSetObjectPosition")
    rospy.wait_for_service("/vrep/simRosGetObjectPose")
    
    #start Simulator
    rospy.wait_for_service('/vrep/simRosStartSimulation')
    startSimulation = rospy.ServiceProxy('/vrep/simRosStartSimulation',simRosStartSimulation)
    resSimStart = startSimulation()  
    
    if -1 ==  (resSimStart.result):
        print 'Simulator could not start value:', resSimStart
    elif 1 ==  (resSimStart.result):
        print 'Simulator started value:',resSimStart
    else :# if 0
        print 'Simulator already running value:',resSimStart
    #time.sleep(3) #wait for node and topics to come up
    # subscibe service    
    srvObHandle = rospy.ServiceProxy('/vrep/simRosGetObjectHandle',simRosGetObjectHandle)
    srvPose = rospy.ServiceProxy('/vrep/simRosGetObjectPose',simRosGetObjectPose)
    srvSetPose = rospy.ServiceProxy('/vrep/simRosSetObjectPose',simRosSetObjectPose)
    srvSetPosition = rospy.ServiceProxy('/vrep/simRosSetObjectPosition',simRosSetObjectPosition)
    srvGetJointMatrix = rospy.ServiceProxy('/vrep/simRosGetJointMatrix',simRosGetJointMatrix)

    #Publish topics
    fkikpub = rospy.Publisher("setFkIk",std_msgs.msg.String,queue_size = 1)
    gripperPub = rospy.Publisher("gripper_controller/position_command",std_msgs.msg.String,queue_size = 1)
    setTargetPub = rospy.Publisher("cmd_vel_SetTarget",std_msgs.msg.Int32MultiArray,queue_size =1)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    redBox1 = "BoxTF_red1"
    tf_sub = TransformListener()
    
    time.sleep(1) #wait for node and topics to come up
    
    # define some relative positions
    pickup1={0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,0*math.pi/180}
    pickup2={0,-13.39*math.pi/180,-93.91*math.pi/180,-72.72*math.pi/180,90*math.pi/180}
    pickup3={0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,90*math.pi/180}
    platformIntermediateDrop={0,16*math.pi/180,52*math.pi/180,73*math.pi/180,0*math.pi/180}
    #platformDrop1={0,54.33*math.pi/180,32.88*math.pi/180,35.76*math.pi/180,0*math.pi/180}--{0,-0.4,0.2}
    #platformDrop2={0,40.74*math.pi/180,45.81*math.pi/180,59.24*math.pi/180,0*math.pi/180}--{0,-0.32,0.2}
    #platformDrop3={0,28.47*math.pi/180,55.09*math.pi/180,78.32*math.pi/180,0*math.pi/180}--{0,-0.24,0.2}
    #define distances, height and speed
    dist1=0.2
    dropHeight1=0.035
    dropHeight2=0.095
    dropHeight3=0.155
    ikSpeed={0.2,0.2,0.2,0.2}
    ikAccel={0.1,0.1,0.1,0.1}
    ikJerk={0.1,0.1,0.1,0.1}
    fkSpeed={1,1,1,1,1}
    fkAccel={0.6,0.6,0.6,0.6,0.6}
    fkJerk={1,1,1,1,1}
    #setGripperTargetMovingWithVehicle()
    gripperPub.publish("open")
    #setFkMode
    fkikpub.publish("fk")
    #openGripper

    # redBox first pickup:
    pose2=getBoxAdjustedMatrixAndFacingAngle2(tf_sub,redBox1)
    
    #print m
    #print '\n '
    #print angle
    #setTargetPub.publish(quaternion)
    # service test

    
    #use service
    RedCube_handle = srvObHandle('redRectangle1').handle
    vehicleTarget_handle = srvObHandle('youBot_vehicleTargetPosition').handle
    
    m,angle=getBoxAdjustedMatrixAndFacingAngle(RedCube_handle)
    pose_test = srvPose(RedCube_handle,-1).pose
   # print pose
   # responde = srvSetPose(vehicle_handle,-1,pose2.pose)

    #simSetObjectPosition(vehicleTarget,-1,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})

    
    pose = Pose()
    pose.position.x=m[1][0]-m[0][1]*dist1
    pose.position.y=m[2][0]-m[1][1]*dist1
    pose.position.z=0
    pose.orientation.x=0
    pose.orientation.y=0
    pose.orientation.z=90-math.pi/2
    
   # pose = pose_test.pose
    pose= srvPose(vehicleTarget_handle,-1).pose.pose
    TarPos= srvPose(RedCube_handle,-1).pose.pose
    #pose.position.x=TarPos.position.x-dist1
    pose.position.y=TarPos.position.y
    print pose.orientation.z
    print TarPos.orientation.z
  #  pose.position.z=0
   # pose.position.x=pose.position.x-2*dist1
    pose.orientation.x=TarPos.orientation.x
    pose.orientation.y=TarPos.orientation.y
    pose.orientation.z=TarPos.orientation.z#-math.pi/2
    responde = srvSetPose(vehicleTarget_handle,-1,pose)
   # responde = srvSetOrintation(vehicle_handle,-1,pose.orientation)
    

    


    #try:
        #handle = rospy.ServiceProxy("vrep/simRosGetObjectHandle","BoxTF_red1")
        #result, pose = rospy.ServiceProxy("/vrep/simRosGetObjectPose",'redRectangle1',-1)
        #print pose
        #set_result = rospy.ServiceProxy("/vrep/simRosSetObjectPose",pose)
        #set_result = rospy.ServiceProxy("/vrep/simRosSetObjectPosition", "youBot_vehicleTargetPosition",-1,{3,2,0})
               
    #    except rospy.ServiceException, e:
     #          print "Service call failed: %s"%e

    
    rospy.spin() # node needs to keep running, so the messages come through
        #start Simulator
        
    time.sleep(5) #just wait, a pause
    rospy.wait_for_service('/vrep/simRosStopSimulation')
    stopSimulation = rospy.ServiceProxy('/vrep/simRosStopSimulation',simRosStopSimulation)
    resSimStop = stopSimulation()  
    
    if -1 ==  (resSimStop.result):
        print 'Simulator could not b stoped value:', resSimStop
    else :# if 0
        print 'Simulator stopped value:',resSimStop
    print('Program ended')


