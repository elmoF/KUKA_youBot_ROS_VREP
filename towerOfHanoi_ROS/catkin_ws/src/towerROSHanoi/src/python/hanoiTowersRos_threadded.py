#!/usr/bin/env python
# this python manage the tower of hanoi problem using ROS
# created by Frederic Starnecker
# Student of HTWG-University of applied science Constance
# Master degree
# Date 2016

import numpy as np
import math
import time


# import ROS
import rospy
import roslib
import std_msgs.msg
import tf
from tf import TransformListener


# import KUKA youBot API messages
import brics_actuator.msg
import sensor_msgs.msg
from std_msgs.msg import Header
import geometry_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point

#import vrep Services
from vrep_common.srv import simRosStartSimulation
from vrep_common.srv import simRosStopSimulation
from vrep_common.srv import simRosGetObjectPose
from vrep_common.srv import simRosSetObjectPose
from vrep_common.srv import simRosGetObjectHandle
from vrep_common.srv import simRosSetObjectPosition
from vrep_common.srv import simRosGetJointMatrix
from vrep_common.srv import simRosSetObjectQuaternion
from vrep_common.srv import simRosSetObjectParent

# just 4 fun- creates twist commands
def createTwistCommand(tw):

    msg = geometry_msgs.msg.Twist()
    msg.linear.x = tw[0]
    msg.linear.y = tw[1]
    msg.linear.z = tw[2]
    
    msg.angular.x = tw[3]
    msg.angular.y = tw[4]
    msg.angular.z = tw[5]
    return msg 

# Multipication of Quaternions
def quadMUL(x,y):
    result = [-1,-1,-1,-1]
    result[3]= x[0]*y[0]-x[1]*y[1]-x[2]*y[2]-x[3]*y[3] #w
    result[0]= x[0]*y[1]+x[1]*y[0]+x[2]*y[3]-x[3]*y[2] #i
    result[1]= x[0]*y[2]-x[1]*y[3]+x[2]*y[0]+x[3]*y[1] #j
    result[2]= x[0]*y[3]+x[1]*y[2]-x[2]*y[1]+x[3]*y[0] #k
    
    return result

def createArmPositionCommand(newPositions):
    numberOfJoints =  5
    msg = brics_actuator.msg.JointPositions()
    if  len(newPositions) != numberOfJoints :
        return msg # return empty message if not enough values provided
    for i in xrange(0,numberOfJoints):
         # Set all values for one joint
         JointValue = brics_actuator.msg.JointValue()
         JointValue.timeStamp = rospy.Time.now()
         JointValue.value=newPositions[i]
         JointValue.unit = "rad"
         
         # create joint names: "arm_joint_1"...
         JointValue.joint_uri="arm_joint_",i
         # add joint to message
         msg.positions.append(JointValue)
    
    return msg
    
def createArmJointState(newPositions):
    numberOfJoints =  5
    numberOfcoordinates = 3
    msg = sensor_msgs.msg.JointState()
    msg.header= Header()
    msg.header.stamp = rospy.Time.now()
    
    if  len(newPositions) != numberOfJoints and  len(newPositions) != numberOfcoordinates:
        return msg # return empty message if not enough values provided
    
    prefix = "arm_joint_"            
    if numberOfcoordinates == len(newPositions):
        prefix = "coordinate"
        numberOfJoints = numberOfcoordinates
            
    for i in xrange(0,numberOfJoints):
        name = prefix +str(i)
        msg.name.append(name)       
        msg.position.append(newPositions[i])
        #msg.velocity.append()
        #msg.effort.append()
    #print '\n JointState: \n',msg
    return msg
    
    

# wait till vehicle is in position, acording tf informations
def waitToReachvehicleTargetNamePositionAndOrientation(listener,vehicleTargetName,vehicleReference):
    pError =oError =999999999
    
    while ((pError>0.001)and(oError>0.1*math.pi/180)) :
        time.sleep(0.3)
        p1,o1=getListenerPose(listener,vehicleTargetName,"map")
 
        p2,o2=getListenerPose(listener,vehicleReference,"map")
        p3,o3 = getListenerPose(listener,vehicleReference,vehicleTargetName)

        p=[p2[0]-p1[0],p2[1]-p1[1]]
        pError=math.sqrt(p[0]*p[0]+p[1]*p[1])
        
        oError=abs(o3[2])
    time.sleep(2)

def waitToReachGripperTarger(listener,Target1Name, gripperTip):
    pError =oError =999999999
    print 'wait for gripper to reach target\n'
    while ((pError>0.001)and(oError>0.123)) :
        
        time.sleep(0.3)
        #if type(Target1Name) != str:
        p1,o1=getListenerPose(listener,Target1Name,"map")
        p2,o2=getListenerPose(listener,gripperTip,"map")
        print 'p1: ',p1
        print 'p2: ',p2
        p3,o3 = getListenerPose(listener,gripperTip,Target1Name)
        print 'p3: ',p3,'\n'
        p=[p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]]
        print 'p: ',p,'\n'
        pError=math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])
        print 'wurzel p2-p1: ', pError,'\n'

        oError=abs(p3[2])
        #print pError, ' ' , oError,'\n'

    print 'gripper reached position'
    time.sleep(3)
# get tf pose from V-REP incl. Transformation
# Param: listner all tfs, boxHandle searching
def getListenerPose(listener,boxHandle,relTo):       
    #lua: p2=simGetObjectPosition(boxHandle,-1)
    t = listener.getLatestCommonTime(boxHandle, relTo)
    pos, orien = listener.lookupTransform(relTo, boxHandle, t)
    print boxHandle,': \npos:',pos,'\norien:',orien

    return pos, orien
    
    
# get new Pose  
# Param: listener and objectHandle    
def getBoxAdjustedMatrixAndFacingAngleTF(listener,objectHandle,vehicleHandle,dist):  
    print 'getBoxAdjustedMatrixAndFacingAngleTF\n'
    p2,rot2=getListenerPose(listener,objectHandle,"map")
    p1,rot1=getListenerPose(listener,vehicleHandle,"map")
  #  boxHandle = srvObHandle('redRectangle1').handle
  #  vehicleTarget_handle = srvObHandle('youBot_vehicleTargetNamePosition').handle
  #  poseP2 = srvPose(boxHandle,-1).pose.pose
  #  p2 = poseP2.position
  #  p2=[p2.x,p2.y,p2.z]
  #  rot2 = [poseP2.orientation.x,poseP2.orientation.y,poseP2.orientation.z,poseP2.orientation.w]
  #  poseP1 = srvPose(vehicleTarget_handle,-1).pose.pose
  #  p1 = poseP1.position
  #  p1=[p1.x,p1.y,p1.z]

    print 'trans2',p2
    print 'rot2:',rot2
    
    # dirction and normalize
    p=[p2[0]-p1[0],p2[1]-p1[1],p2[2]-p1[2]]
    pl=math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])
    p[0]=p[0]/pl
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    print 'p',p
    ######### V-REP:simGetObjectMatrix
    # numpy arrays to 4x4 transform matrix 
    transl_mat = tf.transformations.translation_matrix(p2)
    rot_mat = tf.transformations.quaternion_matrix(rot2)
    # create a 4x4 matrix
    m = np.dot(transl_mat, rot_mat)
    print 'm\n',m 

    #V-REP: find best Orientation
    matchingScore=0
    for i in xrange(0,3):
        v=[m[0][i],m[1][i],m[2][i]] # go x,y,z ache
        print 'v: ',v
        score=v[0]*p[0]+v[1]*p[1]+v[2]*p[2]
        print 'score', score

        if (abs(score)>matchingScore):
            s=1
            if (score<0):
                s=-1
            matchingScore=abs(score)
            bestMatch=[v[0]*s,v[1]*s,v[2]*s]
        print i,'.bestmatchtemp:',bestMatch        
    angle=math.atan2(bestMatch[1],bestMatch[0])

    print 'angle: ', angle 
    print 'final Bestmatch',bestMatch
    #### m=simBuildMatrix(p2,{0,0,angle})

    #quad = tf.transformations.quaternion_from_euler(0, 0, angle)
    #rotMat =tf.transformations.quaternion_matrix(quad)
    m = tf.transformations.euler_matrix(0, 0, angle,axes='sxyz')
    print 'trans1',p1
    m[0][3] = p2[0]
    m[1][3] = p2[1]
    m[2][3] = p2[2]
    #rotMat =tf.transformations.quaternion_matrix(quad)    
    
   # m = np.dot(transl_mat,  rotMat)
    print 'return m:\n', m
    new_poseTF = Pose()
    new_poseTF.position.x= (m[0][3]-m[0][0]*dist)#0.54999976158142#m[0][3]-m[0][0]
    new_poseTF.position.y= (m[1][3]-m[1][0]*dist)#-0.00000003952776133076#m[1][3]-m[1][0]
    #new_poseTF.position.x= m[0][3]-m[0][0]#0.54999976158142#m[0][3]-m[0][0]
    #new_poseTF.position.y= m[1][3]-m[1][0]#-0.00000003952776133076#m[1][3]-m[1][0]
  
    new_poseTF.position.z= 0
    quad = tf.transformations.quaternion_from_euler(0, 0, angle-(0.5*math.pi))
    new_poseTF.orientation.x= quad[0]
    new_poseTF.orientation.y= quad[1]
    new_poseTF.orientation.z= quad[2]
    new_poseTF.orientation.w= quad[3]
    
    print 'target pose',new_poseTF
    return new_poseTF,m,angle

def dropToPlace(placeHandle,shift,verticalPos,startConf,noVerticalArmForUpMovement,dist,listener,targetName,refName):
    print 'dropToPlace:',placeHandle,'\n'
    print 'targetName: ', targetName,'\n'

    #get transformmatrix of drop place    
    newposeTF,m,angle=getBoxAdjustedMatrixAndFacingAngleTF(listener,placeHandle,refName,dist)    
    newposeTF.position.x+=m[0][1]*shift
    newposeTF.position.y +=m[1][1]*shift

    # drive to position/place, by setting vehicle Target position
    srvSetPose(vehicleTarget_handle,-1,newposeTF)
    pub_fkik.publish("fk")
    
    # set arm: to Param-startConf position
    #time.sleep(5)  
    msg = createArmJointState(startConf)
    pub_youbot_armJointState.publish(msg)
    
    # wait for youbot for receiving target
    waitToReachvehicleTargetNamePositionAndOrientation(listener,targetName,refName)
    # set IK-true mode to place at position
    pub_fkik.publish("iktrue")
    
    # Get object position of gripperTarget to world
    #p,rot = getListenerPose(listener,placeHandle,"map")
  
    # set cube on exact position
    p=[m[0][3]+m[0][1]*shift,m[1][3]+m[1][1]*shift,verticalPos]
    print '\n\n\n\n\n\n', p
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    time.sleep(2) # pause to get in position


    # wait to reach target
    time.sleep(4)
    # openGripper: release cube
    pub_gripper.publish("open")
    
    if (noVerticalArmForUpMovement):
        # no vertical orintation to the target frame
        pub_fkik.publish("ikfalse")
    # prevent slipping over cube
    time.sleep(1)
    print p[2]
    p[2]+= 0.1
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    time.sleep(2.5)
    pub_fkik.publish("fk")

# pick up a box from a specific place
#pickupBoxFromPlace(tf_sub,yellowBox1Name,pickup2,vehicleTargetName,vehicleName,dist1)
def pickupBoxFromPlace(listener,boxHandle,pickupConf,targetName,refName,gripperTip,dist):
    # get transformation matrix of  box and set as target
    newposeTF,m,angle=getBoxAdjustedMatrixAndFacingAngleTF(listener,boxHandle,refName,dist) 
    #drive to target
    srvSetPose(vehicleTarget_handle,-1,newposeTF)
    
    # get in position for pickup box from place and height
    pub_fkik.publish("fk") 
    msg = createArmJointState(pickupConf)
    pub_youbot_armJointState.publish(msg)
    time.sleep(4) #pause to get on position
    # wait for youbot for receiving target
    waitToReachvehicleTargetNamePositionAndOrientation(listener,targetName,refName)
    pub_fkik.publish("iktrue")
    # Get object position of gripperTarget to world
    pub_gripper.publish("open")
    p,rot = getListenerPose(listener,boxHandle,"map")
    p = [m[0][3], m[1][3], m[2][3]]
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    # grab object/cube
    waitToReachGripperTarger(tf_sub,boxHandle,gripperTip)
    pub_gripper.publish("close")
    time.sleep(2)
    #lift cube
    p[2]=p[2]+0.1
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    pub_fkik.publish("fk")
    time.sleep(3)

# drop sth. onto plattform
def dropToPlatform(platform):
    pub_fkik.publish("fk")
    msg = createArmJointState(platform)
    pub_youbot_armJointState.publish(msg)
    time.sleep(8)
    pub_gripper.publish("open")
    
# pickup cube from platform    
def pickupFromPlatformAndReorient(listener,boxHandle,platformIntermediateDrop,vehicleref,vehicleTarget,gripperTip,dist,pickup):

    pub_fkik.publish("fk")
    msg = createArmJointState(platformIntermediateDrop)
    pub_youbot_armJointState.publish(msg)
    time.sleep(4)
    pub_fkik.publish("ikfalse")
    p,rot = getListenerPose(listener,boxHandle,"map")
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    waitToReachGripperTarger(tf_sub,boxHandle,gripperTip)
    pub_gripper.publish("close")

    pub_fkik.publish("fk")

    #  -- Move a bit back from current position:
    #new_poseTF,m,angle=getBoxAdjustedMatrixAndFacingAngleTF(listener,vehicleTarget,vehicleref,dist)    
    p,rot = getListenerPose(listener,gripperTip,"map")
    p2,rot = getListenerPose(listener,vehicleTarget,"map")
    new_poseTF = Pose()
    new_poseTF.position.x = p[0]
    new_poseTF.position.y = p[1]
    new_poseTF.position.z = 0.03
    new_poseTF.orientation.x = rot[0]
    new_poseTF.orientation.y = rot[1]
    new_poseTF.orientation.z = rot[2]
    new_poseTF.orientation.w = rot[3]
   
    srvSetPose(vehicleTarget_handle,-1,new_poseTF)
    waitToReachvehicleTargetNamePositionAndOrientation(listener,vehicleTarget,vehicleref)
   # time.sleep(5)

    #-- Now drop it
    msg = createArmJointState(pickup)
    pub_youbot_armJointState.publish(msg)
    time.sleep(6)
    pub_gripper.publish("open")
    time.sleep(2.5)

    #-- Now orient yourself according to the box and pick it up:
    newposeTF,m,angle=getBoxAdjustedMatrixAndFacingAngleTF(listener,boxHandle,vehicleref,dist) 
    srvSetPose(vehicleTarget_handle,-1,newposeTF)
    time.sleep(1)
    waitToReachvehicleTargetNamePositionAndOrientation(listener,vehicleTarget,vehicleref)

    pub_fkik.publish("iktrue")
    # Get object position of gripperTarget to world
    pub_gripper.publish("open")
    p,rot = getListenerPose(listener,boxHandle,"map")
    p = [m[0][3], m[1][3], m[2][3]]
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    # grab object/cube
    waitToReachGripperTarger(tf_sub,boxHandle,gripperTip)
    pub_gripper.publish("close")
    time.sleep(2)
    p[2]=p[2]+0.1
    msg = createArmJointState(p)
    pub_youbot_armJointState.publish(msg)
    time.sleep(1)
    pub_fkik.publish("fk")
    time.sleep(2)


     


##########
## Main
############

if __name__ == '__main__':
    print('Program started')
    ## Define Node and Topics give them time to get ready
    rospy.init_node("hanoi")
    
    #### wait for important sevices of V-REP to come up
    print 'waiting for v-rep'
    rospy.wait_for_service("/vrep/simRosSetObjectPosition")
    rospy.wait_for_service("/vrep/simRosGetObjectPose")
    
    ##### start Simulator
    rospy.wait_for_service('/vrep/simRosStartSimulation')
    startSimulation = rospy.ServiceProxy('/vrep/simRosStartSimulation',simRosStartSimulation)
    resSimStart = startSimulation()  
    
    if -1 ==  (resSimStart.result):
        print 'Simulator could not start value:', resSimStart
    elif 1 ==  (resSimStart.result):
        print 'Simulator started value:',resSimStart
    else :# if 0
        print 'Simulator already running value:',resSimStart


    ###### subscibe V-REP services    
    srvObHandle = rospy.ServiceProxy('/vrep/simRosGetObjectHandle',simRosGetObjectHandle)
    srvPose = rospy.ServiceProxy('/vrep/simRosGetObjectPose',simRosGetObjectPose)
    srvSetPose = rospy.ServiceProxy('/vrep/simRosSetObjectPose',simRosSetObjectPose)
    srvSetPosition = rospy.ServiceProxy('/vrep/simRosSetObjectPosition',simRosSetObjectPosition)
    srvSetObjectOrientation = rospy.ServiceProxy('/vrep/simRosSetObjectQuaternion',simRosSetObjectQuaternion)
    srvGetJointMatrix = rospy.ServiceProxy('/vrep/simRosGetJointMatrix',simRosGetJointMatrix)
    srvsimRosSetObjectParent = rospy.ServiceProxy('/vrep/simRosSetObjectParent',simRosSetObjectParent)

    ##### advertise topics
    pub_fkik = rospy.Publisher("setFkIk",std_msgs.msg.String,queue_size = 1)
    pub_gripper = rospy.Publisher("gripper_controller/position_command",std_msgs.msg.String,queue_size = 1)
    pub_cmd_vel = rospy.Publisher('cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    pub_youbot_arm = rospy.Publisher('/arm_controller/position_command',  brics_actuator.msg.JointPositions,queue_size =1)
    pub_youbot_armJointState = rospy.Publisher('/arm_controller/JointState', sensor_msgs.msg.JointState, queue_size=10)
    pub_youbot_armPose = rospy.Publisher('/moveArm2Pos',geometry_msgs.msg.Pose,queue_size=1)   
    ## Ros name definitions 
    #TF
    redBox1Name = "BoxTF_red1"
    yellowBox1Name = "BoxTF_yellow1"
    yellowBox2Name = "BoxTF_yellow2"
    yellowBox3Name = "BoxTF_yellow3"
    greenBox1Name = "BoxTF_green1"
    greenBox2Name = "BoxTF_green2"
    greenBox3Name = "BoxTF_green3"
    vehicleName = "base_link"
    vehicleTargetName = "youBotTarget"
    world = "map"
    gripperTip = "gripperTip"
    place1 = "DropPlaceTF_1Start"
    place2 = "DropPlaceTF_2Temp"
    place3 = "DropPlaceTF_3Goal"
    
    ######  Subsrcibe topics
    tf_sub = TransformListener()
    
    ###### Do Hanoi
    #wait for node and topics to come up
    time.sleep(2) 
    
    # define some relative positions
    pickup1=[0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,0*math.pi/180]
    pickup2=[0,-13.39*math.pi/180,-93.91*math.pi/180,-72.72*math.pi/180,90*math.pi/180]
    pickup3=[0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,90*math.pi/180]
   # pickup4=[0,-15.39*math.pi/180,-93.91*math.pi/180,-72.72*math.pi/180,90*math.pi/180]
    platformIntermediateDrop=[0,16*math.pi/180,52*math.pi/180,73*math.pi/180,0*math.pi/180]
    platformDrop1=[0,54.33*math.pi/180,32.88*math.pi/180,35.76*math.pi/180,0*math.pi/180]#{0,-0.4,0.2}
    platformDrop2=[0,40.74*math.pi/180,45.81*math.pi/180,59.24*math.pi/180,0*math.pi/180]#{0,-0.32,0.2}
    platformDrop3=[0,28.47*math.pi/180,55.09*math.pi/180,78.32*math.pi/180,0*math.pi/180]#{0,-0.24,0.2}
    
    #define distances, height and speed
    dist1=0.2
    dropHeight1=0.03
    dropHeight2=0.095
    dropHeight3=0.155
    ikSpeed={0.2,0.2,0.2,0.2}
    ikAccel={0.1,0.1,0.1,0.1}
    ikJerk={0.1,0.1,0.1,0.1}
    fkSpeed={1,1,1,1,1}
    fkAccel={0.6,0.6,0.6,0.6,0.6}
    fkJerk={1,1,1,1,1}
    #--setGripperTargetMovingWithVehicle()

    #--setFkMode
    pub_fkik.publish("fk")
    #openGripper
    pub_gripper.publish("open")
    ######-- redBox first pickup:
        
    # use V-REP service for getting handles
    gripperTarget_handle = srvObHandle('youBot_gripperPositionTarget').handle
    RedCube_handle = srvObHandle('redRectangle1').handle
    vehicleTarget_handle = srvObHandle('youBot_vehicleTargetPosition').handle
    vehicle_handle = srvObHandle('youBot_vehicleReference').handle
    #pose= srvPose(vehicleTarget_handle,-1).pose.pose
    #TarPos= srvPose(RedCube_handle,-1).pose.pose
    


    # mit  Transformationsmatrix

    newTFPose,tm,ang= getBoxAdjustedMatrixAndFacingAngleTF(tf_sub,redBox1Name,vehicleName,dist1)
    #newTFPose.position.x = newTFPose.position.x*dist1
    #newTFPose.position.y = newTFPose.position.y*dist1
    print 'targetPos*Dist',newTFPose
    responde = srvSetPose(vehicleTarget_handle,-1,newTFPose)
    waitToReachvehicleTargetNamePositionAndOrientation(tf_sub,vehicleTargetName,vehicleName)
    print 'Hurray I am there'
    
    ### VREP: simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickup1,nil)
    jointvalues=pickup1
    
    # brics_actuator not supported in V-REP
   # msg = createArmPositionCommand(jointvalues)
    #print type(msg)
    #pub_youbot_arm.publish(msg)

    #### sensor_msgs/JointState
    # move arm in pickup1 position FK
    msg = createArmJointState(jointvalues)
    pub_youbot_armJointState.publish(msg)
    
    # wait to get in position, no tf infromation for this position
    time.sleep(5)
    
    # change to Ik mode to grap redbox
    pub_fkik.publish("iktrue")
    # get object position for gripperTarget(redBox) to world
    trans,rot=getListenerPose(tf_sub,redBox1Name,world)
    # set gripper Target
    msg = createArmJointState(trans)
    pub_youbot_armJointState.publish(msg)
    waitToReachGripperTarger(tf_sub,redBox1Name,gripperTip)
    # close gripper to grap box
    pub_gripper.publish("close")
    time.sleep(2) # pause so box is realy picked
    msg.position[2] += 0.05 # lift box
    pub_youbot_armJointState.publish(msg)
    time.sleep(1) # pause, prevent slipping over yellow stone
    
    ## redBox first drop:
    print '\n\n\n\nDrive to place 3 ',      
    dropToPlace(place3,0,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    
    ### yellow box1 first pickup and intermediate drop:
    pickupBoxFromPlace(tf_sub,yellowBox1Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlatform(platformDrop2) # yellow box1 intermediate drop onto platform:
    pickupBoxFromPlace(tf_sub,yellowBox2Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01) # yellow box2 first pickup:
    #### drop yellow
    dropToPlace(place2,0.04,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    pickupFromPlatformAndReorient(tf_sub,yellowBox1Name,platformIntermediateDrop,vehicleName,vehicleTargetName,gripperTip,dist1-0.01,pickup2)    
    dropToPlace(place2,-0.04,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    
    # build subtower on place 3
    pickupBoxFromPlace(tf_sub,redBox1Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlace(place2,0,dropHeight2,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    
    # pickup green boxes
    pickupBoxFromPlace(tf_sub,greenBox1Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlatform(platformDrop1)
    
    pickupBoxFromPlace(tf_sub,greenBox2Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlatform(platformDrop3)
    
    pickupBoxFromPlace(tf_sub,greenBox3Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    
    # set green boxes
    dropToPlace(place3,0.08,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    pickupFromPlatformAndReorient(tf_sub,greenBox2Name,platformIntermediateDrop,vehicleName,vehicleTargetName,gripperTip,dist1-0.01,pickup2)
    dropToPlace(place3,0.0,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    pickupFromPlatformAndReorient(tf_sub,greenBox1Name,platformIntermediateDrop,vehicleName,vehicleTargetName,gripperTip,dist1-0.01,pickup2)
    dropToPlace(place3,-0.08,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)    
    
    # replace red box
    pickupBoxFromPlace(tf_sub,redBox1Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlace(place1,0,dropHeight1,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    
    # place yellow boxes on tower(green)
    pickupBoxFromPlace(tf_sub,yellowBox1Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)    
    dropToPlatform(platformDrop2)
    pickupBoxFromPlace(tf_sub,yellowBox2Name,pickup2,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlace(place3,0.04,dropHeight2,pickup2,False,dist1,tf_sub,vehicleTargetName,vehicleName)
    pickupFromPlatformAndReorient(tf_sub,yellowBox1Name,platformIntermediateDrop,vehicleName,vehicleTargetName,gripperTip,dist1-0.01,pickup2)
    dropToPlace(place3,-0.04,dropHeight2,pickup3,True,dist1,tf_sub,vehicleTargetName,vehicleName)
    # place red box on tower
    pub_fkik.publish("fk")
    pickupBoxFromPlace(tf_sub,redBox1Name,pickup1,vehicleTargetName,vehicleName,gripperTip,dist1-0.01)
    dropToPlace(place3,0,dropHeight3,pickup1,True,dist1,tf_sub,vehicleTargetName,vehicleName)
    
    # return into the middle
    new_poseTF = Pose()
    new_poseTF.position.x = 0
    new_poseTF.position.y = 0
    new_poseTF.position.z = 0

    new_poseTF.orientation.x = 0
    new_poseTF.orientation.y = 0
    new_poseTF.orientation.z = 0  
    new_poseTF.orientation.w = 0   
    srvSetPose(vehicleTarget_handle,-1,new_poseTF)
    pub_fkik.publish("fk")
    msg = createArmJointState(platformIntermediateDrop)
    pub_youbot_armJointState.publish(msg)
    waitToReachvehicleTargetNamePositionAndOrientation(tf_sub,vehicleTargetName,vehicleName)
    
    # do the twist and sing Let's twist agian
    dotwist = [0,0,0,2,2,2] 
    cmd = createTwistCommand(dotwist)
    pub_cmd_vel.publish(cmd)
    time.sleep(5)
    
    #stopp Simulator   
    time.sleep(2) #just wait, a pause
    rospy.wait_for_service('/vrep/simRosStopSimulation')
    stopSimulation = rospy.ServiceProxy('/vrep/simRosStopSimulation',simRosStopSimulation)
    resSimStop = stopSimulation()  

    if -1 ==  (resSimStop.result):
        print 'Simulator could not b stoped value:', resSimStop
    else :# if 0
        print 'Simulator stopped value:',resSimStop
    print('Program ended')


