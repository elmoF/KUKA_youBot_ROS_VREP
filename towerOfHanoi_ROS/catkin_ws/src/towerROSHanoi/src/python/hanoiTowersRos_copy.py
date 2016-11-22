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
from geometry_msgs.msg import Twist
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
    msg = sensor_msgs.msg.JointState()
    msg.header= Header()
    msg.header.stamp = rospy.Time.now()
    
    if  len(newPositions) != numberOfJoints :
        return msg # return empty message if not enough values provided
    for i in xrange(0,numberOfJoints):
        name= "arm_joint_"+str(i)
        msg.name.append(name)       
        msg.position.append(newPositions[i])
        #msg.velocity.append()
        #msg.effort.append()
    print '\n JointState: \n',msg
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


# get tf pose from V-REP incl. Transformation
# Param: listner all tfs, boxHandle searching
def getListenerPose(listener,boxHandle,relTo):       
    #lua: p2=simGetObjectPosition(boxHandle,-1)
    t = listener.getLatestCommonTime(boxHandle, relTo)
    pos, orien = listener.lookupTransform(boxHandle, relTo, t)
    print boxHandle,': \npos:',pos,'\norien:',orien
    pos = (-pos[0],-pos[1],-pos[2])
    orien = (-orien[0],-orien[1],-orien[2],1)
    return pos, orien
    
    
# get new Pose  
# Param: listener and objectHandle    
def getBoxAdjustedMatrixAndFacingAngleTF(listener,objectHandle,vehicleHandle):  
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
        print i
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
    new_poseTF.position.x= -(m[0][3]-m[0][0])#0.54999976158142#m[0][3]-m[0][0]
    new_poseTF.position.y= -(m[1][3]-m[1][0])#-0.00000003952776133076#m[1][3]-m[1][0]
    #new_poseTF.position.x= m[0][3]-m[0][0]#0.54999976158142#m[0][3]-m[0][0]
    #new_poseTF.position.y= m[1][3]-m[1][0]#-0.00000003952776133076#m[1][3]-m[1][0]
  
    new_poseTF.position.z= 0
    quad = tf.transformations.quaternion_from_euler(0, 0, -1*angle-math.pi/2)
    new_poseTF.orientation.x= -quad[0]
    new_poseTF.orientation.y= -quad[1]
    new_poseTF.orientation.z= quad[2]
    new_poseTF.orientation.w= quad[3]
    
    print 'target pose',new_poseTF
    return new_poseTF

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
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    pub_youbot_arm = rospy.Publisher('/arm_controller/position_command',  brics_actuator.msg.JointPositions,queue_size =1)
    pub_youbot_armJointState = rospy.Publisher('/arm_controller/JointState', sensor_msgs.msg.JointState, queue_size=10)
    pub_youbot_armPose = rospy.Publisher('/moveArm2Pos',geometry_msgs.msg.Pose,queue_size=1)   
    ## Ros name definitions 
    #TF
    redBox1Name = "BoxTF_red1"
    vehicleName = "base_link"
    vehicleTargetName = "youBotTarget"
    
    ######  Subsrcibe topics
    tf_sub = TransformListener()
    
    ###### Do Hanoi
    #wait for node and topics to come up
    time.sleep(1) 
    
    # define some relative positions
    pickup1=[0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,0*math.pi/180]
    pickup2=[0,-13.39*math.pi/180,-93.91*math.pi/180,-72.72*math.pi/180,90*math.pi/180]
    pickup3=[0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,90*math.pi/180]
    platformIntermediateDrop=[0,16*math.pi/180,52*math.pi/180,73*math.pi/180,0*math.pi/180]
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
    #--setGripperTargetMovingWithVehicle()

    #--setFkMode
    pub_fkik.publish("fk")
    #openGripper
    pub_gripper.publish("open")
    ######-- redBox first pickup:
    #poseTF,quadTF=getBoxAdjustedMatrixAndFacingAngle2(tf_sub,redBox1)
    
    # use V-REP service
    gripperTarget_handle =srvObHandle('youBot_gripperPositionTarget').handle
    RedCube_handle = srvObHandle('redRectangle1').handle
    vehicleTarget_handle = srvObHandle('youBot_vehicleTargetPosition').handle
    vehicle_handle = srvObHandle('youBot_vehicleReference').handle
    #pose= srvPose(vehicleTarget_handle,-1).pose.pose
    #TarPos= srvPose(RedCube_handle,-1).pose.pose
    
    
    #m,angle=getBoxAdjustedMatrixAndFacingAngle(RedCube_handle)

    #--Original simSetObjectPosition(vehicleTargetName,-1,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
 
    #pose = Pose()
   # pose.position.x=m[1][0]-m[0][1]*dist1
  #  pose.position.y=m[2][0]-m[1][1]*dist1
  #  pose.position.z=0
  #  pose.orientation.x=0
  #  pose.orientation.y=0
  #  pose.orientation.z=90-math.pi/2
    
    #############VREP solution
#    pose= srvPose(vehicleTarget_handle,-1).pose.pose
#    TarPos= srvPose(RedCube_handle,-1).pose.pose
  #  print "vehiclePose:\n",pose
  #  print "TarPos\n",TarPos    
    
  #  x = [TarPos.orientation.x,TarPos.orientation.y,TarPos.orientation.z,TarPos.orientation.w]
  #  kx= [TarPos.orientation.x,-TarPos.orientation.y,-TarPos.orientation.z,-TarPos.orientation.w]
  #  y = [pose.position.x,pose.position.y,pose.position.z,1]
  #  result = quadMUL(x,y)
  #  result = quadMUL(result,kx)
  
   # pose2 = Pose()
   # pose2.position.x=TarPos.position.x-2*dist1
   # pose2.position.y=TarPos.position.y
   # pose2.position.z=TarPos.position.z-dist1#-math.pi/2
   # 
   # x = [0,1,1,90-math.pi/2]
   # kx= [0,-1,-1,-90 -math.pi/2]
   # y = [pose.orientation.x,pose.orientation.y,pose.orientation.z,1]
   # result = quadMUL(x,y)
   # result = quadMUL(result,kx)
    
   # pose2.orientation.x= TarPos.orientation.x
   # pose2.orientation.y= TarPos.orientation.y
   # pose2.orientation.z= TarPos.orientation.z*90#(90 -math.pi/2)
   # pose2.orientation.z= TarPos.orientation.z*angle #bei gedrehtem Cube
   # pose2.orientation.w= TarPos.orientation.w*angle
   # print pose2
   # responde = srvSetPose(vehicleTarget_handle,-1,pose2)
  
    ###### ROS Listener Solution  
#    positionVehiclTF,quadVehicleTF=getListenerPose(tf_sub,redBox1Name,"map")
#    positionTF,quadTF=getListenerPose(tf_sub,redBox1Name,"map")
#    new_poseTF = Pose()
#    new_poseTF.position.x=-positionTF[0]-dist1
#    new_poseTF.position.y=-positionTF[1]#-dist1
#    new_poseTF.position.z=positionTF[2]
#    
#    new_poseTF.orientation.x= 0#-quadTF[0]+ p [0]
#    new_poseTF.orientation.y= 0#-quadTF[1]+ p [1]
#    new_poseTF.orientation.z= quadTF[2]
#    new_poseTF.orientation.w= math.cos(90)
    
#    angular = 4 * math.atan2(positionTF[1], positionTF[0])
#    linear = 0.5 * math.sqrt(positionTF[0] ** 2 + positionTF[1] ** 2)
#    cmd = geometry_msgs.msg.Twist()
#    cmd.linear.x = linear
#    cmd.angular.z = angular
    #pub_cmd_vel.publish(cmd)
#    angular =0
#    linear = 0
#    cmd = geometry_msgs.msg.Twist()
#    cmd.linear.x = linear
#    cmd.angular.z = angular
#    time.sleep(2)
   # pub_cmd_vel.publish(cmd)


#    print "vehiclePoseTF:\n","Position: ",getListenerPose(tf_sub,vehicleName,"map")[0],"\nOrientation",getListenerPose(tf_sub,vehicleName,"map")[1],"\n"
#    print "TarPosTF\n","Position: ",getListenerPose(tf_sub,redBox1,"map")[0],"\nOrientation",getListenerPose(tf_sub,redBox1,"map")[1],"\n"     
  
    
   # responde = srvSetPose(vehicleTarget_handle,-1,new_poseTF)
   # time.sleep(5)
 #   print "vehicleGoalPoseTF:\n","Position: ",getListenerPose(tf_sub,vehicleName,"map")[0],"\nOrientation",getListenerPose(tf_sub,vehicleName,"map")[1],"\n"
  



    #versuch mit  Transformationsmatrix
    dist1=2
    newTFPose= getBoxAdjustedMatrixAndFacingAngleTF(tf_sub,'BoxTF_red1',vehicleName)
    newTFPose.position.x = newTFPose.position.x*dist1
    newTFPose.position.y = newTFPose.position.y*dist1
    print 'targetPos*Dist',newTFPose
    responde = srvSetPose(vehicleTarget_handle,-1,newTFPose)
    waitToReachvehicleTargetNamePositionAndOrientation(tf_sub,vehicleTargetName,vehicleName)
    print 'Hurray I am there'
    
    ### VREP: simRMLMoveToJointPositions(armJoints,-1,nil,nil,fkSpeed,fkAccel,fkJerk,pickup1,nil)
    jointvalues=pickup1
    
    # brics_actuator not supported in V-REP
    msg = createArmPositionCommand(jointvalues)
    print msg
    pub_youbot_arm.publish(msg)

    # sensor_msgs/JointState
    msg = createArmJointState(jointvalues)
    pub_youbot_armJointState.publish(msg)
    
    time.sleep(5)
    pub_youbot_armPose.publish(newTFPose)
   # pub_fkik.publish("iktrue")
    
    
    
    
    
    
    
    
    #responde = srvSetPosition(vehicleTarget_handle,-1,newTFPose.position)
    #responde = srvSetObjectOrientation(vehicleTarget_handle,-1,newTFPose.orientation)
    #time.sleep(3)
    #newTFPose.position.y = newTFPose.position.y +10
    #responde = srvSetPose(vehicleTarget_handle,-1,newTFPose)
   # print 'tf: ', getListenerPose(tf_sub,'BoxTF_red1',"map"),'\n'
   # print 'vrep: ',srvPose(RedCube_handle,-1).pose.pose
    #time.sleep(5)
   # print 'tf: \n', getListenerPose(tf_sub,vehicleName,"map"),'\n'
    #print 'vrep: ENDPOSE: ',srvPose(vehicle_handle,-1).pose.pose
   # print getListenerPose(tf_sub,'BoxTF_yellow1',"map")
   # print getListenerPose(tf_sub,'BoxTF_yellow2',"map")
   # print getListenerPose(tf_sub,'BoxTF_yellow3',"map")
   
   
#    time.sleep(5)
#    newTFPose= getBoxAdjustedMatrixAndFacingAngleTF(tf_sub,'BoxTF_yellow2',vehicleName)
#    newTFPose.position.x = newTFPose.position.x*dist1
#    newTFPose.position.y = newTFPose.position.y*dist1
#    #print 'targetPos*Dist',newTFPose
    #responde = srvSetPose(vehicleTarget_handle,-1,newTFPose)
#    time.sleep(5)
#    newTFPose= getBoxAdjustedMatrixAndFacingAngleTF(tf_sub,'DropPlaceTF_3Goal',vehicleName)
#    newTFPose.position.x = newTFPose.position.x*dist1
#    newTFPose.position.y = newTFPose.position.y*dist1
    #print 'targetPos*Dist',newTFPose
   # responde = srvSetPose(vehicleTarget_handle,-1,newTFPose)

    #try:
        #handle = rospy.ServiceProxy("vrep/simRosGetObjectHandle","BoxTF_red1")
        #result, pose = rospy.ServiceProxy("/vrep/simRosGetObjectPose",'redRectangle1',-1)
        #print pose
        #set_result = rospy.ServiceProxy("/vrep/simRosSetObjectPose",pose)
        #set_result = rospy.ServiceProxy("/vrep/simRosSetObjectPosition", "youBot_vehicleTargetNamePosition",-1,{3,2,0})
               
    #    except rospy.ServiceException, e:
     #          print "Service call failed: %s"%e

    
   # rospy.spin() # node needs to keep running, so the messages come through
        
        
        
        
    #stopp Simulator   
    time.sleep(5) #just wait, a pause
    rospy.wait_for_service('/vrep/simRosStopSimulation')
    #stopSimulation = rospy.ServiceProxy('/vrep/simRosStopSimulation',simRosStopSimulation)
   # resSimStop = stopSimulation()  
    
  #  if -1 ==  (resSimStop.result):
  #      print 'Simulator could not b stoped value:', resSimStop
  #  else :# if 0
  #      print 'Simulator stopped value:',resSimStop
  #  print('Program ended')


