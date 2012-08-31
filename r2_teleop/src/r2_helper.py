#!/usr/bin/env python

"""
Copyright (c) 2012, General Motors, Co.
All rights reserved.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; 
roslib.load_manifest('r2_teleop')
#roslib.load_manifest("r2_controllers")
roslib.load_manifest("r2_gazebo")
#roslib.load_manifest("r2_sensing")
import rospy
import math
import time
import PyKDL as kdl

import marker_helper

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from r2_msgs.srv import *
#from r2_gazebo.srv import *
#from r2_sensing.srv import *

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

TORAD = math.pi/180.0
TODEG = 1.0/TORAD
    
# pose topics
left_pose_pub  = rospy.Publisher('r2_controller/left/pose_command',  PoseStamped)
right_pose_pub = rospy.Publisher('r2_controller/right/pose_command', PoseStamped)
gaze_pose_pub  = rospy.Publisher('r2_controller/gaze/pose_command',  PoseStamped)

# joint topics
left_jnt_pub  = rospy.Publisher('r2_controller/left_arm/joint_command',  JointState)
right_jnt_pub = rospy.Publisher('r2_controller/right_arm/joint_command', JointState)
neck_jnt_pub  = rospy.Publisher('r2_controller/neck/joint_command',      JointState)
waist_jnt_pub = rospy.Publisher('r2_controller/waist/joint_command',     JointState)

# meshes
left_palm_mesh = "package://r2_description/meshes/Left_Palm.dae"
left_thumb_carp_mesh = "package://r2_description/meshes/Left_Thumb_Carp.dae"
left_thumb_carp_mtcar = "package://r2_description/meshes/Left_Thumb_MtCar.dae"

right_palm_mesh = "package://r2_description/meshes/Right_Palm.dae"
right_thumb_carp_mesh = "package://r2_description/meshes/Left_Thumb_Carp.dae"
right_thumb_carp_mtcar = "package://r2_description/meshes/Right_Thumb_MtCar.dae"

thumb_prox_mesh = "package://r2_description/meshes/Thumb_Proximal.dae"
thumb_dist_mesh = "package://r2_description/meshes/Thumb_Dist.dae"
finger_prox_mesh = "package://r2_description/meshes/Finger_Proximal.dae"
finger_mid_mesh = "package://r2_description/meshes/Finger_Mid.dae"
finger_dist_mesh = "package://r2_description/meshes/Finger_Dist.dae"

body_mesh = "package://r2_description/meshes/Body_Cover.dae"
head_mesh = "package://r2_description/meshes/Head.dae"
backpack_mesh = "package://r2_description/meshes/Backpack.dae"

# mesh poses
left_palm_mesh_pose = Pose()
right_palm_mesh_pose = Pose()
waist_mesh_pose = Pose()
head_mesh_pose = Pose()
backpack_mesh_pose = Pose()

# service calls
def SetArmsToCartMode(left_frame, right_frame) :
    SetArmToCartMode('left', left_frame)
    SetArmToCartMode('right', right_frame)

def SetArmToCartMode(arm, frame) :

    rospy.wait_for_service('r2_controller/set_tip_name')
    set_tip_name = rospy.ServiceProxy('r2_controller/set_tip_name', SetTipName)
    
    print "setting ", arm, " tip to: ", frame
    try:
        resp1 = set_tip_name(arm, frame)
        print "Set Tip Name: ", resp1.result
    except rospy.ServiceException, e: 
        print "Service call failed for ", arm, " arm: %s"%e
 
def SetArmToJointMode(arm) :

    rospy.wait_for_service('r2_controller/set_joint_mode')
    set_joint_mode = rospy.ServiceProxy('r2_controller/set_joint_mode', SetJointMode)
    
    print "setting ", arm, " to joint mode"
    try:
        resp1 = set_joint_mode(arm)
        print "Set ", arm, " joint mode: ", resp1.result
    except rospy.ServiceException, e: 
        print "Service call failed for ", arm, " arm: %s"%e

def SetPower(p) :

    rospy.wait_for_service('r2_controller/power')
    power = rospy.ServiceProxy('r2_controller/power', Power)
    
    print "setting motor power to: ", p
    try:
        resp1 = power('motor', p)
        print "Set Power: ", resp1.status
    except rospy.ServiceException, e: 
        print "Service call failed for setting power: %s"%e

    return resp1.status

def Servo(s, mode) :

    rospy.wait_for_service('r2_controller/servo')
    Servo = rospy.ServiceProxy('r2_controller/servo', Servo)
    
    print "servoing: ", s, " in mode: ", m
    try:
        resp1 = servo(s, mode)
        print "Servo Status: ", resp1.status
    except rospy.ServiceException, e: 
        print "Service call failed for servoing: %s"%e

    return resp1.status

def SegmentTableTop() :

    rospy.wait_for_service('tabletop_segmentation/take_snapshot')
    take_snapshot = rospy.ServiceProxy('tabletop_segmentation/take_snapshot', TakeSnapshot)
    
    print "taking snapshot"
    try:
        resp1 = take_snapshot(True)
    except rospy.ServiceException, e: 
        print "Service call failed for taking snapshot %s"%e

def ParseTableScene() :

    rospy.wait_for_service('tabletop_clustering/parse_scene')
    parse_scene = rospy.ServiceProxy('tabletop_clustering/parse_scene', ParseTableScene)
    
    rospy.loginfo("parsing table scene")
    try:
        resp1 = parse_scene(True)
        print "response : ", resp
    except rospy.ServiceException, e: 
        print "Service call failed for parsing table scene %s"%e


# poses
leftCartReadyPose = PoseStamped()
leftCartReadyPose.header.seq = 0
leftCartReadyPose.header.stamp = 0
leftCartReadyPose.header.frame_id = "r2/waist_center"
leftCartReadyPose.pose.position.x = 0.27
leftCartReadyPose.pose.position.y = -0.35
leftCartReadyPose.pose.position.z = -.22
leftCartReadyPose.pose.orientation.x = -0.707
leftCartReadyPose.pose.orientation.y = 0
leftCartReadyPose.pose.orientation.z = 0
leftCartReadyPose.pose.orientation.w = 0.707

rightCartReadyPose = PoseStamped()
rightCartReadyPose.header.seq = 0
rightCartReadyPose.header.stamp = 0
rightCartReadyPose.header.frame_id = "r2/waist_center"
rightCartReadyPose.pose.position.x = 0.27
rightCartReadyPose.pose.position.y = 0.35
rightCartReadyPose.pose.position.z = -.22
rightCartReadyPose.pose.orientation.x = 0.707
rightCartReadyPose.pose.orientation.y = 0
rightCartReadyPose.pose.orientation.z = 0
rightCartReadyPose.pose.orientation.w = 0.707

# joint states
leftArmJointNames = ['/r2/left_arm/joint'+str(i) for i in range(7)] 
leftThumbJointNames = ['/r2/left_arm/hand/thumb/joint' +str(i) for i in range(4)]
leftIndexJointNames = ['/r2/left_arm/hand/index/joint'+str(i) for i in range(3)]
leftMiddleJointNames = ['/r2/left_arm/hand/middle/joint'+str(i) for i in range(3)]
leftRingJointNames = ['/r2/left_arm/hand/ring/joint'+str(i) for i in range(1)]
leftLittleJointNames = ['/r2/left_arm/hand/little/joint'+str(i) for i in range(1)]
leftHandNames = leftThumbJointNames + leftIndexJointNames + leftMiddleJointNames + leftRingJointNames + leftLittleJointNames
leftJointNames = leftArmJointNames + leftHandNames

rightArmJointNames = ['/r2/right_arm/joint'+str(i) for i in range(7)] 
rightThumbJointNames = ['/r2/right_arm/hand/thumb/joint'+str(i) for i in range(4)]
rightIndexJointNames = ['/r2/right_arm/hand/index/joint'+str(i) for i in range(3)]
rightMiddleJointNames = ['/r2/right_arm/hand/middle/joint'+str(i) for i in range(3)]
rightRingJointNames = ['/r2/right_arm/hand/ring/joint'+str(i) for i in range(1)]
rightLittleJointNames = ['/r2/right_arm/hand/little/joint'+str(i) for i in range(1)]
rightHandNames = rightThumbJointNames + rightIndexJointNames + rightMiddleJointNames + rightRingJointNames + rightLittleJointNames
rightJointNames = rightArmJointNames + rightHandNames

neckJointNames = ['/r2/neck/joint0', '/r2/neck/joint1', '/r2/neck/joint2']
waistJointNames = ['/r2/waist/joint0']

leftJointReadyPose = JointState()
rightJointReadyPose = JointState()
neckJointReadyPose = JointState()
waistJointReadyPose = JointState()

leftJointReadyPose.header.seq = 0
leftJointReadyPose.header.stamp = 0
leftJointReadyPose.header.frame_id = "r2/waist_center"
leftJointReadyPose.name = leftJointNames
leftJointReadyPose.position = [50.0*TORAD, -80.0*TORAD, -105.0*TORAD, -140.0*TORAD, 80.0*TORAD, 0.0*TORAD, 0.0*TORAD]+[0*TORAD]*12

rightJointReadyPose.header.seq = 0
rightJointReadyPose.header.stamp = 0
rightJointReadyPose.header.frame_id = "world"
rightJointReadyPose.name = rightJointNames
rightJointReadyPose.position = [-50.0*TORAD, -80.0*TORAD, 105.0*TORAD, -140.0*TORAD, -80.0*TORAD, 0.0*TORAD, 0.0*TORAD]+[0*TORAD]*12
   
neckJointReadyPose.header.seq = 0
neckJointReadyPose.header.stamp = 0
neckJointReadyPose.header.frame_id = "world"
neckJointReadyPose.name = neckJointNames
neckJointReadyPose.position = [0.0*TORAD, 0.0*TORAD, -5.0*TORAD]

waistJointReadyPose.header.seq = 0
waistJointReadyPose.header.stamp = 0
waistJointReadyPose.header.frame_id = "world"
waistJointReadyPose.name = waistJointNames
waistJointReadyPose.position = [180.0*TORAD]

leftHandClose = JointState()
leftHandClose.header.stamp = 0
leftHandClose.header.frame_id = "world"
leftHandClose.name = leftHandNames
leftHandClose.position = [-70*TORAD, 50*TORAD, 50*TORAD, 0*TORAD, 0*TORAD, 90*TORAD, 90*TORAD, 0*TORAD, 90*TORAD, 80*TORAD, 170*TORAD, 170*TORAD]

rightHandClose = JointState()
rightHandClose.header.stamp = 0
rightHandClose.header.frame_id = "world"
rightHandClose.name = rightHandNames
rightHandClose.position = [70*TORAD, 50*TORAD, 50*TORAD, 0*TORAD, 0*TORAD, 90*TORAD, 90*TORAD, 0*TORAD, 90*TORAD, 80*TORAD, 170*TORAD, 170*TORAD]

leftHandOpen = JointState()
leftHandOpen.header.stamp = 0
leftHandOpen.header.frame_id = "world"
leftHandOpen.name = leftHandNames
leftHandOpen.position = [0*TORAD]*12

rightHandOpen = JointState()
rightHandOpen.header.stamp = 0
rightHandOpen.header.frame_id = "world"
rightHandOpen.name = rightHandNames
rightHandOpen.position = [0*TORAD]*12


leftHandPowerClose = JointState()
#leftHandPowerClose.header.stamp = 0
leftHandPowerClose.header.frame_id = "world"
leftHandPowerClose.name = leftHandNames
leftHandPowerClose.position = [-100*TORAD, 0*TORAD, 0*TORAD, 0*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 170*TORAD, 170*TORAD]

rightHandPowerClose = JointState()
#rightHandPowerClose.header.stamp = 0
rightHandPowerClose.header.frame_id = "world"
rightHandPowerClose.name = rightHandNames
rightHandPowerClose.position = [100*TORAD, 0*TORAD, 0*TORAD, 0*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 150*TORAD, 150*TORAD]


leftHandPowerCloseThumb = JointState()
#leftHandPowerCloseThumb.header.stamp = 0
leftHandPowerCloseThumb.header.frame_id = "world"
leftHandPowerCloseThumb.name = leftHandNames
leftHandPowerCloseThumb.position = [-200*TORAD, 100*TORAD, 100*TORAD, 50*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 170*TORAD, 170*TORAD]

rightHandPowerCloseThumb = JointState()
#rightHandPowerCloseThumb.header.stamp = 0
rightHandPowerCloseThumb.header.frame_id = "world"
rightHandPowerCloseThumb.name = rightHandNames
rightHandPowerCloseThumb.position = [200*TORAD, 100*TORAD, 100*TORAD, 50*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 0*TORAD, 100*TORAD, 70*TORAD, 150*TORAD, 150*TORAD]



def SetUpMeshData() :

    lq = kdl.Rotation.RPY(3.14, 0, 1.57).GetQuaternion()
    left_palm_mesh_pose.position.x = 0.0
    left_palm_mesh_pose.position.y = 0.0
    left_palm_mesh_pose.position.z = 0.0
    left_palm_mesh_pose.orientation.x = lq[0]
    left_palm_mesh_pose.orientation.y = lq[1]
    left_palm_mesh_pose.orientation.z = lq[2]
    left_palm_mesh_pose.orientation.w = lq[3]

    rq = kdl.Rotation.RPY(3.14, 0, 1.57).GetQuaternion()
    right_palm_mesh_pose.position.x = 0.0
    right_palm_mesh_pose.position.y = 0.0
    right_palm_mesh_pose.position.z = 0.0
    right_palm_mesh_pose.orientation.x = rq[0]
    right_palm_mesh_pose.orientation.y = rq[1]
    right_palm_mesh_pose.orientation.z = rq[2]
    right_palm_mesh_pose.orientation.w = rq[3]
 
    wq = kdl.Rotation.RPY(-1.57, 0, -1.57).GetQuaternion()
    waist_mesh_pose.position.x = 0.02
    waist_mesh_pose.position.y = 0.05
    waist_mesh_pose.position.z = -0.5625
    waist_mesh_pose.orientation.x = wq[0]
    waist_mesh_pose.orientation.y = wq[1]
    waist_mesh_pose.orientation.z = wq[2]
    waist_mesh_pose.orientation.w = wq[3]

    bq = kdl.Rotation.RPY(0, 0, 0.25).GetQuaternion()
    backpack_mesh_pose.position.x = 0.0
    backpack_mesh_pose.position.y = 0.0
    backpack_mesh_pose.position.z = -0.54
    backpack_mesh_pose.orientation.x = bq[0]
    backpack_mesh_pose.orientation.y = bq[1]
    backpack_mesh_pose.orientation.z = bq[2]
    backpack_mesh_pose.orientation.w = bq[3]

    hq = kdl.Rotation.RPY(0, 1.57, 0).GetQuaternion()
    head_mesh_pose.position.x = 0.0
    head_mesh_pose.position.y = 0.07
    head_mesh_pose.position.z = 0.0
    head_mesh_pose.orientation.x = hq[0]
    head_mesh_pose.orientation.y = hq[1]
    head_mesh_pose.orientation.z = hq[2]
    head_mesh_pose.orientation.w = hq[3]


def getJointCommand(jd, name, d) :
    jnt_cmd = JointState()
    j_act = jd.position[jd.name.index(name)]
    j_ref = j_act + d
    jnt_cmd.header.seq = 0
    jnt_cmd.header.stamp = rospy.get_rostime()
    jnt_cmd.header.frame_id = "r2/robot_base"
    jnt_cmd.name = [name]
    jnt_cmd.position = [j_ref]
    return jnt_cmd

def makeLeftHandSetpointMarker ( msg ):
    control =  InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.markers.append( marker_helper.makeMesh(msg, left_palm_mesh, left_palm_mesh_pose, 1.1) )
    msg.controls.append( control )
    return control

def makeRightHandSetpointMarker ( msg ):
    control =  InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.markers.append( marker_helper.makeMesh(msg, right_palm_mesh, right_palm_mesh_pose, 1.1) )
    msg.controls.append( control )
    return control

