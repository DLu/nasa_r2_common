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
roslib.load_manifest("r2_teleop")
import rospy
import math
import copy
import PyKDL as kdl

import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

import kdl_posemath as pm
import r2_helper as r2
import frames
from marker_helper import *

TORAD = math.pi/180.0
TODEG = 1.0/TORAD

server = None
counter = 0

joint_mode = [False, False]
edit_tool = [False, False]
edit_setpoint = [False, False]

gaze_control_mode = False
gaze_tracking_mode = False
gaze_point_tracking_mode = False
gaze_left_tracking_mode = False
gaze_right_tracking_mode = False

power_mode = False

left_arm_menu_handler = MenuHandler()
right_arm_menu_handler = MenuHandler()
waist_menu_handler = MenuHandler()
head_menu_handler = MenuHandler()
gaze_menu_handler = MenuHandler()
backpack_menu_handler = MenuHandler()
left_setpoint_menu_handler = MenuHandler()
right_setpoint_menu_handler = MenuHandler()

tool_offset = [Pose(), Pose()]
setpoint_offset = [Pose(), Pose()]
setpoint_store = [PoseStamped(), PoseStamped()]
joint_data = JointState()

left_arm_cart_marker = InteractiveMarker()
right_arm_cart_marker = InteractiveMarker()
neck_lower_pitch_marker = InteractiveMarker()
neck_roll_marker = InteractiveMarker()
neck_upper_pitch_marker = InteractiveMarker()
left_posture_marker = InteractiveMarker()
right_posture_marker = InteractiveMarker()

left_finger_markers = [InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker()]  
right_finger_markers = [InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker(), InteractiveMarker()]  

gaze_marker = InteractiveMarker()
backpack_marker = InteractiveMarker()
left_setpoint_marker = InteractiveMarker()
right_setpoint_marker = InteractiveMarker()

gaze_sphere_scale = 0.5

fast_update_rate = 10
slow_update_divider = 10
dur_time = 2.0

tf_listener = None

def resetMarker( feedback, frame_id ):    
    pose = feedback.pose
    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    server.setPose( frame_id, pose )
    server.applyChanges()

def ResetToolOffset(arm) :
    global tool_offset
    if arm == "left" :
        tool_offset[0] = Pose()
        tool_offset[0].orientation.w = 1
    if arm == "right" :
        tool_offset[1] = Pose()
        tool_offset[1].orientation.w = 1

def ResetSetpointOffset(arm) :
    global setpoint_offset
    if arm == "left" :
        setpoint_offset[0] = Pose()
        setpoint_offset[0].orientation.w = 1
    if arm == "right" :
        setpoint_offset[1] = Pose()
        setpoint_offset[1].orientation.w = 1

def handleLeftArmMenu( feedback ) :
    global edit_tool, edit_setpoint
    handle = feedback.menu_entry_id

    if(handle == 1) : # ready pose
        r2.leftCartReadyPose.header.seq = r2.leftCartReadyPose.header.seq + 1
        r2.leftCartReadyPose.header.stamp = rospy.get_rostime()
        r2.left_pose_pub.publish(r2.leftCartReadyPose)
        r2.leftJointReadyPose.header.seq = r2.leftJointReadyPose.header.seq + 1
        r2.leftJointReadyPose.header.stamp = rospy.get_rostime()
        r2.left_jnt_pub.publish(r2.leftJointReadyPose) 

    elif(handle == 2) : # cart/joint mode
        state = left_arm_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            left_arm_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("Setting Cartesian Control Mode for Left Arm")
            r2.SetArmToCartMode('left', frames.control_frame_id[0])
            joint_mode[0] = False
            makeLeftArmControl( )
        else:
            left_arm_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("Setting Joint Control Mode for Left Arm")
            r2.SetArmToJointMode('left')
            joint_mode[0] = True
            removeLeftArmControl()

    elif(handle == 3) : # tool offset
        state = left_arm_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            left_arm_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            edit_tool[0] = False
            rospy.loginfo("Fixing Left Arm tool offset")
        else:
            edit_tool[0] = True
            left_arm_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("Moving Left Arm tool offset ")
    
    elif(handle == 4) : # Setpoint
        state = left_arm_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            left_arm_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("Resetting Left Arm Reach Point")
            removeLeftSetpointControl()
            makeLeftArmControl()
            edit_setpoint[0] = False
        else:
            left_arm_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("Setting Left Arm Reach Point ")
            removeLeftArmControl()
            ResetSetpointOffset("left")
            makeLeftSetpointControl()
            edit_setpoint[0] = True
            
    elif(handle == 6) :  # open hand
        print "open left hand"
        r2.leftHandOpen.header.seq = r2.leftHandOpen.header.seq + 1
        r2.leftHandOpen.header.stamp = rospy.get_rostime()
        r2.left_jnt_pub.publish(r2.leftHandOpen) 
        resetMarker(feedback, frames.control_marker_id[0])

    elif(handle == 7) : # close hand
        print "close left hand"
        r2.leftHandClose.header.seq = r2.leftHandClose.header.seq + 1
        r2.leftHandClose.header.stamp = rospy.get_rostime()
        r2.left_jnt_pub.publish(r2.leftHandClose) 
        resetMarker(feedback, frames.control_marker_id[0])

    elif(handle == 8) : # reset tool offset
        print "Reseting Left Tool Control Frame"
        ResetToolOffset("left")
        resetMarker(feedback, frames.control_marker_id[0])
    
    left_arm_menu_handler.reApply( server )


def handleRightArmMenu( feedback ) :
    global edit_tool, edit_setpoint
    handle = feedback.menu_entry_id

    if(handle == 1) :
        r2.rightCartReadyPose.header.seq = r2.rightCartReadyPose.header.seq + 1
        r2.rightCartReadyPose.header.stamp = rospy.get_rostime()
        r2.right_pose_pub.publish(r2.rightCartReadyPose)
        r2.rightJointReadyPose.header.seq = r2.rightJointReadyPose.header.seq + 1
        r2.rightJointReadyPose.header.stamp = rospy.get_rostime()
        r2.right_jnt_pub.publish(r2.rightJointReadyPose) 

    elif(handle == 2) :
        state = right_arm_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            right_arm_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("Setting Cartesian Control Mode for Right Arm")
            r2.SetArmToCartMode('right', frames.control_frame_id[1])
            joint_mode[1] = False
            makeRightArmControl( )
        else:
            right_arm_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("Setting Joint Control Mode for Right Arm")
            r2.SetArmToJointMode('right')
            joint_mode[1] = True
            removeRightArmControl()

    elif(handle == 3) :
        state = right_arm_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            right_arm_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            edit_tool[1] = False
            rospy.loginfo("Fixing Right Arm tool offset")
        else:
            edit_tool[1] = True
            right_arm_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("Moving Right Arm tool offset ")

    elif(handle == 4) :
        state = right_arm_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            right_arm_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("Resetting Right Arm Reach Point")
            removeRightSetpointControl()
            makeRightArmControl()
            edit_setpoint[1] = False
        else:
            right_arm_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("Setting Right Arm Reach Point ")
            removeRightArmControl()
            ResetSetpointOffset("right")
            makeRightSetpointControl()
            edit_setpoint[1] = True

    elif(handle == 6) :
        print "open right hand"
        r2.rightHandOpen.header.seq = r2.rightHandOpen.header.seq + 1
        r2.rightHandOpen.header.stamp = rospy.get_rostime()
        r2.right_jnt_pub.publish(r2.rightHandOpen) 
        resetMarker(feedback, frames.control_marker_id[1])

    elif(handle == 7) :
        print "close right hand"
        r2.rightHandClose.header.seq = r2.rightHandClose.header.seq + 1
        r2.rightHandClose.header.stamp = rospy.get_rostime()
        r2.right_jnt_pub.publish(r2.rightHandClose) 
        resetMarker(feedback, frames.control_marker_id[1])

    elif(handle == 8) :
        print "Reseting Right Tool Control Frame"
        ResetToolOffset("right")
        resetMarker(feedback, frames.control_marker_id[1])

    right_arm_menu_handler.reApply( server )


def handleWaistMenu( feedback ) :
    if(feedback.menu_entry_id == 1) :
        r2.waistJointReadyPose.header.seq = r2.waistJointReadyPose.header.seq + 1
        r2.waistJointReadyPose.header.stamp = rospy.get_rostime()
        r2.waist_jnt_pub.publish(r2.waistJointReadyPose)

def handleBackpackMenu( feedback ) :
    global power_mode
    handle = feedback.menu_entry_id
    if(feedback.menu_entry_id == 1) :
        state = backpack_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            backpack_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("turning off power")
            power_mode = False
        else:
            backpack_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("turning on power")
            power_mode = True
        r2.SetPower(power_mode)
        backpack_menu_handler.reApply( server )

def handleHeadMenu( feedback ) :
    handle = feedback.menu_entry_id
    if(handle == 1) :
        r2.neckJointReadyPose.header.seq = r2.neckJointReadyPose.header.seq + 1
        r2.neckJointReadyPose.header.stamp = rospy.get_rostime()
        r2.neck_jnt_pub.publish(r2.neckJointReadyPose)
        resetMarker(feedback, feedback.marker_name)  
    elif(handle == 2) :
        state = head_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            head_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("turning off head joint control")
            removeHeadControl()
        else:
            head_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("turning on head joint control")
            makeHeadControl()
        head_menu_handler.reApply( server )
    elif(feedback.menu_entry_id == 3) :
        toggleGazeControl()
    elif(feedback.menu_entry_id == 4) :
        r2.SegmentTableTop()


def handleGazeMenu( feedback ) :
    global gaze_menu_handler
    global gaze_tracking_mode, gaze_point_tracking_mode, gaze_left_tracking_mode, gaze_right_tracking_mode
    pose = PoseStamped()
    handle = feedback.menu_entry_id
   
    if(handle == 1) :
        pose.header.seq = pose.header.seq 
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = frames.base_frame_id
        pose.pose = feedback.pose
        r2.gaze_pose_pub.publish(pose)
        gaze_menu_handler.reApply( server )
    
    elif(handle == 3) :
        state = gaze_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            gaze_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("turning off gaze point tracking control")
            gaze_tracking_mode = False
            gaze_point_tracking_mode = False
            removeGazeControl()
        else:
            gaze_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            rospy.loginfo("turning on gaze point tracking control")
            gaze_menu_handler.setCheckState( handle+1, MenuHandler.UNCHECKED )
            gaze_menu_handler.setCheckState( handle+2, MenuHandler.UNCHECKED )
            gaze_tracking_mode = True
            gaze_point_tracking_mode = True
            gaze_left_tracking_mode = False
            gaze_right_tracking_mode = False
        gaze_menu_handler.reApply( server )

    elif(handle == 4) :
        state = gaze_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            gaze_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("turning off gaze left tracking control")
            gaze_tracking_mode = False
            gaze_left_tracking_mode = False
        else:
            gaze_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            gaze_menu_handler.setCheckState( handle-1, MenuHandler.UNCHECKED )
            gaze_menu_handler.setCheckState( handle+1, MenuHandler.UNCHECKED )
            rospy.loginfo("turning on gaze left tracking control")
            gaze_tracking_mode = True
            gaze_point_tracking_mode = False
            gaze_left_tracking_mode = True
            gaze_right_tracking_mode = False
            removeGazeControl()
        gaze_menu_handler.reApply( server )

    elif(handle == 5) :
        state = gaze_menu_handler.getCheckState( handle )
        if state == MenuHandler.CHECKED:
            gaze_menu_handler.setCheckState( handle, MenuHandler.UNCHECKED )
            rospy.loginfo("turning off gaze right tracking control")
            gaze_tracking_mode = False
            gaze_right_tracking_mode = False
        else:
            gaze_menu_handler.setCheckState( handle, MenuHandler.CHECKED )
            gaze_menu_handler.setCheckState( handle-1, MenuHandler.UNCHECKED )
            gaze_menu_handler.setCheckState( handle-2, MenuHandler.UNCHECKED )
            rospy.loginfo("turning on gaze right tracking control")
            gaze_tracking_mode = True
            gaze_point_tracking_mode = False
            gaze_left_tracking_mode = False
            gaze_right_tracking_mode = True
            removeGazeControl()
        gaze_menu_handler.reApply( server )


def handleLeftSetpointMenu( feedback ) :
    global edit_setpoint
    pose = PoseStamped()
    handle = feedback.menu_entry_id
    now = rospy.Time(0)
    if(handle == 1) :
        #tf_listener.waitForTransform(frames.control_frame_id[0], frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0], now)
        pose.pose = pm.toPose(trans, rot) 

        Fp = pm.fromMsg(pose.pose)
        Fs = pm.fromMsg(setpoint_offset[0])
        #Ft = pm.fromMsg(tool_offset[0])

        pose.header.seq = 0
        pose.header.stamp = now
        pose.header.frame_id = frames.base_frame_id
        pose.pose = pm.toMsg(Fp * Fs )#* Ft.Inverse()) 
        
        # send setpoint to arm
        r2.left_pose_pub.publish(pose)

        # reset setpoint marker stuff
        removeLeftSetpointControl()
        makeLeftArmControl()
        left_arm_menu_handler.setCheckState( 4, MenuHandler.UNCHECKED )
        left_arm_menu_handler.reApply( server )
        edit_setpoint[0] = False

def handleRightSetpointMenu( feedback ) :
    global edit_setpoint
    pose = PoseStamped()
    handle = feedback.menu_entry_id
    now = rospy.Time(0)
    if(handle == 1) :
        #tf_listener.waitForTransform(frames.control_frame_id[1], frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[1], now)
        pose.pose = pm.toPose(trans, rot) 
        
        Fp = pm.fromMsg(pose.pose)
        Fs = pm.fromMsg(setpoint_offset[1])
        #Ft = pm.fromMsg(tool_offset[1])

        pose.header.seq = 0
        pose.header.stamp = now
        pose.header.frame_id = frames.base_frame_id
        pose.pose = pm.toMsg(Fp * Fs)# * Ft.Inverse()) 
        
        # send setpoint to arm
        r2.right_pose_pub.publish(pose)

        # reset setpoint marker stuff
        removeRightSetpointControl()
        makeRightArmControl()
        right_arm_menu_handler.setCheckState( 4, MenuHandler.UNCHECKED )
        right_arm_menu_handler.reApply( server )
        edit_setpoint[1] = False

def toggleGazeControl() :
    global gaze_control_mode, gaze_tracking_mode, gaze_point_tracking_mode, gaze_left_tracking_mode, gaze_right_tracking_mode
    gaze_control_mode = not gaze_control_mode
    if gaze_control_mode :
        makeGazeControl()
        gaze_tracking_mode = False
        gaze_point_tracking_mode = False
        gaze_left_tracking_mode = False
        gaze_right_tracking_mode = False
        gaze_menu_handler.setCheckState( 3, MenuHandler.UNCHECKED )
        gaze_menu_handler.setCheckState( 4, MenuHandler.UNCHECKED )
        gaze_menu_handler.setCheckState( 5, MenuHandler.UNCHECKED )    
    else :
        removeGazeControl()
        gaze_tracking_mode = False
        gaze_point_tracking_mode = False
        gaze_left_tracking_mode = False
        gaze_right_tracking_mode = False
        gaze_menu_handler.setCheckState( 3, MenuHandler.UNCHECKED )
        gaze_menu_handler.setCheckState( 4, MenuHandler.UNCHECKED )
        gaze_menu_handler.setCheckState( 5, MenuHandler.UNCHECKED )    
    gaze_menu_handler.reApply( server )

def poseUpdate( feedback ) :
    global joint_data, tool_offset, setpoint_offset
    pose = PoseStamped()
   
    if feedback.marker_name == frames.gaze_frame_id and gaze_tracking_mode:
        pose.header.seq = pose.header.seq 
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = frames.base_frame_id
        pose.pose = copy.deepcopy(feedback.pose)
        server.setPose( feedback.marker_name, pose.pose )
        server.applyChanges()
        r2.gaze_pose_pub.publish(pose)


def mouseUpdate( feedback ) :

    print "marker_name: ", feedback.marker_name
    global joint_data, tool_offset, setpoint_offset
    
    pose = PoseStamped()
    stamped = PoseStamped()
    now = rospy.get_rostime()
    
    pose.header.seq = 0
    pose.header.stamp = now
    pose.header.frame_id = feedback.header.frame_id;
    
    
    wTh = Pose()
    wTm = Pose()
    jnt_cmd = JointState()
    now = rospy.Time(0)

    if feedback.marker_name == frames.control_marker_id[0] and edit_tool[0] :
        tf_listener.waitForTransform(frames.control_frame_id[0], frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0], now)
        wTh = pm.toPose(trans, rot) 
        wTm = feedback.pose
        Fwth = pm.fromMsg(wTh)
        Fwtm = pm.fromMsg(wTm)
        Fhtw = (pm.fromMsg(wTh)).Inverse()
        Fhtm = Fhtw*Fwtm
        tool_offset[0] = pm.toMsg(Fhtm) 
        print "Moved left tool frame: ", tool_offset[0]
 
    elif feedback.marker_name == frames.control_marker_id[1] and edit_tool[1] :
        tf_listener.waitForTransform(frames.control_frame_id[1], frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[1], now)
        wTh = pm.toPose(trans, rot)
        Fwth = pm.fromMsg(wTh)
        wTm = feedback.pose
        Fwtm = pm.fromMsg(wTm)
        Fhtw = (pm.fromMsg(wTh)).Inverse()
        Fhtm = Fhtw*Fwtm
        tool_offset[1] = pm.toMsg(Fhtm) 
        print "Moved right tool frame: ", tool_offset[1]

    elif feedback.marker_name == frames.control_marker_id[0] :
        pose.pose = feedback.pose
        tf_listener.waitForTransform(frames.control_frame_id[0], frames.base_frame_id, now, rospy.Duration(dur_time))
        ps = tf_listener.transformPose(frames.base_frame_id, pose)
        Fp = pm.fromMsg(ps.pose)
        Fo = pm.fromMsg(tool_offset[0])
        # apply offset (inverse) and send to robot   
        pose.pose = pm.toMsg(Fp * Fo.Inverse()) 
        r2.left_pose_pub.publish(pose)
    
    elif feedback.marker_name == frames.control_marker_id[1] :
        pose.pose = feedback.pose
        tf_listener.waitForTransform(frames.control_frame_id[1], frames.base_frame_id, now, rospy.Duration(dur_time))
        ps = tf_listener.transformPose(frames.base_frame_id, pose)
        Fp = pm.fromMsg(ps.pose)
        Fo = pm.fromMsg(tool_offset[1])
        # apply offset (inverse) and send to robot   
        pose.pose = pm.toMsg(Fp * Fo.Inverse()) 
        r2.right_pose_pub.publish(pose)

    elif feedback.marker_name == frames.gaze_frame_id and gaze_tracking_mode:
        pose.pose = feedback.pose
        r2.gaze_pose_pub.publish(pose)

    elif feedback.marker_name == frames.setpoint_marker_id[0] and edit_setpoint[0] :
        #tf_listener.waitForTransform(frames.control_frame_id[0], frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0], now)
        wTh = pm.toPose(trans, rot)
        Fwth = pm.fromMsg(wTh)
        wTs = feedback.pose
        Fwts = pm.fromMsg(wTs)
        Fhtw = (pm.fromMsg(wTh)*pm.fromMsg(tool_offset[0])).Inverse()
        Fhts = Fhtw*Fwts
        setpoint_offset[0] = pm.toMsg(Fhts) 
        print "Moved left setpoint frame: ", setpoint_offset[0]

    elif feedback.marker_name == frames.setpoint_marker_id[1] and edit_setpoint[1] :
        #tf_listener.waitForTransform(frames.control_frame_id[1], frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[1], now)
        wTh = pm.toPose(trans, rot)  
        Fwth = pm.fromMsg(wTh)
        wTs = feedback.pose
        Fwts = pm.fromMsg(wTs)
        Fhtw = (pm.fromMsg(wTh)*pm.fromMsg(tool_offset[1])).Inverse()
        Fhts = Fhtw*Fwts
        setpoint_offset[1] = pm.toMsg(Fhts) 
        print "Moved right setpoint frame: ", setpoint_offset[1]

    elif feedback.marker_name == frames.waist_frame_id :
        r2.waist_jnt_pub.publish(r2.getJointCommand(joint_data, '/r2/waist/joint0', feedback.pose.orientation.z))
        resetMarker(feedback, feedback.marker_name)  
    
    elif feedback.marker_name == frames.neck_frame_id[0] :
        r2.neck_jnt_pub.publish(r2.getJointCommand(joint_data, '/r2/neck/joint0', -feedback.pose.orientation.y))
        resetMarker(feedback, feedback.marker_name)  

    elif feedback.marker_name == frames.neck_frame_id[1] :
        r2.neck_jnt_pub.publish(r2.getJointCommand(joint_data, '/r2/neck/joint1', feedback.pose.orientation.y))
        resetMarker(feedback, feedback.marker_name)  

    elif feedback.marker_name == frames.neck_frame_id[2] :
        r2.neck_jnt_pub.publish(r2.getJointCommand(joint_data, '/r2/neck/joint2', feedback.pose.orientation.y))
        resetMarker(feedback, feedback.marker_name)  

    elif feedback.marker_name == frames.posture_frame_id[0] :
        r2.left_jnt_pub.publish(r2.getJointCommand(joint_data, '/r2/left_arm/joint1', feedback.pose.orientation.z))
        resetMarker(feedback, feedback.marker_name)  
    
    elif feedback.marker_name == frames.posture_frame_id[1] :
        r2.right_jnt_pub.publish(r2.getJointCommand(joint_data, '/r2/right_arm/joint1', feedback.pose.orientation.z))
        resetMarker(feedback, feedback.marker_name)  
    
    for i in range(len(r2.leftHandNames)) :
        if feedback.marker_name == frames.finger_frame_id[0][i] :
            r2.left_jnt_pub.publish(r2.getJointCommand(joint_data, r2.leftHandNames[i], feedback.pose.orientation.z))
            resetMarker(feedback, feedback.marker_name)  

    for i in range(len(r2.rightHandNames)) :
        if feedback.marker_name == frames.finger_frame_id[1][i] :
            r2.right_jnt_pub.publish(r2.getJointCommand(joint_data, r2.rightHandNames[i], feedback.pose.orientation.z))
            resetMarker(feedback, feedback.marker_name)  

def processFeedback( feedback ) :
         
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo( s + ": button click" + mp + "." )

    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." + "marker_name: " + feedback.marker_name  )
        if feedback.marker_name == frames.menu_frame_id[0]:
            handleLeftArmMenu(feedback)
        elif feedback.marker_name == frames.menu_frame_id[1] :
            handleRightArmMenu(feedback)
        elif feedback.marker_name == frames.waist_frame_id :
            handleWaistMenu(feedback)
        elif feedback.marker_name == frames.head_frame_id :
            handleHeadMenu(feedback)
        elif feedback.marker_name == frames.gaze_frame_id :
            handleGazeMenu(feedback)
        elif feedback.marker_name == frames.backpack_frame_id :
            handleBackpackMenu(feedback)
        elif feedback.marker_name == frames.setpoint_marker_id[0]:
            handleLeftSetpointMenu(feedback)
        elif feedback.marker_name == frames.setpoint_marker_id[1]:
            handleRightSetpointMenu(feedback)

    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        poseUpdate(feedback)

    #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    #    rospy.loginfo( s + ": mouse down" + mp + "." )
    
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        mouseUpdate(feedback)

    server.applyChanges()



#####################################################################
# Arm Control Marker Creation
def makeLeftArmControl( ):
    global left_arm_cart_marker
    left_arm_cart_marker.controls.append(makeXTransControl())
    left_arm_cart_marker.controls.append(makeYTransControl())
    left_arm_cart_marker.controls.append(makeZTransControl())
    left_arm_cart_marker.controls.append(makeXRotControl())
    left_arm_cart_marker.controls.append(makeYRotControl())
    left_arm_cart_marker.controls.append(makeZRotControl())
    server.insert(left_arm_cart_marker, processFeedback)

def makeRightArmControl( ):
    global right_arm_cart_marker
    right_arm_cart_marker.controls.append(makeXTransControl())
    right_arm_cart_marker.controls.append(makeYTransControl())
    right_arm_cart_marker.controls.append(makeZTransControl())
    right_arm_cart_marker.controls.append(makeXRotControl())
    right_arm_cart_marker.controls.append(makeYRotControl())
    right_arm_cart_marker.controls.append(makeZRotControl())
    server.insert(right_arm_cart_marker, processFeedback)

def removeLeftArmControl() :
    global left_arm_cart_marker
    left_arm_cart_marker.controls = []
    server.erase(left_arm_cart_marker.name)

def removeRightArmControl() :
    global right_arm_cart_marker
    right_arm_cart_marker.controls = []
    server.erase(right_arm_cart_marker.name)

def makeLeftSetpointControl( ) :
    global left_setpoint_marker
    left_setpoint_marker.controls.append(makeXTransControl())
    left_setpoint_marker.controls.append(makeYTransControl())
    left_setpoint_marker.controls.append(makeZTransControl())
    left_setpoint_marker.controls.append(makeXRotControl())
    left_setpoint_marker.controls.append(makeYRotControl())
    left_setpoint_marker.controls.append(makeZRotControl())
    r2.makeLeftHandSetpointMarker(left_setpoint_marker)
    server.insert(left_setpoint_marker, processFeedback)
    left_setpoint_menu_handler.apply( server, left_setpoint_marker.name )

def makeRightSetpointControl( ) :
    global right_setpoint_marker
    right_setpoint_marker.controls.append(makeXTransControl())
    right_setpoint_marker.controls.append(makeYTransControl())
    right_setpoint_marker.controls.append(makeZTransControl())
    right_setpoint_marker.controls.append(makeXRotControl())
    right_setpoint_marker.controls.append(makeYRotControl())
    right_setpoint_marker.controls.append(makeZRotControl())
    r2.makeRightHandSetpointMarker(right_setpoint_marker)
    server.insert(right_setpoint_marker, processFeedback)
    right_setpoint_menu_handler.apply( server, right_setpoint_marker.name )

def removeLeftSetpointControl() :
    global left_setpoint_marker
    left_setpoint_marker.controls = []
    server.erase(left_setpoint_marker.name)

def removeRightSetpointControl() :
    global right_setpoint_marker
    right_setpoint_marker.controls = []
    server.erase(right_setpoint_marker.name)

def makeArmMenu( frame_id, mesh_name, p, menu_handler ):

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.scale = 0.2
    int_marker.name = frame_id

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    marker = makeMesh( int_marker, mesh_name, p, 1.02 )
    control.markers.append( marker )
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

def makeGazeControl( ):

    global gaze_sphere_scale
    global gaze_marker

    makeSphereControl(gaze_marker, gaze_sphere_scale)

    gaze_marker.controls.append(makeXTransControl())
    gaze_marker.controls.append(makeYTransControl())
    gaze_marker.controls.append(makeZTransControl())
    gaze_marker.controls.append(makeXRotControl())
    gaze_marker.controls.append(makeYRotControl())
    gaze_marker.controls.append(makeZRotControl())

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    marker = makeSphere( gaze_marker, gaze_sphere_scale*1.01 )
    control.markers.append( marker )
    gaze_marker.controls.append(control)

    server.insert(gaze_marker, processFeedback)
    gaze_menu_handler.apply( server, gaze_marker.name )


def removeGazeControl() :
    global gaze_marker
    gaze_marker.controls = []
    server.erase(gaze_marker.name)


def makeWaistMarker( ):

    int_marker = InteractiveMarker()
    frame_id = frames.waist_frame_id
    int_marker.header.frame_id = frame_id
    int_marker.scale = 0.4
    int_marker.name = frame_id

    int_marker.controls.append(makeYRotControl())

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    marker = makeMesh( int_marker,  r2.body_mesh, r2.waist_mesh_pose, 1.02 )
    control.markers.append( marker )
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    waist_menu_handler.apply( server, int_marker.name )
   

def makeHeadControl( ) :

    global neck_lower_pitch_marker, neck_roll_marker, neck_upper_pitch_marker

    lower_pitch_control = makeZRotControl()
    roll_control = makeZRotControl()
    upper_pitch_control = makeZRotControl()

    neck_lower_pitch_marker.controls.append(lower_pitch_control)
    neck_roll_marker.controls.append(roll_control)
    neck_upper_pitch_marker.controls.append(upper_pitch_control)
   
    server.insert(neck_lower_pitch_marker, processFeedback)
    server.insert(neck_roll_marker, processFeedback)
    server.insert(neck_upper_pitch_marker, processFeedback)
 
 
def makeHeadMenu( ) :
    global neck_roll_marker
    head_menu_control = InteractiveMarkerControl()
    head_menu_control.interaction_mode = InteractiveMarkerControl.MENU
    marker = makeMesh( neck_roll_marker, r2.head_mesh, r2.head_mesh_pose, 1.02 )
    head_menu_control.markers.append( marker )
    neck_roll_marker.controls.append(head_menu_control)
    server.insert(neck_roll_marker, processFeedback)
    head_menu_handler.apply( server, neck_roll_marker.name )

def makeBackpackMenu( ) :
    global backpack_marker
    backpack_menu_control = InteractiveMarkerControl()
    backpack_menu_control.interaction_mode = InteractiveMarkerControl.MENU
    marker = makeMesh( backpack_marker, r2.backpack_mesh, r2.backpack_mesh_pose, 1.02 )
    backpack_menu_control.markers.append( marker )
    backpack_marker.controls.append(backpack_menu_control)
    server.insert(backpack_marker, processFeedback)
    backpack_menu_handler.apply( server, backpack_marker.name )

def removeHeadControl() :
    global neck_lower_pitch_marker, neck_roll_marker, neck_upper_pitch_marker
    neck_roll_marker.controls = []
    server.erase(neck_lower_pitch_marker.name)
    server.erase(neck_roll_marker.name)
    server.erase(neck_upper_pitch_marker.name)
    makeHeadMenu()

def makePostureMarkers():
    left_posture_marker.name = frames.posture_frame_id[0]
    right_posture_marker.name = frames.posture_frame_id[1]
    left_posture_control = makeYRotControl()
    right_posture_control = makeYRotControl()
    left_posture_marker.controls.append(left_posture_control)
    right_posture_marker.controls.append(right_posture_control)
    server.insert(right_posture_marker, processFeedback)
    server.insert(left_posture_marker, processFeedback)

def makeFingerMarkers():
    for i in range(len(left_finger_markers)) :
        left_finger_markers[i].name = frames.finger_frame_id[0][i]
        right_finger_markers[i].name = frames.finger_frame_id[1][i]
        left_finger_markers[i].controls.append(makeYRotControl())
        right_finger_markers[i].controls.append(makeYRotControl())
        server.insert(left_finger_markers[i], processFeedback)
        server.insert(right_finger_markers[i], processFeedback)

def joint_state_cb(data) :
    global joint_data
    joint_data = data
   
def SetUpMenus() :

    left_arm_menu_handler.insert( "Go To ReadyPose", callback=processFeedback )
    left_arm_menu_handler.setCheckState( left_arm_menu_handler.insert( "Joint Mode", callback=processFeedback ), MenuHandler.UNCHECKED ) 
    left_arm_menu_handler.setCheckState( left_arm_menu_handler.insert( "Edit Control Frame", callback=processFeedback ), MenuHandler.UNCHECKED )  
    left_arm_menu_handler.setCheckState( left_arm_menu_handler.insert( "Set Reach Point", callback=processFeedback ), MenuHandler.UNCHECKED )  
    left_arm_sub_menu_handle = left_arm_menu_handler.insert( "Grasp Controls" )
    left_arm_menu_handler.insert( "Open Hand", parent=left_arm_sub_menu_handle, callback=processFeedback )
    left_arm_menu_handler.insert( "Close Hand", parent=left_arm_sub_menu_handle, callback=processFeedback )
    left_arm_menu_handler.insert( "Reset Control Frame", callback=processFeedback )

    right_arm_menu_handler.insert( "Go To ReadyPose", callback=processFeedback )
    right_arm_menu_handler.setCheckState( right_arm_menu_handler.insert( "Joint Mode", callback=processFeedback ), MenuHandler.UNCHECKED )   
    right_arm_menu_handler.setCheckState( right_arm_menu_handler.insert( "Edit Control Frame", callback=processFeedback ), MenuHandler.UNCHECKED )  
    right_arm_menu_handler.setCheckState( right_arm_menu_handler.insert( "Set Reach Point", callback=processFeedback ), MenuHandler.UNCHECKED )  
    right_arm_sub_menu_handle = right_arm_menu_handler.insert( "Grasp Controls" )
    right_arm_menu_handler.insert( "Open Hand", parent=right_arm_sub_menu_handle, callback=processFeedback )
    right_arm_menu_handler.insert( "Close Hand", parent=right_arm_sub_menu_handle, callback=processFeedback )
    right_arm_menu_handler.insert( "Reset Control Frame", callback=processFeedback )

    waist_menu_handler.insert( "Go To ReadyPose", callback=processFeedback )

    head_menu_handler.insert( "Go To ReadyPose", callback=processFeedback )
    head_menu_handler.setCheckState( head_menu_handler.insert( "Joint Control", callback=processFeedback ), MenuHandler.UNCHECKED )
    head_menu_handler.insert( "Toggle Gaze Control", callback=processFeedback )
    head_menu_handler.insert( "Segment Table Top", callback=processFeedback )

    gaze_menu_handler.insert( "Look At This Point", callback=processFeedback )
    gaze_tracking_sub_menu_handle = gaze_menu_handler.insert( "Tracking Mode", callback=processFeedback )
    gaze_menu_handler.setCheckState( gaze_menu_handler.insert( "Free Floating", parent=gaze_tracking_sub_menu_handle, callback=processFeedback ), MenuHandler.UNCHECKED )
    gaze_menu_handler.setCheckState( gaze_menu_handler.insert( "Left Tool", parent=gaze_tracking_sub_menu_handle, callback=processFeedback ), MenuHandler.UNCHECKED )
    gaze_menu_handler.setCheckState( gaze_menu_handler.insert( "Right Tool", parent=gaze_tracking_sub_menu_handle, callback=processFeedback ), MenuHandler.UNCHECKED )

    backpack_menu_handler.setCheckState( backpack_menu_handler.insert( "Power", callback=processFeedback ), MenuHandler.UNCHECKED )

    left_setpoint_menu_handler.insert( "Move to Point", callback=processFeedback )
    right_setpoint_menu_handler.insert( "Move to Point", callback=processFeedback )

  
def SetUpMarkers() :

    global left_finger_markers, right_finger_markers

    left_arm_cart_marker.header.frame_id = frames.base_frame_id
    left_arm_cart_marker.name = frames.control_marker_id[0]
    left_arm_cart_marker.scale = 0.2

    right_arm_cart_marker.header.frame_id = frames.base_frame_id
    right_arm_cart_marker.name = frames.control_marker_id[1]
    right_arm_cart_marker.scale = 0.2

    left_setpoint_marker.header.frame_id = frames.base_frame_id
    left_setpoint_marker.name = frames.setpoint_marker_id[0]
    left_setpoint_marker.scale = 0.2

    right_setpoint_marker.header.frame_id = frames.base_frame_id
    right_setpoint_marker.name = frames.setpoint_marker_id[1]
    right_setpoint_marker.scale = 0.2

    left_posture_marker.header.frame_id = frames.posture_frame_id[0]
    right_posture_marker.header.frame_id = frames.posture_frame_id[1]
    left_posture_marker.scale = 0.25
    right_posture_marker.scale = 0.25

    neck_lower_pitch_marker.header.frame_id = frames.neck_frame_id[0]
    neck_lower_pitch_marker.name = frames.neck_frame_id[0]
    neck_lower_pitch_marker.scale = 0.4

    neck_roll_marker.header.frame_id = frames.neck_frame_id[1]
    neck_roll_marker.name = frames.neck_frame_id[1]
    neck_roll_marker.scale = 0.4

    neck_upper_pitch_marker.header.frame_id = frames.neck_frame_id[2]
    neck_upper_pitch_marker.name = frames.neck_frame_id[2]
    neck_upper_pitch_marker.scale = 0.4

    gaze_marker.header.frame_id = frames.base_frame_id
    gaze_marker.pose.position.x = -1.5
    gaze_marker.pose.position.z = 0
    gaze_marker.scale = 0.2
    gaze_marker.name = 'gaze_control_link'

    backpack_marker.header.frame_id = frames.backpack_frame_id
    backpack_marker.name = frames.backpack_frame_id
    backpack_marker.scale = 0.4

    for i in range(len(left_finger_markers)) :
        left_finger_markers[i].header.frame_id = frames.finger_frame_id[0][i]
        right_finger_markers[i].header.frame_id = frames.finger_frame_id[1][i]
        left_finger_markers[i].scale = .03
        right_finger_markers[i].scale = .03

def slowUpdate( ) :

    now = rospy.get_rostime()
    if gaze_left_tracking_mode :
        tf_listener.waitForTransform(frames.control_frame_id[0],frames.base_frame_id, now, rospy.Duration(5.0))

    elif gaze_right_tracking_mode :
        tf_listener.waitForTransform(frames.control_frame_id[1],frames.base_frame_id, now, rospy.Duration(5.0))
   
    tf_listener.waitForTransform(frames.control_frame_id[0],frames.base_frame_id, rospy.Time(0), rospy.Duration(5.0))
    #tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0],   rospy.Duration(dur_time))
    
    tf_listener.waitForTransform(frames.control_frame_id[1],frames.base_frame_id, rospy.Time(0), rospy.Duration(5.0))
    #tf_listener.lookupTransform(frames.control_frame_id[1],frames.base_frame_id, rospy.Duration(dur_time))
    
def fastUpdate( ) :

    global setpoint_store

    pose = PoseStamped()
    pose.header.seq = 0
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = frames.base_frame_id

    now = rospy.Time(0)
    if gaze_left_tracking_mode :
        tf_listener.waitForTransform(frames.control_frame_id[0],frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0], now)
        pose.pose = pm.toPose(trans, rot) 
        r2.gaze_pose_pub.publish(pose)

    elif gaze_right_tracking_mode :
        tf_listener.waitForTransform(frames.control_frame_id[1],frames.base_frame_id, now, rospy.Duration(dur_time))
        (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[1], now)
        pose.pose = pm.toPose(trans, rot) 
        r2.gaze_pose_pub.publish(pose)

    tf_listener.waitForTransform(frames.control_frame_id[0],frames.base_frame_id, now, rospy.Duration(dur_time))
    (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0], now)
    pose.pose = pm.toPose(trans, rot) 
    
    # append tool offset to cart marker
    if not edit_setpoint[0] :
        Fp = pm.fromMsg(pose.pose)
        Ft = pm.fromMsg(tool_offset[0])
        pose.pose = pm.toMsg(Fp*Ft) 
        pose.header.frame_id = frames.base_frame_id  
        server.setPose( left_arm_cart_marker.name, pose.pose )

    # append setpoint offset to marker
    else :
        Fp = pm.fromMsg(pose.pose)
        Fs = pm.fromMsg(setpoint_offset[0])
        Ft = pm.fromMsg(tool_offset[0])
        pose.pose = pm.toMsg(Fp*Ft*Fs) 
        pose.header.frame_id = frames.base_frame_id  
        server.setPose( left_setpoint_marker.name, pose.pose )
        
    #server.applyChanges()  

    tf_listener.waitForTransform(frames.control_frame_id[1],frames.base_frame_id, now, rospy.Duration(dur_time))
    (trans, rot) = tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[1], now)
    pose.pose = pm.toPose(trans, rot) 

    # append tool offset to cart marker
    if not edit_setpoint[1] :
        Fp = pm.fromMsg(pose.pose)
        Ft = pm.fromMsg(tool_offset[1])
        pose.pose = pm.toMsg(Fp*Ft) 
        pose.header.frame_id = frames.base_frame_id  
        server.setPose( right_arm_cart_marker.name, pose.pose )

    # append setpoint offset to marker
    else :
        Fp = pm.fromMsg(pose.pose)
        Fs = pm.fromMsg(setpoint_offset[1])
        Ft = pm.fromMsg(tool_offset[1])
        pose.pose = pm.toMsg(Fp*Ft*Fs) 
        pose.header.frame_id = frames.base_frame_id  
        server.setPose( right_setpoint_marker.name, pose.pose )

    server.applyChanges()   


if __name__=="__main__":

    rospy.init_node("r2_interactive_control")    
    server = InteractiveMarkerServer("r2_interactive_control")
    tf_listener = tf.TransformListener()
   
    # set cart modes
    r2.SetArmsToCartMode(frames.control_frame_id[0], frames.control_frame_id[1]) 
    
    # Subscribe to joint data
    rospy.Subscriber("/joint_states", JointState, joint_state_cb)

    # set up menus
    SetUpMenus()

    # set meshes and corresponding pose offsets    
    r2.SetUpMeshData()

    # set up markers
    SetUpMarkers()

    # init tool offsets
    ResetToolOffset("left")
    ResetToolOffset("right")

    # set up control markers
    makeLeftArmControl( )
    makeRightArmControl( )

    makeWaistMarker( )
    makePostureMarkers()
    makeFingerMarkers()

    # set up menus
    makeArmMenu( frames.menu_frame_id[0], r2.left_palm_mesh, r2.left_palm_mesh_pose, left_arm_menu_handler)
    makeArmMenu( frames.menu_frame_id[1], r2.right_palm_mesh, r2.right_palm_mesh_pose, right_arm_menu_handler )
    makeHeadMenu( )
    makeBackpackMenu( )
    
    server.applyChanges()

    # peform state update s
    r = rospy.Rate(fast_update_rate)
    c = 0
    while not rospy.is_shutdown():
        if(c % slow_update_divider == 0) : slowUpdate()
        c = c+1
        fastUpdate()
        r.sleep()
    rospy.spin()

