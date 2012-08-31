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

import numpy as np

import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped

import r2_helper as r2
import kdl_posemath as pm


TORAD = math.pi/180.0
TODEG = 1.0/TORAD

server = None
counter = 0

marker_data = None

interactive_marker_list = []
obj_transforms = []

obj_menu_handler = MenuHandler()

robot_frame = "r2/robot_reference"
tf_listener = None
tf_broadcaster = None

need_to_orient = False

X = np.array([1, 0, 0])
Y = np.array([0, 1, 0])
Z = np.array([0, 0, 1])
	 		
	 		
def getInteractiveMarkerID(name) :
	for i in range(len(interactive_marker_list)) :
		if name == interactive_marker_list[i].name :
			return i

def TrotX(theta):
	s = math.sin(theta)
	c = math.cos(theta)
	T = kdl.Frame(kdl.Rotation(1, 0, 0, 0, c, -s, 0, s, c), kdl.Vector(0, 0, 0))
	return T

def TrotY(theta):
	s = math.sin(theta)
	c = math.cos(theta)
	T = kdl.Frame(kdl.Rotation(c, 0, s, 0, 1, 0, -s, 0, c), kdl.Vector(0, 0, 0))
	return T

def TrotZ(theta):
	s = math.sin(theta)
	c = math.cos(theta)
	T = kdl.Frame(kdl.Rotation(c, -s, 0, s, c, 0, 0, 0, 1), kdl.Vector(0, 0, 0))
	return T

def parseObjectPose(idx):
	
	print "parsing object pose"
	now = rospy.Time(0)
	pose = PoseStamped()
	
	frame_id = interactive_marker_list[idx].controls[0].markers[0].header.frame_id
	marker = interactive_marker_list[idx].controls[0].markers[0]
	scale = (marker.scale.x, marker.scale.y, marker.scale.z)

	print "object[", idx,"]: ", interactive_marker_list[idx].name
	print "pose:\n", interactive_marker_list[idx].controls[0].markers[0].pose
	print "\tin frame: ", frame_id
	print "\tscale: ", scale

	tf_listener.waitForTransform(robot_frame, frame_id, now, rospy.Duration(2.0))
    #(trans, rot) = tf_listener.lookupTransform(frame_id, robot_frame, now)

	pose.header = interactive_marker_list[idx].controls[0].markers[0].header 
	pose.pose = interactive_marker_list[idx].controls[0].markers[0].pose
		
	pose.header.stamp = now
	pose = tf_listener.transformPose(robot_frame, pose)
	print "transformed pose: "
	print pose

	p = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
	q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)

	# get object frame
	#obj = obj_transforms[idx]
	#T = pm.fromTf((obj[0],obj[1]))
	#t = pm.toTf(T)
	
	# get rotational and component parts of frame
	#R = T.M
	#p = T.p
	
	if p[1] > 0:
		arm = "right"
	else :
		arm = "left"

	print "choosing: ", arm
	wide = True
	approach= 'top'
	if (scale[2] > scale[1]*1.25) and (scale[2] > 0.12) :
		wide = False
		approach = "side"
		print "object tall"
	else :
		print "object wide"

	return pose, scale, arm, approach, p, q

def reachToObject(idx) :

	pose, scale, arm, approach, p, q = parseObjectPose(idx)

	print "p: ", p
	print "q: ", q
	print "s: ", scale
	print "pose: ", pose
	print "arm: ", arm
	print "approach", approach

	x_off = -0.085
	y_off = 0.075
 	z_off = 0.12
	
	marker = interactive_marker_list[idx].controls[0].markers[0]
	dim = np.array([marker.scale.x, marker.scale.y, marker.scale.z]) 
	poseOff = copy.deepcopy(pose)

	if scale[0] > scale[1] :
		max_side_dim = 0
	else :
		max_side_dim = 1       
	x_reach_off = x_off
	z_reach_off = scale[2]+z_off
	y_reach_off = scale[max_side_dim]+y_off # should be scaled by side projected width (not just max side width)
        
	if approach == "side" :
		if arm == "left" :
		    q = (kdl.Rotation.RPY(-1.35,0,0)).GetQuaternion()
		    y_reach_off *= -1
		else :
		    q = (kdl.Rotation.RPY( 1.35,0,0)).GetQuaternion()

	print "y_reach_off: ", y_reach_off
	print "z_reach_off: ", z_reach_off

	poseOff.header.stamp = rospy.get_rostime()

	poseOff.pose.position.x = p[0] + x_reach_off
	poseOff.pose.position.y = p[1]
	poseOff.pose.position.z = p[2]
	poseOff.pose.orientation.x = q[0]
	poseOff.pose.orientation.y = q[1]
	poseOff.pose.orientation.z = q[2]
	poseOff.pose.orientation.w = q[3]

	if approach == "top" :
		poseOff.pose.position.z -= z_reach_off
	else: 	
		poseOff.pose.position.y += y_reach_off

	print "final poseOff: "
	print poseOff

	if arm == "left" :
		r2.left_pose_pub.publish(poseOff)
	else :
		r2.right_pose_pub.publish(poseOff)

	# wait to finish reaching	
	rospy.sleep(4.0)

	# do grasp approach
	if approach == "top" :
		graspPose = copy.deepcopy(pose)
	else :
		graspPose = copy.deepcopy(pose)
		graspPose.pose.orientation.x = q[0]
		graspPose.pose.orientation.y = q[1]
		graspPose.pose.orientation.z = q[2]
		graspPose.pose.orientation.w = q[3]
	
	graspPose.header.stamp = rospy.get_rostime()
	
	if approach == "top" :
		graspPose.pose.position.z -= z_reach_off*0.1
	else: 
		graspPose.pose.position.y += y_reach_off*0.0

	if arm == "left" :
		r2.left_pose_pub.publish(graspPose)
	else :
		r2.right_pose_pub.publish(graspPose)

	# wait to finish reaching	
	rospy.sleep(2.0)

	if arm == "left" :
		r2.left_jnt_pub.publish(r2.leftHandPowerClose)
		rospy.sleep(0.3)
		r2.left_jnt_pub.publish(r2.leftHandPowerCloseThumb)
	else :
		r2.right_jnt_pub.publish(r2.rightHandPowerClose)
		rospy.sleep(0.3)
		r2.right_jnt_pub.publish(r2.rightHandPowerCloseThumb)
		
	# wait to finish reaching	
	rospy.sleep(1.5)

	liftPose = copy.deepcopy(graspPose)
	liftPose.pose.position.z -= 0.15	

	if arm == "left" :
		r2.left_pose_pub.publish(liftPose)
	else :
		r2.right_pose_pub.publish(liftPose)


def handleObjectMenu( feedback ) :
	
	handle = feedback.menu_entry_id
	idx = getInteractiveMarkerID(feedback.marker_name)
	print "Pick Me Up call on object: ", feedback.marker_name
	
	if(handle == 1) : # pick me up
		reachToObject(idx)
	elif(handle == 2) : # parse
		parseObjectPose(idx)
	
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
        handleObjectMenu(feedback)
        
    #elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
    #    poseUpdate(feedback)

    #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    #    rospy.loginfo( s + ": mouse down" + mp + "." )
    
    #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
    #    mouseUpdate(feedback)

    server.applyChanges()

def marker_array_cb(data) :
	global marker_data, interactive_marker_list, obj_transforms, need_to_orient
	pose = PoseStamped()

	print "in marker array callback"
	for m in range(len(interactive_marker_list)) :
		server.erase(interactive_marker_list[m].name)
	interactive_marker_list = []
	obj_transforms = []
	server.applyChanges()

	rospy.sleep(0.25)

	marker_data = data	
	print "got new marker array of size: ", len(data.markers)
	#print marker_data

	for m in range(len(marker_data.markers)) :
		
		interactive_marker_list.append(InteractiveMarker())
		interactive_marker_list[m].header.frame_id = "r2/asus_frame" 
		interactive_marker_list[m].scale = 1#0.2
		interactive_marker_list[m].name = "obj_"+str(m)
		interactive_marker_list[m].description = "obj_"+str(m)
		control = InteractiveMarkerControl()
		control.interaction_mode = InteractiveMarkerControl.MENU
		
		marker = marker_data.markers[m]
		marker.color.r = 1
		marker.color.g = 0
		marker.color.b = 1
		marker.color.a = 0.75
		control.markers.append( marker )
		interactive_marker_list[m].controls.append(control)
		server.insert(interactive_marker_list[m], processFeedback)
		obj_menu_handler.apply( server, interactive_marker_list[m].name )
		server.applyChanges()

		pose.header = interactive_marker_list[m].controls[0].markers[0].header 
		pose.pose = interactive_marker_list[m].controls[0].markers[0].pose
		pose = tf_listener.transformPose(robot_frame, pose)
		
		p = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
		q = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
		obj_transforms.append((p, q, interactive_marker_list[m].name))
		
		need_to_orient = True
		
def SetUpObjectMenu() :
	
	obj_menu_handler.insert( "Pick Up",  callback=processFeedback )
	obj_menu_handler.insert( "Parse Object",  callback=processFeedback )

if __name__=="__main__":

    rospy.init_node("r2_interactive_sensing")    
    server = InteractiveMarkerServer("r2_interactive_sensing")
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    
    need_to_orient = False
    
    # Subscribe to marker data
    rospy.Subscriber("/tabletop_clustering/vis_marker", MarkerArray, marker_array_cb)

    # set up menus
    SetUpObjectMenu()

    server.applyChanges()
    
    print "in r2_interactive_sensing"
#	rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        if need_to_orient :
            need_to_orient = False
	    print "got objects"
	    for i in range(len(obj_transforms)) :
                obj = obj_transforms[i]
	     		
                T = pm.fromTf((obj[0],obj[1]))
                #T = T*TrotX(90*TORAD)*TrotY(270*TORAD)
                t = pm.toTf(T)	     			     		
                tf_broadcaster.sendTransform(t[0], t[1], rospy.Time.now(), obj[2], robot_frame) 
                q = T.M.GetQuaternion()
                obj_transforms[i] = (t[0], q, interactive_marker_list[i].name)
                rospy.sleep(0.5)



