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

import roslib; roslib.load_manifest("r2_teleop")
import rospy
import math
import copy
import PyKDL as kdl # ros-hydro-python-orocos-kdl

import tf
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TransformStamped

import r2_teleop.kdl_posemath as pm
import r2_teleop.r2_helper as r2
import r2_teleop.frames as frames
from r2_teleop.marker_helper import *

    
class R2InteractiveNode:
    def __init__(self):
        rospy.init_node("r2_interactive_control")    
        self.server = InteractiveMarkerServer("r2_interactive_control")
        self.tf_listener = tf.TransformListener()
        """self.arms = []
        for marker_frame, control_frame, setpoint_frame in zip(frames.control_marker_id, frames.control_frame_id, frames.setpoint_marker_id):
            marker = ArmControl(marker_frame, control_frame, setpoint_frame, self.processFeedback, self)
            self.arms.append( marker )
        self.gaze = GazeControl(self)
        self.neck = NeckControl(self)
        self.backpack = BackpackControl(self)
        self.waist = WaistControl(self)
        self.js_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)"""
            

    def erase(self, marker):
        marker.controls = []
        self.server.erase(marker.name)
        
    def resetMarker( self, feedback, frame_id ):    
        pose = feedback.pose
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        self.server.setPose( frame_id, pose )
        self.server.applyChanges()

    def processFeedback( self, feedback ) :
             
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
                self.arms[0].handleArmMenu(feedback)
            elif feedback.marker_name == frames.menu_frame_id[1] :
                self.arms[1].handleArmMenu(feedback)
            elif feedback.marker_name == frames.waist_frame_id :
                self.waist.handleMenu(feedback)
            elif feedback.marker_name == frames.head_frame_id :
                handleHeadMenu(feedback)
            elif feedback.marker_name == frames.gaze_frame_id :
                self.gaze.handleMenu(feedback)
            elif feedback.marker_name == frames.backpack_frame_id :
                self.backpack.handleMenu(feedback)
            elif feedback.marker_name == frames.setpoint_marker_id[0]:
                self.arms[0].handleSetpointMenu(feedback)
            elif feedback.marker_name == frames.setpoint_marker_id[1]:
                self.arms[0].handleSetpointMenu(feedback)

        elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            poseUpdate(feedback)

        #elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        #    rospy.loginfo( s + ": mouse down" + mp + "." )
        
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            mouseUpdate(feedback)

        self.server.applyChanges()
        
    def joint_state_cb(data) :
        self.joint_data = data


    def run(self):          
        # set cart modes
        r2.SetArmsToCartMode(frames.control_frame_id[0], frames.control_frame_id[1])   
        # peform state updates
        fast_update_rate = 10
        slow_update_divider = 10
        
        r = rospy.Rate(fast_update_rate)
        c = 0
        while not rospy.is_shutdown():
            if(c % slow_update_divider == 0) : self.slowUpdate()
            c = c+1
            self.fastUpdate()
            r.sleep()
        rospy.spin()
        
    def slowUpdate(self) :
        now = rospy.Time.now()
        if self.gaze.left_tracking_mode:
            self.tf_listener.waitForTransform(frames.control_frame_id[0],frames.base_frame_id, now, rospy.Duration(5.0))

        elif self.gaze.right_tracking_mode:
            self.tf_listener.waitForTransform(frames.control_frame_id[1],frames.base_frame_id, now, rospy.Duration(5.0))
       
        for frame in frames.control_frame_id:
            self.tf_listener.waitForTransform( frame, frames.base_frame_id, rospy.Time(0), rospy.Duration(5.0))
        
    def fastUpdate(self) :
        pose = PoseStamped()
        pose.header.seq = 0
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = frames.base_frame_id

        now = rospy.Time(0)
        if self.gaze.left_tracking_mode:
            self.tf_listener.waitForTransform(frames.control_frame_id[0],frames.base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[0], now)
            pose.pose = pm.toPose(trans, rot) 
            r2.gaze_pose_pub.publish(pose)

        elif self.gaze.right_tracking_mode:
            self.tf_listener.waitForTransform(frames.control_frame_id[1],frames.base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.tf_listener.lookupTransform(frames.base_frame_id, frames.control_frame_id[1], now)
            pose.pose = pm.toPose(trans, rot) 
            r2.gaze_pose_pub.publish(pose)

        for arm in self.arms:
            self.tf_listener.waitForTransform(arm.control_frame, frames.base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.tf_listener.lookupTransform(frames.base_frame_id, arm.control_frame, now)
            pose.pose = pm.toPose(trans, rot) 
        
            # append tool offset to cart marker
            if not arm.edit_setpoint :
                Fp = pm.fromMsg(pose.pose)
                Ft = pm.fromMsg(arm.tool_offset)
                pose.pose = pm.toMsg(Fp*Ft) 
                pose.header.frame_id = frames.base_frame_id  
                self.server.setPose( arm.name, pose.pose )
            # append setpoint offset to marker
            else :
                Fp = pm.fromMsg(pose.pose)
                Fs = pm.fromMsg(arm.setpoint_offset)
                Ft = pm.fromMsg(arm.tool_offset)
                pose.pose = pm.toMsg(Fp*Ft*Fs) 
                pose.header.frame_id = frames.base_frame_id  
                server.setPose( arm.setpoint_marker.name, pose.pose )
        self.server.applyChanges()   

if __name__=="__main__":
    node = R2InteractiveNode()
    node.run()

