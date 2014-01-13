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

import tf
from interactive_markers.interactive_marker_server import *
from sensor_msgs.msg import JointState
from r2_teleop import *
from r2_teleop.ArmControl import ArmControl
from r2_teleop.TorsoControl import TorsoControl
from r2_teleop.HeadControl import HeadControl
from r2_teleop.GazeControl import GazeControl

LR = ['left', 'right']

ARMS = True
HEAD = True
TORSO = True
FAKE = False

class R2InteractiveNode(InteractiveMarkerServer):
    def __init__(self):
        InteractiveMarkerServer.__init__(self, "r2_interactive_control")
        self.tf_listener = tf.TransformListener()
        self.arms = []
        for side in LR:
            marker = ArmControl(self, side, FAKE)
            self.arms.append( marker )
        self.gaze = GazeControl(self) if HEAD else None
        self.head = HeadControl(self) if HEAD else None
        self.torso = TorsoControl(self, FAKE) if TORSO else None
        self.js_sub = rospy.Subscriber("/r2/joint_states", JointState, self.joint_state_cb)
            
    def erase(self, marker):
        InteractiveMarkerServer.erase(self, marker.name)
        marker.controls = []
        
    def resetMarker( self, frame_id ):    
        pose = Pose()
        pose.orientation.w = 1
        self.setPose( frame_id, pose )
        self.applyChanges()

    def joint_state_cb(self, data) :
        self.joint_data = data

    def get_joint_command(self, name, d):
        jnt_cmd = JointState()
        i = self.joint_data.name.index(name)
        j_act = self.joint_data.position[i] + d
        jnt_cmd.header.stamp = rospy.Time.now()
        jnt_cmd.header.frame_id = "r2/robot_base"
        jnt_cmd.name = [name]
        jnt_cmd.position = [j_act]
        return jnt_cmd

    def run(self):          
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
        if self.gaze:
            self.gaze.slow_update()
        for arm in self.arms:
            if arm:
                arm.slow_update()
        
    def fastUpdate(self) :
        now = rospy.Time(0)
        if self.gaze:
            self.gaze.fast_update()

        for arm in self.arms:
            if arm:
                arm.fast_update()
        self.applyChanges()   

if __name__=="__main__":
    rospy.init_node("r2_interactive_control")    
    node = R2InteractiveNode()
    node.run()

