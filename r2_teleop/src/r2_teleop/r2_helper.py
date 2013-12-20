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
import time

import marker_helper

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from r2_msgs.srv import *
#from r2_gazebo.srv import *
#from r2_sensing.srv import *

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

def Servo(s, mode) :

    rospy.wait_for_service('/r2/r2_controller/servo')
    Servo = rospy.ServiceProxy('/r2/r2_controller/servo', Servo)
    
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


# joint states
leftArmJointNames = ['/r2/left_arm/joint'+str(i) for i in range(7)] 
leftJointNames = leftArmJointNames + leftHandNames

rightArmJointNames = ['/r2/right_arm/joint'+str(i) for i in range(7)] 

rightJointNames = rightArmJointNames + rightHandNames


