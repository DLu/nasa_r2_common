#!/usr/bin/python
import roslib; roslib.load_manifest("r2_teleop")
import rospy

import tf
from interactive_markers.interactive_marker_server import *
from sensor_msgs.msg import JointState
from r2_msgs.srv import *
from r2_teleop import *
from r2_teleop.HeadControl import neckJointNames
from r2_teleop.FingerControl import get_hand_names


fake = False

class KeyboardTeleop:

    def __init__(self):
        self.js_sub = rospy.Subscriber("/r2/joint_states", JointState, self.joint_state_cb)
        self.jnt_pub = rospy.Publisher('/r2/r2_controller/left_arm/joint_command',  JointState)
        self.tf_listener = tf.TransformListener()

        if not fake:
            rospy.wait_for_service('/r2/r2_controller/set_joint_mode')
            set_joint_mode = rospy.ServiceProxy('/r2/r2_controller/set_joint_mode', SetJointMode)
            for side in ['left', 'right']:
                try:
                    resp1 = set_joint_mode(side)
                except rospy.ServiceException, e: 
                    print "Service call failed for ", self.side, " arm: %s"%e

        self.left_hand = get_hand_names('left')
        self.right_hand = get_hand_names('right')
        self.left_names = get_joint_names('/r2/left_arm/joint', 7) + self.left_hand
        self.right_names = get_joint_names('/r2/right_arm/joint', 7) + self.right_hand

        self.left_ready = make_joint_state(self.left_names, [50.0, -80.0, -105.0, -140.0, 80.0, 0.0, 0.0]+[0.0]*12)
        self.right_ready = make_joint_state(self.right_names, [-50.0, -80.0, 105.0, -140.0, -80.0, 0.0, 0.0]+[0.0]*12)
        self.horns_pose = make_joint_state(self.right_names, [0.23, -0.92, 2.53, -2.69, -2.19, .11, 0, 1.22, 1.40, .28, -.39, -.28, 0, 0, 0, 1.57, 1.57, 1.57, 0], convert_to_rad=False)

        self.lclose = make_joint_state(self.left_hand, [-100, 0, 0, 0, 0, 100, 70, 0, 100, 70, 170, 170])
        self.rclose = make_joint_state(self.right_hand, [100, 0, 0, 0, 0, 100, 70, 0, 100, 70, 150, 150])
        self.lopen = make_joint_state(self.left_hand, [0]*12)
        self.ropen = make_joint_state(self.right_hand, [0]*12)

        self.head_ready = make_joint_state(neckJointNames, [0.0, 0.0, -5.0])

        self.MENU = [('Head Menu', [('Ready Pose', self.head_ready)]),
                     ('Arm Menu', [
                        ('Left Ready Pose', self.left_ready),
                        ('Right Ready Pose', self.right_ready),
                        ('Right Special', self.horns_pose),
                        ('Left Close', self.lclose),
                        ('Left Open', self.lopen),
                        ('Right Close', self.rclose),
                        ('Right Open', self.ropen)
                                  ])]

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

    def spin(self):
        index = -1
        while not rospy.is_shutdown():
            try:
                if index < 0:
                    menu = self.MENU + [('Quit', None)]
                else:
                    menu = self.MENU[index][1] + [('Back', None)]
                for i, (name, item) in enumerate(menu):
                    print "%d) %s"%(i+1, name)
                sel = input('?')-1
            except:
                continue
            name, item = menu[sel]
            if name=='Quit':
                break
            elif name=='Back':
                index = -1
            elif index < 0:
                index = sel
            else:
                item.header.stamp = rospy.Time.now()
                self.jnt_pub.publish(item) 


if __name__=='__main__':
    rospy.init_node("r2_keyboard_control")  
    kt = KeyboardTeleop()
    kt.spin()
