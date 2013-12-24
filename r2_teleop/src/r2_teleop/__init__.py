import PyKDL as kdl # ros-hydro-python-orocos-kdl
from math import radians
from interactive_markers.menu_handler import *
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, TransformStamped
import r2_teleop.kdl_posemath as pm
from r2_teleop.marker_helper import *
from sensor_msgs.msg import JointState

base_frame_id = "r2/robot_base"
dur_time = 2.0

def make_joint_state(names, positions, frame="world"):
    state = JointState()
    state.header.frame_id = frame
    state.name = names
    state.position = map(radians, positions)
    return state

def add_to_menu(handler, text, callback, unchecked=False):
    x = handler.insert(text, callback=callback)
    if unchecked:
        handler.setCheckState(x, MenuHandler.UNCHECKED)


def add_group_to_menu(handler, text, texts, callback, unchecked=False):
    sub_menu = handler.insert(text)
    for text0 in texts:
        x = handler.insert(text0, parent=sub_menu, callback=callback)
        if unchecked:
            handler.setCheckState(x, MenuHandler.UNCHECKED)

def get_quaternion(r, p, y):
    return kdl.Rotation.RPY(r, p, y).GetQuaternion()

def get_joint_names(base, n):
    return [base + str(i) for i in range(n)]
