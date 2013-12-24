from r2_teleop import *

FINGER_NAMES = ['thumb_base', 'thumb_medial_prime', 'thumb_medial', 'thumb_distal',
                'index_base', 'index_proximal', 'index_medial', 
                'middle_base', 'middle_proximal', 'middle_medial', 
                'ring_proximal', 
                'little_proximal']

finger_frame_id = 'r2/%s_%s'


JOINT_PATTERN = 'r2/%%s_arm/hand/%s/joint' 
FINGER_DATA = [(JOINT_PATTERN%'thumb', 4),
               (JOINT_PATTERN%'index', 3),
               (JOINT_PATTERN%'middle', 3),
               (JOINT_PATTERN%'ring', 1),
               (JOINT_PATTERN%'little', 1)]

def get_hand_names(side):
    a = []
    for pattern, n in FINGER_DATA:
        a += get_joint_names(pattern%side, n)
    return a

# meshes
"""
left_thumb_carp_mesh = "package://r2_description/meshes/Left_Thumb_Carp.dae"
left_thumb_carp_mtcar = "package://r2_description/meshes/Left_Thumb_MtCar.dae"

right_thumb_carp_mesh = "package://r2_description/meshes/Left_Thumb_Carp.dae"
right_thumb_carp_mtcar = "package://r2_description/meshes/Right_Thumb_MtCar.dae"

thumb_prox_mesh = "package://r2_description/meshes/Thumb_Proximal.dae"
thumb_dist_mesh = "package://r2_description/meshes/Thumb_Dist.dae"
finger_prox_mesh = "package://r2_description/meshes/Finger_Proximal.dae"
finger_mid_mesh = "package://r2_description/meshes/Finger_Mid.dae"
finger_dist_mesh = "package://r2_description/meshes/Finger_Dist.dae"
"""

class FingerControl:
    def __init__(self, server, side):
        self.side = side
        self.finger_markers = []
        for name in FINGER_NAMES:
            m = InteractiveMarker()
            m.name = finger_frame_id%(self.side, name)
            m.controls.append(makeYRotControl())
            m.header.frame_id = m.name
            m.scale = .03
            self.finger_markers.append(m)
            
            #server.insert(m, self.processFeedback)
        self.joint_names = get_hand_names(self.side)
        
        if self.side == 'left':
             self.close = make_joint_state(self.joint_names, [-100, 0, 0, 0, 0, 100, 70, 0, 100, 70, 170, 170])
             self.close_thumb = make_joint_state(self.joint_names, [-200, 100, 100, 50, 0, 100, 70, 0, 100, 70, 170, 170])
        else:
             self.close = make_joint_state(self.joint_names, [100, 0, 0, 0, 0, 100, 70, 0, 100, 70, 150, 150])
             self.close_thumb = make_joint_state(self.joint_names, [200, 100, 100, 50, 0, 100, 70, 0, 100, 70, 150, 150])

        #TODO OPEN POSE, CLOSED POSE

    def processFeedback(self):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        for name in [marker.name for marker in self.finger_markers]:
            if feedbac.marker_name == name:
                self.jnt_pub.publish( self.server.get_joint_command(name, feedback.pose.orientation.z))
                resetMarker(feedback, feedback.marker_name)  
                return
        self.server.applyChanges()




