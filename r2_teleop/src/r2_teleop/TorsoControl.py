from r2_msgs.srv import *
from r2_teleop import *

waist_frame_id    = 'r2/waist_center'
backpack_frame_id = 'r2/backpack'
body_mesh = "package://r2_description/meshes/Body_Cover.dae"
backpack_mesh = "package://r2_description/meshes/Backpack.dae"
waistJointNames = ['/r2/waist/joint0']

class TorsoControl:
    def __init__(self, server, fake=False):
        self.power_mode = False
        self.server = server
        self.waist_menu = MenuHandler()
        add_to_menu(self.waist_menu, "Go To ReadyPose", self.handleWaistMenu)

        self.backpack_menu = MenuHandler()
        add_to_menu(self.backpack_menu, "Power", self.handleBackpackMenu, True)

        self.jnt_pub = rospy.Publisher('/r2/r2_controller/waist/joint_command',     JointState)

        self.backpack_marker = InteractiveMarker()
        self.backpack_marker.header.frame_id = backpack_frame_id
        self.backpack_marker.name = backpack_frame_id
        self.backpack_marker.scale = 0.4

        self.waistJointReadyPose = make_joint_state(waistJointNames, [180.0])

        self.waist_mesh_pose = Pose()
        wq = get_quaternion(-1.57, 0, -1.57)
        self.waist_mesh_pose.position.x = 0.02
        self.waist_mesh_pose.position.y = 0.05
        self.waist_mesh_pose.position.z = -0.5625
        self.waist_mesh_pose.orientation.x = wq[0]
        self.waist_mesh_pose.orientation.y = wq[1]
        self.waist_mesh_pose.orientation.z = wq[2]
        self.waist_mesh_pose.orientation.w = wq[3]

        self.backpack_mesh_pose = Pose()
        bq = get_quaternion(0, 0, 0.25)
        self.backpack_mesh_pose.position.x = 0.0
        self.backpack_mesh_pose.position.y = 0.0
        self.backpack_mesh_pose.position.z = -0.54
        self.backpack_mesh_pose.orientation.x = bq[0]
        self.backpack_mesh_pose.orientation.y = bq[1]
        self.backpack_mesh_pose.orientation.z = bq[2]
        self.backpack_mesh_pose.orientation.w = bq[3]
        
        self.makeWaistMarker()
        self.makeBackpackMenu()

        if not fake:
            rospy.wait_for_service('/r2/r2_controller/power')
            self.power_srv = rospy.ServiceProxy('/r2/r2_controller/power', Power)

    def SetPower(self, p) :
        print "setting motor power to: ", p
        try:
            resp1 = self.power_srv('motor', p)
            print "Set Power: ", resp1.status
        except rospy.ServiceException, e: 
            print "Service call failed for setting power: %s"%e

        return resp1.status

    def handleWaistMenu( self, feedback ) :
        if feedback.event_type != InteractiveMarkerFeedback.MENU_SELECT:
            return
        if(feedback.menu_entry_id == 1) :
            self.waistJointReadyPose.header.stamp = rospy.Time.now()
            self.jnt_pub.publish(self.waistJointReadyPose)
        self.server.applyChanges()


    def handleBackpackMenu( self, feedback ) :
        if feedback.event_type != InteractiveMarkerFeedback.MENU_SELECT:
            return
        handle = feedback.menu_entry_id
        if(feedback.menu_entry_id == 1) :
            state = self.backpack_menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.backpack_menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("turning off power")
                self.power_mode = False
            else:
                self.backpack_menu.setCheckState( handle, MenuHandler.CHECKED )
                rospy.loginfo("turning on power")
                self.power_mode = True
            self.SetPower(self.power_mode)
            self.backpack_menu.reApply( self.server )
        self.server.applyChanges()


    def makeWaistMarker(self):
        int_marker = InteractiveMarker()
        frame_id = waist_frame_id
        int_marker.header.frame_id = frame_id
        int_marker.scale = 0.4
        int_marker.name = frame_id
        int_marker.controls.append(makeYRotControl())

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        marker = makeMesh( int_marker,  body_mesh, self.waist_mesh_pose, 1.02 )
        control.markers.append( marker )
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.handle_feedback) 
        self.waist_menu.apply( self.server, int_marker.name )
        self.server.applyChanges()


    def makeBackpackMenu(self ) :

        backpack_menu_control = InteractiveMarkerControl()
        backpack_menu_control.interaction_mode = InteractiveMarkerControl.MENU
        marker = makeMesh( self.backpack_marker, backpack_mesh, self.backpack_mesh_pose, 1.02 )
        backpack_menu_control.markers.append( marker )
        self.backpack_marker.controls.append(backpack_menu_control)
        self.server.insert(self.backpack_marker, self.handleBackpackMenu)
        self.backpack_menu.apply( self.server, self.backpack_marker.name )
        self.server.applyChanges()



    def handle_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        quat = feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w
        rpy = euler_from_quaternion(quat)

        self.jnt_pub.publish( self.server.get_joint_command('/r2/waist/joint0', rpy[2]) )
        self.server.resetMarker(feedback.marker_name)
        self.server.applyChanges()




