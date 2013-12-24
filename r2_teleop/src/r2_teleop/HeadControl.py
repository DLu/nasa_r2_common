from r2_teleop import *

head_mesh = "package://r2_description/meshes/Head.dae"
neckJointNames = ['/r2/neck/joint0', '/r2/neck/joint1', '/r2/neck/joint2']
neck_frame_id = ['r2/neck_lower_pitch', 'r2/neck_roll', 'r2/neck_upper_pitch']

class HeadControl:
    def __init__(self, server):
        self.server = server
        self.mesh_pose = Pose()
        hq = get_quaternion(0, 1.57, 0)
        self.mesh_pose.position.x = 0.0
        self.mesh_pose.position.y = 0.07
        self.mesh_pose.position.z = 0.0
        self.mesh_pose.orientation.x = hq[0]
        self.mesh_pose.orientation.y = hq[1]
        self.mesh_pose.orientation.z = hq[2]
        self.mesh_pose.orientation.w = hq[3]

        self.JointReadyPose = make_joint_state(neckJointNames, [0.0, 0.0, -5.0])

        self.markers = []
        for frame in neck_frame_id:
            marker = InteractiveMarker()
            marker.header.frame_id = frame
            marker.name = frame
            marker.scale = 0.4
            self.markers.append(marker)


        self.jnt_pub  = rospy.Publisher('/r2/r2_controller/neck/joint_command',      JointState)

        self.menu = MenuHandler()
        add_to_menu(self.menu, "Go To ReadyPose", self.handleHeadMenu)
        add_to_menu(self.menu, "Joint Control", self.handleHeadMenu, True)
        add_to_menu(self.menu, "Toggle Gaze Control", self.handleHeadMenu)
        # TODO add_to_menu(self.menu, "Segment Table Top", self.handleHeadMenu)

        self.makeHeadMenu()

    def makeHeadControl(self) :
        for marker in self.markers:
            control = makeZRotControl()
            marker.controls.append(control)
            self.server.insert(marker, self.handle_feedback)

    def removeHeadControl(self) :
#        self.markers[1].controls = [] TODO?
        for marker in self.markers:
            self.server.erase(marker)
        self.makeHeadMenu()

 
    def makeHeadMenu(self) :
        neck_roll_marker = self.markers[1]
        
        head_menu_control = InteractiveMarkerControl()
        head_menu_control.interaction_mode = InteractiveMarkerControl.MENU
        marker = makeMesh( neck_roll_marker, head_mesh, self.mesh_pose, 1.02 )
        head_menu_control.markers.append( marker )
        neck_roll_marker.controls.append(head_menu_control)
        self.server.insert(neck_roll_marker, self.handleHeadMenu) 
        self.menu.apply( self.server, neck_roll_marker.name ) 
        self.server.applyChanges()




    def handleHeadMenu( self, feedback ) :
        if feedback.event_type != InteractiveMarkerFeedback.MENU_SELECT:
            return
        handle = feedback.menu_entry_id
        if(handle == 1) :
            self.JointReadyPose.header.stamp = rospy.Time.now()
            self.jnt_pub.publish(self.JointReadyPose)
            self.server.resetMarker(feedback.marker_name)
        elif(handle == 2) :
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("turning off head joint control")
                self.removeHeadControl()
            else:
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                rospy.loginfo("turning on head joint control")
                self.makeHeadControl()
            self.menu.reApply( self.server ) 
        elif(feedback.menu_entry_id == 3) :
            self.server.gaze.toggleGazeControl()
        elif(feedback.menu_entry_id == 4) :
            r2.SegmentTableTop() #TODO Segment
        self.server.applyChanges()


    def handle_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return

        cmd = None
        if feedback.marker_name == neck_frame_id[0]:
            cmd = self.server.get_joint_command('/r2/neck/joint0', -feedback.pose.orientation.y)
        elif feedback.marker_name == neck_frame_id[1]:
            cmd = self.server.get_joint_command('/r2/neck/joint1', feedback.pose.orientation.y)
        elif feedback.marker_name == neck_frame_id[2]:
            cmd = self.server.get_joint_command('/r2/neck/joint2', feedback.pose.orientation.y)

        if cmd:
            self.jnt_pub.publish(cmd)
            self.server.resetMarker(feedback.marker_name)
        self.server.applyChanges()


