from r2_msgs.srv import *
from r2_teleop import *
from r2_teleop.FingerControl import FingerControl

control_marker_id = 'r2/%s_control_frame'
control_frame_id  = 'r2/%s_middle_base'
setpoint_marker_id = 'r2/%s_setpoint_frame'
posture_frame_id = 'r2/%s_shoulder_pitch'
menu_frame_id     = 'r2/%s_palm'

def get_arm_control_frame(side):
    return control_frame_id % side

class ArmControl:
    def __init__(self, server, side):
        self.server = server
        self.side = side
        self.joint_mode = False
        self.edit_tool = False
        self.edit_setpoint = False

        self.SetArmToCartMode()

        self.menu = MenuHandler()
        add_to_menu(self.menu, "Go To ReadyPose", self.handleArmMenu)
        add_to_menu(self.menu, "Joint Mode", self.handleArmMenu, True)
        add_to_menu(self.menu, "Edit Control Frame", self.handleArmMenu, True) 
        add_to_menu(self.menu, "Set Reach Point", self.handleArmMenu, True) 
        add_group_to_menu(self.menu, "Grasp Controls", ["Open Hand", "Close Hand"], self.handleArmMenu)
        add_to_menu(self.menu, "Reset Control Frame", self.handleArmMenu)
        
        self.setpoint_menu = MenuHandler()
        add_to_menu(self.setpoint_menu, "Move to Point", self.handleSetpointMenu)

        self.cart_marker = InteractiveMarker()
        self.cart_marker.header.frame_id = base_frame_id
        self.cart_marker.name = control_marker_id%self.side
        self.cart_marker.scale = 0.2
        self.makeArmControl()

        self.setpoint_marker = InteractiveMarker()
        self.setpoint_marker.header.frame_id = base_frame_id
        self.setpoint_marker.name = setpoint_marker_id%self.side
        self.setpoint_marker.scale = 0.2

        self.ResetSetpointOffset()
        self.ResetToolOffset()


        self.finger_control = FingerControl(self.server, self.side)
        self.joint_names = get_joint_names('/r2/%s_arm/joint'%self.side, 7) + self.finger_control.joint_names


        self.pose_pub = rospy.Publisher('/r2/r2_controller/%s/pose_command'%side,  PoseStamped)
        self.jnt_pub = rospy.Publisher('/r2/r2_controller/%s_arm/joint_command'%side,  JointState)
        
        self.cartReadyPose = PoseStamped()
        self.cartReadyPose.header.frame_id = "r2/waist_center"
        self.cartReadyPose.pose.position.x = 0.27
        self.cartReadyPose.pose.position.y = -0.35
        self.cartReadyPose.pose.position.z = -.22
        if side == 'left':
            self.cartReadyPose.pose.orientation.x = -0.707 
        else:
            self.cartReadyPose.pose.orientation.x = 0.707 
        self.cartReadyPose.pose.orientation.y = 0
        self.cartReadyPose.pose.orientation.z = 0
        self.cartReadyPose.pose.orientation.w = 0.707

        if side == 'left':
            Side = 'Left'
            self.jointReadyPose = make_joint_state(self.joint_names, [50.0, -80.0, -105.0, -140.0, 80.0, 0.0, 0.0]+[0.0]*12)
        else:
            Side = 'Right'
            self.jointReadyPose = make_joint_state(self.joint_names, [-50.0, -80.0, 105.0, -140.0, -80.0, 0.0, 0.0]+[0.0]*12)

        self.palm_mesh = "package://r2_description/meshes/%s_Palm.dae"%Side
        self.palm_mesh_pose = Pose()
        lq = get_quaternion(3.14, 0, 1.57)
        self.palm_mesh_pose.position.x = 0.0
        self.palm_mesh_pose.position.y = 0.0
        self.palm_mesh_pose.position.z = 0.0
        self.palm_mesh_pose.orientation.x = lq[0]
        self.palm_mesh_pose.orientation.y = lq[1]
        self.palm_mesh_pose.orientation.z = lq[2]
        self.palm_mesh_pose.orientation.w = lq[3]

        self.makeArmMenu()
        self.posture_marker = InteractiveMarker()
        self.makePostureMarkers()
        self.server.applyChanges()

    def SetArmToCartMode(self):
        rospy.wait_for_service('/r2/r2_controller/set_tip_name')
        set_tip_name = rospy.ServiceProxy('/r2/r2_controller/set_tip_name', SetTipName)
        frame = control_frame_id % self.side
        print "setting ", self.side, " tip to: ", frame
        try:
            resp1 = set_tip_name(self.side, frame)
            print "Set Tip Name: ", resp1.result
        except rospy.ServiceException, e: 
            print "Service call failed for ", self.side, " arm: %s"%e
     
    def SetArmToJointMode(self):
        rospy.wait_for_service('/r2/r2_controller/set_joint_mode')
        set_joint_mode = rospy.ServiceProxy('/r2/r2_controller/set_joint_mode', SetJointMode)
        
        print "setting ", self.side, " to joint mode"
        try:
            resp1 = set_joint_mode(self.side)
            print "Set ", self.side, " joint mode: ", resp1.result
        except rospy.ServiceException, e: 
            print "Service call failed for ", self.side, " arm: %s"%e

    def ResetToolOffset(self) :
        self.tool_offset = Pose()
        self.tool_offset.orientation.w = 1

    def ResetSetpointOffset(self) :
        self.setpoint_offset = Pose()
        self.setpoint_offset.orientation.w = 1

    def makeArmControl(self):
        self.cart_marker.controls += sixAxis()
        self.cart_marker.pose.position.x = .5
        self.cart_marker.pose.orientation.w = 1
        self.server.insert(self.cart_marker, self.handle_feedback)

    def removeArmControl(self) :
        self.server.erase(self.cart_marker)

    def makeSetpointControl(self) :
        self.setpoint_marker.controls += sixAxis()
        self.makeSetpointMarker()
        self.server.insert(self.setpoint_marker, self.handle_setpoint)
        self.setpoint_menu.apply( self.server, self.setpoint_marker.name )

    def makeSetpointMarker(self):
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.markers.append( marker_helper.makeMesh(self.setpoint_marker, self.palm_mesh, self.palm_mesh_pose, 1.1) )
        self.setpoint_marker.controls.append( control )

    def removeSetpointControl(self) :
        self.server.erase(self.setpoint_marker)

    def makeArmMenu(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = menu_frame_id%self.side
        int_marker.scale = 0.2
        int_marker.name = menu_frame_id%self.side

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        marker = makeMesh( int_marker, self.palm_mesh, self.palm_mesh_pose, 1.02 )
        control.markers.append( marker )
        int_marker.controls.append(control)

        self.server.insert(int_marker, self.handleArmMenu)
        self.menu.apply(self.server, int_marker.name)


    def handleArmMenu(self, feedback ) :
        if feedback.event_type != InteractiveMarkerFeedback.MENU_SELECT:
            return
        handle = feedback.menu_entry_id

        if(handle == 1) : # ready pose
            self.cartReadyPose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(self.cartReadyPose)

            self.jointReadyPose.header.stamp = rospy.Time.now()
            self.jnt_pub.publish(self.jointReadyPose) 

        elif(handle == 2) : # cart/joint mode
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("Setting Cartesian Control Mode for %s Arm"%self.side)
                self.SetArmToCartMode()
                self.joint_mode = False
                self.makeArmControl( )
            else:
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                rospy.loginfo("Setting Joint Control Mode for %s Arm"%self.side)
                self.SetArmToJointMode()
                self.joint_mode = True
                self.removeArmControl()

        elif(handle == 3) : # tool offset
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                self.edit_tool = False
                rospy.loginfo("Fixing %s Arm tool offset"%self.side)
            else:
                self.edit_tool = True
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                rospy.loginfo("Moving %s Arm tool offset "%self.side)
        
        elif(handle == 4) : # Setpoint
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("Resetting %s Arm Reach Point"%self.side)
                self.removeSetpointControl()
                self.makeArmControl()
                self.edit_setpoint = False
            else:
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                rospy.loginfo("Setting %s Arm Reach Point "%self.side)
                self.removeArmControl()
                self.ResetSetpointOffset()
                self.makeSetpointControl()
                self.edit_setpoint = True
                
        elif(handle == 6) :  # open hand
            print "open %s hand"%self.side
            self.finger.open_pose.header.stamp = rospy.Time.now()
            self.jnt_pub.publish(self.finger.open_pose) 
            self.server.resetMarker(control_marker_id % self.side)
        elif(handle == 7) : # close hand
            print "close left hand"
            self.finger.close_pose.header.stamp = rospy.Time.now()
            self.jnt_pub.publish(self.finger.close_pose) 
            self.server.resetMarker(control_marker_id % self.side)
        elif(handle == 8) : # reset tool offset
            print "Reseting Left Tool Control Frame"
            self.ResetToolOffset()
            self.server.resetMarker(control_marker_id % self.side)
        
        self.menu.reApply( self.server )

    def handleSetpointMenu(self, feedback ) :
        if feedback.event_type != InteractiveMarkerFeedback.MENU_SELECT:
            return
        pose = PoseStamped()
        handle = feedback.menu_entry_id
        now = rospy.Time(0)
        if(handle == 1) :
            #tf_listener.waitForTransform(frames.control_frame_id[0], base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.server.tf_listener.lookupTransform(base_frame_id, control_frame_id%self.side, now)
            pose.pose = pm.toPose(trans, rot) 

            Fp = pm.fromMsg(pose.pose)
            Fs = pm.fromMsg(setpoint_offset[0])
            #Ft = pm.fromMsg(tool_offset[0])

            pose.header.stamp = now
            pose.header.frame_id = base_frame_id
            pose.pose = pm.toMsg(Fp * Fs )#* Ft.Inverse()) 
            
            # send setpoint to arm
            self.pose_pub.publish(pose)

            # reset setpoint marker stuff
            self.removeSetpointControl()
            self.makeArmControl()
            self.menu.setCheckState( 4, MenuHandler.UNCHECKED )
            self.menu.reApply( self.server ) 
            self.edit_setpoint = False
        self.server.applyChanges()


    def makePostureMarkers(self):
        self.posture_marker.header.frame_id = posture_frame_id %self.side
        self.posture_marker.scale = 0.25
        self.posture_marker.name = posture_frame_id % self.side
        self.posture_marker.controls.append( makeYRotControl() )
        self.server.insert(self.posture_marker, self.handle_posture_feedback) 

    def handle_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        now = rospy.Time.now()
        if self.edit_tool:
            self.server.tf_listener.waitForTransform(control_frame_id %self.side, base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.server.tf_listener.lookupTransform(base_frame_id, control_frame_id %self.side, now)
            wTh = pm.toPose(trans, rot) 
            wTm = feedback.pose
            Fwth = pm.fromMsg(wTh)
            Fwtm = pm.fromMsg(wTm)
            Fhtw = (pm.fromMsg(wTh)).Inverse()
            Fhtm = Fhtw*Fwtm
            self.tool_offset = pm.toMsg(Fhtm) 
            print "Moved %s tool frame: "%self.side, self.tool_offset
        else:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = control_frame_id%self.side;

            pose.pose = feedback.pose
            self.server.tf_listener.waitForTransform(control_frame_id %self.side, base_frame_id, now, rospy.Duration(dur_time))
            ps = self.server.tf_listener.transformPose(base_frame_id, pose)
            Fp = pm.fromMsg(ps.pose)
            Fo = pm.fromMsg(self.tool_offset)
            # apply offset (inverse) and send to robot   
            pose.pose = pm.toMsg(Fp * Fo.Inverse()) 
            self.pose_pub.publish(pose)
            #print "Publish %s Pose"%self.side, pose
        self.server.applyChanges()


    def handle_setpoint(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        now = rospy.Time(0)
        if self.edit_setpoint:
            #tf_listener.waitForTransform(frames.control_frame_id[0], base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.server.tf_listener.lookupTransform(base_frame_id, control_frame_id % self.side, now)
            wTh = pm.toPose(trans, rot)
            Fwth = pm.fromMsg(wTh)
            wTs = feedback.pose
            Fwts = pm.fromMsg(wTs)
            Fhtw = (pm.fromMsg(wTh)*pm.fromMsg(self.tool_offset)).Inverse()
            Fhts = Fhtw*Fwts
            self.setpoint_offset = pm.toMsg(Fhts) 
            print "Moved %s setpoint frame: "%self.side, self.setpoint_offset
        self.server.applyChanges()



    def handle_posture_feedback(self, feedback):
        if feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            return
        self.jnt_pub.publish( self.server.get_joint_command('/r2/%s_arm/joint1'%self.side, feedback.pose.orientation.z))
        self.server.resetMarker(feedback.marker_name)
        self.server.applyChanges()


    def slow_update(self):
        self.server.tf_listener.waitForTransform( control_frame_id%self.side, base_frame_id, rospy.Time(0), rospy.Duration(5.0))

    def fast_update(self):
        now = rospy.Time(0)
        frame = control_frame_id%self.side
        self.server.tf_listener.waitForTransform( frame, base_frame_id, now, rospy.Duration(dur_time))

        (trans, rot) = self.server.tf_listener.lookupTransform(base_frame_id, frame, now)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = base_frame_id
        pose.pose = pm.toPose(trans, rot) 
        
        # append tool offset to cart marker
        if not self.edit_setpoint :
            Fp = pm.fromMsg(pose.pose)
            Ft = pm.fromMsg(self.tool_offset)
            pose.pose = pm.toMsg(Fp*Ft) 
            pose.header.frame_id = base_frame_id  
            self.server.setPose( self.cart_marker.name, pose.pose )
        # append setpoint offset to marker
        else :
            Fp = pm.fromMsg(pose.pose)
            Fs = pm.fromMsg(self.setpoint_offset)
            Ft = pm.fromMsg(self.tool_offset)
            pose.pose = pm.toMsg(Fp*Ft*Fs) 
            pose.header.frame_id = base_frame_id  
            self.server.setPose( self.setpoint_marker.name, pose.pose )
        self.server.applyChanges()


