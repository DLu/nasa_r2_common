from r2_teleop import *
from r2_teleop.ArmControl import get_arm_control_frame
import copy

gaze_frame_id = '/r2/gaze_control_link'

class GazeControl:
    def __init__(self, server):
        self.server = server
        self.marker = InteractiveMarker()

        self.marker.header.frame_id = base_frame_id
        self.marker.pose.position.x = -1.5
        self.marker.pose.position.z = 0
        self.marker.scale = 0.2
        self.marker.name = gaze_frame_id
        
        self.menu = MenuHandler()
        add_to_menu(self.menu, "Look At This Point", self.handleGazeMenu)
        add_group_to_menu(self.menu, "Tracking Mode", ["Free Floating", "Left Tool", "Right Tool"], self.handleGazeMenu)

        self.control_mode = False
        self.tracking_mode = False
        self.point_tracking_mode = False
        self.left_tracking_mode = False
        self.right_tracking_mode = False
        self.pose_pub  = rospy.Publisher('/r2/r2_controller/gaze/pose_command',  PoseStamped)

    def handleGazeMenu(self, feedback ) :
        pose = PoseStamped()
        handle = feedback.menu_entry_id
       
        if(handle == 1) :
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = base_frame_id
            pose.pose = feedback.pose
            self.pose_pub.publish(pose)
            self.menu.reApply( self.server ) 
        
        elif(handle == 3) :
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("turning off gaze point tracking control")
                self.tracking_mode = False
                self.point_tracking_mode = False
                self.removeGazeControl()
            else:
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                rospy.loginfo("turning on gaze point tracking control")
                self.menu.setCheckState( handle+1, MenuHandler.UNCHECKED )
                self.menu.setCheckState( handle+2, MenuHandler.UNCHECKED )
                self.tracking_mode = True
                self.point_tracking_mode = True
                self.left_tracking_mode = False
                self.right_tracking_mode = False
            self.menu.reApply( self.server )

        elif(handle == 4) :
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("turning off gaze left tracking control")
                self.tracking_mode = False
                self.left_tracking_mode = False
            else:
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                self.menu.setCheckState( handle-1, MenuHandler.UNCHECKED )
                self.menu.setCheckState( handle+1, MenuHandler.UNCHECKED )
                rospy.loginfo("turning on gaze left tracking control")
                self.tracking_mode = True
                self.point_tracking_mode = False
                self.left_tracking_mode = True
                self.right_tracking_mode = False
                self.control_mode = False #PROBABLY
                self.removeGazeControl()
            self.menu.reApply( self.server )

        elif(handle == 5) :
            state = self.menu.getCheckState( handle )
            if state == MenuHandler.CHECKED:
                self.menu.setCheckState( handle, MenuHandler.UNCHECKED )
                rospy.loginfo("turning off gaze right tracking control")
                self.tracking_mode = False
                self.right_tracking_mode = False
            else:
                self.menu.setCheckState( handle, MenuHandler.CHECKED )
                self.menu.setCheckState( handle-1, MenuHandler.UNCHECKED )
                self.menu.setCheckState( handle-2, MenuHandler.UNCHECKED )
                rospy.loginfo("turning on gaze right tracking control")
                self.tracking_mode = True
                self.point_tracking_mode = False
                self.left_tracking_mode = False
                self.right_tracking_mode = True
                self.control_mode = False #PROBABLY
                self.removeGazeControl()
            self.menu.reApply( self.server )

    def toggleGazeControl(self) :
        self.control_mode = not self.control_mode
        if self.control_mode :
            self.makeGazeControl()
            self.tracking_mode = False
            self.point_tracking_mode = False
            self.left_tracking_mode = False
            self.right_tracking_mode = False
            self.menu.setCheckState( 3, MenuHandler.UNCHECKED )
            self.menu.setCheckState( 4, MenuHandler.UNCHECKED )
            self.menu.setCheckState( 5, MenuHandler.UNCHECKED )    
        else :
            self.removeGazeControl()
            self.tracking_mode = False
            self.point_tracking_mode = False
            self.left_tracking_mode = False
            self.right_tracking_mode = False
            self.menu.setCheckState( 3, MenuHandler.UNCHECKED )
            self.menu.setCheckState( 4, MenuHandler.UNCHECKED )
            self.menu.setCheckState( 5, MenuHandler.UNCHECKED )    
        self.menu.reApply( self.server )

    def makeGazeControl(self, sphere_scale=0.5):
        makeSphereControl(self.marker, sphere_scale)
        self.marker.controls += sixAxis()
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        marker = makeSphere( self.marker, sphere_scale*1.01 )
        control.markers.append( marker )
        self.marker.controls.append(control)

        self.server.insert(self.marker, self.handle_feedback)
        self.menu.apply( self.server, self.marker.name )


    def removeGazeControl(self) :
        self.server.erase(self.marker)

    def handle_feedback(self, feedback):
        if self.tracking_mode and feedback.event_type != InteractiveMarkerFeedback.POSE_UPDATE:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = base_frame_id
            pose.pose = copy.deepcopy(feedback.pose)
            self.server.setPose( gaze_frame_id, pose.pose )
            self.server.applyChanges()
            self.pose_pub.publish(pose)
        elif self.tracking_mode and feedback.event_type != InteractiveMarkerFeedback.MOUSE_UP:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = base_frame_id
            pose.pose = feedback.pose
            self.pose_pub.publish(pose)
        self.server.applyChanges()


    #TODO: Control_frame_id
    def slow_update(self):
        now = rospy.Time.now()
        if self.left_tracking_mode:
            self.server.tf_listener.waitForTransform(get_arm_control_frame('left'), base_frame_id, now, rospy.Duration(5.0))
        elif self.right_tracking_mode:
            self.server.tf_listener.waitForTransform(get_arm_control_frame('right'), base_frame_id, now, rospy.Duration(5.0))

    def fast_update(self) :
        pose = PoseStamped()
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = base_frame_id

        now = rospy.Time(0)
        if self.left_tracking_mode or self.right_tracking_mode:
            if self.left_tracking_mode:
                frame = get_arm_control_frame('left')
            else:
                frame = get_arm_control_frame('right')
            self.server.tf_listener.waitForTransform(frame, base_frame_id, now, rospy.Duration(dur_time))
            (trans, rot) = self.server.tf_listener.lookupTransform(base_frame_id, frame, now)
            pose.pose = pm.toPose(trans, rot) 
            self.pose_pub.publish(pose)
        

        
