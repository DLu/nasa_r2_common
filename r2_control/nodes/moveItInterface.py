#! /usr/bin/env python

import sys

import rospy
import roslib; roslib.load_manifest('r2_control')

import geometry_msgs.msg
import visualization_msgs.msg
import sensor_msgs.msg
import moveit_commander
import moveit_msgs.msg
import tf
import PyKDL as kdl

class MoveItInterface :

    def __init__(self, N, robotName, groupName):

        self.numJoints = N
        self.groupName = groupName
        self.robotName = robotName

        print "============ Setting up MoveIt! for robot: \'", self.robotName, "\', group: \'", self.groupName, "\'"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(self.groupName)
        self.obstacleMarkers = visualization_msgs.msg.MarkerArray()

        rospy.Subscriber(str(self.robotName + "/joint_states"), sensor_msgs.msg.JointState, self.jointStateCallback)
        self.trajectoryPublisher = rospy.Publisher(str('/' + self.robotName + '/' + self.groupName + '/move_group/display_planned_path'), moveit_msgs.msg.DisplayTrajectory)
        self.obstaclePublisher = rospy.Publisher(str('/' + self.robotName + '/' + self.groupName + '/obstacle_markers'), visualization_msgs.msg.MarkerArray)

        self.tfListener = tf.TransformListener()
        rospy.sleep(2)


    def printBasicInfo(self) :
        print "============================================================"
        print "============ Robot Name: %s" % self.robotName
        print "============ Group Name: %s" % self.groupName
        print "============ Planning frame: %s" % self.group.get_planning_frame()
        print "============ End-Effector link: %s" % self.group.get_end_effector_link()
        print "============ Printing robot state"
        print self.robot.get_current_state()
        print "============================================================"

    def getNumJoints(self) :
        return self.numJoints

    def jointStateCallback(self, data):
        self.currentState = data

    def createJointPlanToTarget(self, js) :
        print "========= Robot Name: %s" % self.robotName
        print "============ MoveIt! Group Name: %s" % self.groupName
        print "============ Generating Joint plan"
        self.group.set_joint_value_target(js)
        plan = self.group.plan()

        if plan != None :
            displayTrajectory = moveit_msgs.msg.DisplayTrajectory()
            displayTrajectory.trajectory_start = self.robot.get_current_state()
            displayTrajectory.trajectory.append(plan)
            self.trajectoryPublisher.publish(displayTrajectory)
            return True
        else :
            return False

    def createPlanToTarget(self, pt) :

        if pt.header.frame_id != self.group.get_planning_frame() :
            self.tfListener.waitForTransform(pt.header.frame_id, self.group.get_planning_frame(), rospy.Time(0), rospy.Duration(2.0))
            pt = self.tfListener.transformPose(self.group.get_planning_frame(), pt)

        print "========= Robot Name: %s" % self.robotName
        print "============ MoveIt! Group Name: %s" % self.groupName
        print "============ Generating plan"
        self.group.set_pose_target(pt)
        plan = self.group.plan()

        if plan != None :
            displayTrajectory = moveit_msgs.msg.DisplayTrajectory()
            displayTrajectory.trajectory_start = self.robot.get_current_state()
            displayTrajectory.trajectory.append(plan)
            self.trajectoryPublisher.publish(displayTrajectory)
            return True
        else :
            return False

    def createRandomTarget(self) :
        print "========= Robot Name: %s" % self.robotName
        print "============ MoveIt! Group Name: %s" % self.groupName
        print "============ Generating random plan"
        self.group.set_random_target()
        plan = self.group.plan()
        if plan != None :
            displayTrajectory = moveit_msgs.msg.DisplayTrajectory()
            displayTrajectory.trajectory_start = self.robot.get_current_state()
            displayTrajectory.trajectory.append(plan)
            self.trajectoryPublisher.publish(displayTrajectory)
            return True
        else :
            return False

    def executePlan(self) :
        r = self.group.go(True)
        print "============ Plan Execution: %s" % r
        return r

    def addCollisionObject(self, p, s, n) :
        p.header.frame_id = self.robot.get_planning_frame()
        self.scene.add_box(n, p, s)

        m = visualization_msgs.msg.Marker()
        m.header.frame_id = p.header.frame_id
        m.type = m.CUBE
        m.action = m.ADD
        m.scale.x = s[0]
        m.scale.y = s[1]
        m.scale.z = s[2]
        m.color.a = 0.8
        m.color.r = 0
        m.color.g = 1
        m.color.b = 0
        m.pose = p.pose
        m.text = n
        m.ns = n
        self.obstacleMarkers.markers.append(m)

        print self.obstacleMarkers.markers
        self.obstaclePublisher.publish(self.obstacleMarkers)

if __name__ == '__main__':

    rospy.init_node('moveItIntefraceTest')

    moveit_commander.roscpp_initialize(sys.argv)

    try:
        moveItTest = []
        moveItTest.append(MoveItInterface(7, "r2", "right_arm"))
        moveItTest.append(MoveItInterface(7, "r2", "left_arm"))

        moveItTest[0].printBasicInfo()
        moveItTest[1].printBasicInfo()
        # collision_object = moveit_msgs.msg.CollisionObject()

        # p1 = geometry_msgs.msg.PoseStamped()
        # p1.pose.position.x = 0.7
        # p1.pose.position.y = 0.0
        # p1.pose.position.z = 0.4
        # p1.pose.orientation.w = 1.0
        # moveItTest.addCollisionObject(p1, (0.2, 2.0, 0.2), "box1")

        # p2 = geometry_msgs.msg.PoseStamped()
        # p2.pose.position.x = 0.6
        # p2.pose.position.y = 0.0
        # p2.pose.position.z = 1.1
        # p2.pose.orientation.w = 1.0
        # moveItTest.addCollisionObject(p2, (0.2, 2.0, 0.2), "box2")


        # for i in range(5) :

        #     pt = geometry_msgs.msg.Pose()
        #     pt.position.x = 1.0+(random.random()/5.0)
        #     pt.position.y = 0.5 - random.random()
        #     pt.position.z = 0.3+(0.5-random.random()/2)
        #     pt.orientation.w = 1.0

        #     # moveItTest.createPlanToTarget(pt)
        #     moveItTest.createRandomTarget()
        #     r = moveItTest.executePlan()
        #     if not r : rospy.logerr("couldn't execute plan")
        #     rospy.sleep(2)


        q = (kdl.Rotation.RPY(-1.57,0,0)).GetQuaternion()
        pt = geometry_msgs.msg.PoseStamped()
        pt.header.frame_id = "world"
        pt.header.seq = 0
        pt.header.stamp = rospy.Time.now()
        pt.pose.position.x = -0.3
        pt.pose.position.y = -0.5
        pt.pose.position.z = 1.2
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]
        moveItTest[0].createPlanToTarget(pt)
        # rospy.sleep(2)

        q = (kdl.Rotation.RPY(1.57,0,-1.57)).GetQuaternion()
        pt = geometry_msgs.msg.PoseStamped()
        pt.header.frame_id = "world"
        pt.header.seq = 0
        pt.header.stamp = rospy.Time.now()
        pt.pose.position.x = 0.3
        pt.pose.position.y = -0.5
        pt.pose.position.z = 1.2
        pt.pose.orientation.x = q[0]
        pt.pose.orientation.y = q[1]
        pt.pose.orientation.z = q[2]
        pt.pose.orientation.w = q[3]
        moveItTest[1].createPlanToTarget(pt)

        r1 = moveItTest[0].executePlan()
        r2 = moveItTest[1].executePlan()
        if not r1 : rospy.logerr("moveItTest(1) -- couldn't execute plan")
        if not r2 : rospy.logerr("moveItTest(2) -- couldn't execute plan")

        moveit_commander.roscpp_shutdown()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass




