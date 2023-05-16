#!/usr/bin/env python3
"""
Initilization Pure Pursuit node for vehicle control
"""
# import math
import rospy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose , PoseStamped
# from pure_pursuit import (
#     WayPoints,
#     State,
#     PidController,
#     purepursuitSteercontrol,
#     Position,
#     plot,
# )
from pure_pursuit import WayPoints, State, purepursuitSteercontrol, Position, PidController
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose,PoseStamped
from tf_helper import *
from tf_helper import TFHelper
import tf
import tf2_ros
#import marker msg
from visualization_msgs.msg import Marker


tf_buffer =tf2_ros.Buffer
waypoints = None
helper = None
# PLOTTING = rospy.get_param("/plotting")
# PLOTNAME = rospy.get_param("/plotName")
# TARGETSPEED = rospy.get_param("/targetSpeed")

def path_cb(msg):
    global helper, waypoints
    print("path_cb")
    waypoints.add(helper.transformMsg(msg, "cg"))
def main() -> None:
    """
    Main function for pure pursuit vehicle control node, subscribes to
    state and waypoints and publishes control actions, also plots the
    path and vehicle trajectory if plotting is set to true
    """
    global waypoints, helper
    rospy.init_node("purepursuit_controller", anonymous=True)
    helper = TFHelper("control")
    TARGETSPEED = 7.0
    controlActionPub = rospy.Publisher("/control_actions", AckermannDrive, queue_size=10)
    targetPosePub = rospy.Publisher("/targetPoint", Marker, queue_size=10)
    waypoints = WayPoints()
    waypoints.xList = [0.0]
    waypoints.yList = [0.0]
    pose = Position(0.0, 0.0)
    print(State)
    state = State(pose, 0.0)
    rospy.Subscriber("/tf_pose", PoseStamped, callback=state.update)
    rospy.Subscriber("/pathplanning/waypoints", Path, callback=path_cb)
    targetPose = Marker()
    controlAction = AckermannDrive()
    pidController = PidController()
    rate = rospy.Rate(10)
    targetInd = 0
    prevError = 0
    while not rospy.is_shutdown():

        delta, targetInd = purepursuitSteercontrol(state, waypoints, targetInd)

        _, error = pidController.proportionalControl(TARGETSPEED, 0.99, prevError)
        # longitudinal controller
        prevError = error
        #Make marker message for targetPose
        targetPose.pose.position.x = waypoints.xList[targetInd]
        targetPose.pose.position.y = waypoints.yList[targetInd]
        targetPose.pose.position.z = -0.5
        targetPose.action = Marker.ADD
        targetPose.header.frame_id = "cg"
        targetPose.header.stamp = rospy.Time.now()
        targetPose.type = Marker.SPHERE
        targetPose.id = 0
        targetPose.color.r = 1.0
        targetPose.ns = "targetPoint"
        targetPose.color.a = 1.0
        targetPose.pose.orientation.w = 1.0
        targetPose.scale.x = 0.5
        targetPose.scale.y = 0.5
        targetPose.scale.z = 0.5

        # targetPose.pose.position.x = waypoints.xList[targetInd]
        # targetPose.pose.position.y = waypoints.yList[targetInd]
        # targetPose.header.frame_id = "Obj_F"
        # targetPose.header.stamp = rospy.Time.now()

        # deltaDegree = math.degrees(delta)
        #controlAction.acceleration = acc  # proportionalControl(targetSpeed, state.currentSpeed)
        controlAction.steering_angle = delta
        controlAction.jerk = TARGETSPEED# velocity set point
        # print(state.currentSpeed)
        controlAction.speed = state.currentSpeed  # current velocity
        #controlAction.steering_angle_velocity = waypoints.yList[targetInd]
        controlActionPub.publish(controlAction)
        targetPosePub.publish(targetPose)
        # if PLOTTING:

        #     plot(waypoints, state, PLOTNAME, targetInd)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass