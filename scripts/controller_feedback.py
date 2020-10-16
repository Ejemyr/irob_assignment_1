#!/usr/bin/env python3
from math import atan2, hypot, pi

import actionlib
import irob_assignment_1.msg
import rospy
import tf2_py
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Twist
from irob_assignment_1.srv import (GetSetpoint, GetSetpointRequest,
                                   GetSetpointResponse)
from nav_msgs.msg import Path

# Use to transform between frames
tf_buffer = None
listener = None

# The exploration simple action client
goal_client_name = 'get_next_goal'
goal_client = None
# The collision avoidance service client
control_client_name = 'get_setpoint'
control_client = None
# The velocity command publisher
pub_name = 'cmd_vel'
pub = None

# The robots frame
robot_frame_id = "base_link"

# Min allowed gain to move along path (in feedback)
min_allowed_gain = 3

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1
# Max percentage of max angular velocity while running
max_rot_and_drive = 0.5

# Bool for all explored
all_explored = False


def goal_active():
    rospy.loginfo("Action server %s: Sent!" % goal_client_name)


def goal_feedback(feedback):
    global min_allowed_gain, goal_client, goal_client_name

    rospy.loginfo("Action server %s: Got feedback." % goal_client_name)

    # Check if this path has higher gain than min_allowed_gain
    if feedback.path and feedback.gain > min_allowed_gain:
        # Cancel action server
        goal_client.cancel_goal()
        rospy.loginfo("Action server %s: Canceled. Got feedback path with gain %f." % (goal_client_name, feedback.gain))
        
        # Follow path
        move(feedback.path)


def goal_result(state, result):
    global all_explored

    rospy.loginfo("I got a result")

    # If the state is succeeded then
    if actionlib.TerminalState.SUCCEEDED == state:
        rospy.loginfo("Action server %s: Returned succeeded")
        # Move along the path if path is not empty
        if result.gain:
            rospy.loginfo("Gain of path is: %s" % result.gain)
            move(result.path)
        else:
            rospy.loginfo("All space explored!")

    elif actionlib.TerminalState.RECALLED == state:
        rospy.loginfo("Action server %s: Returned recalled")
    elif actionlib.TerminalState.REJECTED == state:
        rospy.loginfo("Action server %s: Returned rejected")
    elif actionlib.TerminalState.PREEMPTED == state:
        rospy.loginfo("Action server %s: Returned preempted")
    elif actionlib.TerminalState.ABORTED == state:
        rospy.loginfo("Action server %s: Returned aborted")
    elif actionlib.TerminalState.LOST == state:
        rospy.loginfo("Action server %s: Returned lost")


def move(path):
    global control_client, control_client_name, robot_frame_id, pub, tf_buffer, max_angular_velocity, max_linear_velocity, max_rot_and_drive

    # Call service client with path
    rate = rospy.Rate(20)
    while path.poses:
        response = control_client(path)

        # Transform Setpoint from service client
        try:
            trans = tf_buffer.lookup_transform(robot_frame_id, 'map', rospy.Time())
        except Exception:
            rate.sleep()
            continue
        transformed_setpoint = tf2_geometry_msgs.do_transform_point(response.setpoint, trans)

        # Create Twist message from the transformed Setpoint
        msg = Twist() 

        # Fix issues with 2nd and 3rd quadrant
        msg.angular.z = min(max_angular_velocity, atan2(transformed_setpoint.point.y, transformed_setpoint.point.x))
        if msg.angular.z / max_angular_velocity <= max_rot_and_drive:
            msg.linear.x = min(max_linear_velocity, hypot(transformed_setpoint.point.x, transformed_setpoint.point.y))


        # Publish Twist
        pub.publish(msg)

        # Call service client again if the returned path is not empty and do stuff again
        path = response.new_path

        rate.sleep()

        if rospy.is_shutdown():
            break


    if not rospy.is_shutdown():
        # Send 0 control Twist to stop robot
        pub.publish(Twist())

        # Get new path
        get_path()
    


def get_path():
    global goal_client, goal_client_name

    # Get path from action server
    goal = irob_assignment_1.msg.GetNextGoalActionGoal()
    rospy.loginfo("Action server %s: Sending goal for path..." % goal_client_name)
    goal_client.send_goal(goal, active_cb=goal_active, done_cb=goal_result, feedback_cb=goal_feedback)


def shutdown_hook():
    global goal_client
    rospy.loginfo("Shutting down gracefully!")
    goal_client.stop_tracking_goal()
    goal_client.cancel_all_goals()
    control_client.close()


if __name__ == "__main__":
    try:
        # Init node
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo("Node %s: Inited!" % rospy.get_name())
        
        # Init publisher
        pub = rospy.Publisher(pub_name, Twist, queue_size=10)
        rospy.loginfo("Publisher %s: Inited!" % pub_name)

        # Send velocity = 0 if this is interupted
        rospy.on_shutdown(shutdown_hook)

        # Init simple action client
        goal_client = actionlib.SimpleActionClient(goal_client_name, irob_assignment_1.msg.GetNextGoalAction)
        rospy.loginfo("Action server %s: Waiting..." % goal_client_name)
        goal_client.wait_for_server()
        rospy.loginfo("Action server %s: Found!" % goal_client_name)

        # Init service client
        rospy.loginfo("Service %s: Waiting..." % control_client_name)
        rospy.wait_for_service(control_client_name)
        control_client = rospy.ServiceProxy(control_client_name, GetSetpoint)
        rospy.loginfo("Service %s: Found!..." % control_client_name)

        # Setup tf listener
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        # Call get path
        get_path()

        # Spin
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
