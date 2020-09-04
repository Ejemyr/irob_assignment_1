#!/usr/bin/env python2
import rospy
import actionlib
import irob_assignment_1.msg
from irob_assignment_1.srv import GetSetpoint, GetSetpointRequest, GetSetpointResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import tf2_ros
import tf2_geometry_msgs
from math import atan2, hypot

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

# Max linear velocity (m/s)
max_linear_velocity = 0.5
# Max angular velocity (rad/s)
max_angular_velocity = 1.0
# Max percentage of max angular velocity while running
max_rot_and_drive = 0.3

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
        msg.angular.z = min(max_angular_velocity, atan2(transformed_setpoint.point.y, transformed_setpoint.point.x))
        if msg.angular.z / max_angular_velocity <= max_rot_and_drive:
            msg.linear.x = min(max_linear_velocity, hypot(transformed_setpoint.point.x, transformed_setpoint.point.y))

        # Publish Twist
        pub.publish(msg)

        # Call service client again if the returned path is not empty and do stuff again
        path = response.new_path

    # Send 0 control Twist to stop robot
    pub.publish(Twist())


def get_path():
    global goal_client, goal_client_name

    # Get path from action server
    goal = irob_assignment_1.msg.GetNextGoalActionGoal()
    rospy.loginfo("Action server %s: Sending goal for path..." % goal_client_name)
    goal_client.send_goal(goal)
    rospy.loginfo("Action server %s: Sent!" % goal_client_name)
    rospy.loginfo("Action server %s: Waiting for result..." % goal_client_name)
    goal_client.wait_for_result()
    rospy.loginfo("Action server %s: Found!" % goal_client_name)

    # Call move with path from action server
    res = goal_client.get_result()
    if res.gain:
        rospy.loginfo("Gain of path is: %s" % res.gain)
        move(res.path)
        return True
    else:
        return False


if __name__ == "__main__":
    try:
        # Init node
        rospy.init_node('controller', anonymous=True)
        rospy.loginfo("Node %s: Inited!" % rospy.get_name())
        
        # Init publisher
        pub = rospy.Publisher(pub_name, Twist, queue_size=10)
        rospy.loginfo("Publisher %s: Inited!" % pub_name)

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
        while not rospy.is_shutdown() and get_path():
            pass

        # Spin
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
