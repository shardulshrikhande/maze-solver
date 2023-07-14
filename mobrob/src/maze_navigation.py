#!/usr/bin/env python3

import rospy
import traceback
import numpy as np
from std_msgs.msg import Float32, Bool
# IMPORT the custom message:
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439WaypointXY")
from mobrob_util.msg import ME439WaypointXY, RobotDistance

##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you could skip segments if you wanted to)
waypoint_number = 0  # for waypoint seeking.
path_complete = Bool()
path_complete.data = False

# Get the parameter that determines how close to a waypoint the robot must be to call it "arrived".
waypoint_tolerance = rospy.get_param('/waypoint_tolerance')

# global variable to hold the waypoint currently being tracked

# Trevor wanted this to toggle when we are using the sensors currently has no function
useSensorsForNavigation = 1
distance_to_target_x = 0
distance_to_target_z = 0
# original code from set waypoint
waypoint_complete = Bool()
waypoint_complete.data = False

robot_number = rospy.get_param('/robot_number')
wall_front_distance = 10
wall_left_distance = 10
wall_right_distance = 10
Nxt_point_x = 0
Nxt_point_y = 0
direction = 0 #0=+y, 1=+x, 2=-y, 3=-x

waypoints = np.array([[0, 0]])
nxt_waypoints = np.array([[0, 0]])

# gets the distance we want to travel for each waypoint from yaml
Dist = rospy.get_param('/wall_distance_tolerance')
DistWaypointsLeft = rospy.get_param('/distance_between_waypoints_left')
DistWaypointsRight = rospy.get_param('/distance_between_waypoints_right')
DistWaypointsFront = rospy.get_param('/distance_between_waypoints_front')

def store_left_dist(float_msg_in):
    global wall_left_distance
    wall_left_distance = float_msg_in.data


def store_right_dist(float_msg_in):
    global wall_right_distance
    wall_right_distance = float_msg_in.data


def store_front_dist(float_msg_in):
    global wall_front_distance
    wall_front_distance = float_msg_in.data


def Maze_Nav(msg_in):
    global Dist, waypoint_number, Nxt_point_x, Nxt_point_y, direction, nxt_waypoints, waypoints, wall_front_distance, wall_left_distance, wall_right_distance
    if (useSensorsForNavigation == 1):
        if (robot_number == 1):
            waypoint_number += 1
            # Increments waypoint number
            if(wall_left_distance > DistWaypointsLeft): #turn left
                direction = direction-1
            elif (wall_front_distance > DistWaypointsFront): #go forward
                direction = direction
            elif (wall_right_distance > DistWaypointsRight): #go right
                direction = direction+1
            else: #go back
                direction = direction+2
            direction = direction%4
            if(direction == 0): #+y
                Nxt_point_y = Nxt_point_y+Dist
            elif(direction == 1): #+x
                Nxt_point_x = Nxt_point_x+Dist
            elif(direction == 2): #-y
                Nxt_point_y = Nxt_point_y-Dist
            elif(direction == 3): #-x
                Nxt_point_x = Nxt_point_x-Dist
            msg_waypoint = ME439WaypointXY()
            msg_waypoint.x = Nxt_point_x
            msg_waypoint.y = Nxt_point_y
            pub_waypoint_xy.publish(msg_waypoint)
        else:
            waypoint_number += 1
            # Increments waypoint number
            if(wall_left_distance > DistWaypointsRight): #turn right
                direction = direction+1
            elif (wall_front_distance > DistWaypointsFront): #go forward
                direction = direction
            elif (wall_right_distance > DistWaypointsLeft): #go left
                direction = direction-1
            else: #go back
                direction = direction+2
            direction = direction%4
            if(direction == 0): #+y
                Nxt_point_y = Nxt_point_y+Dist
            elif(direction == 1): #+x
                Nxt_point_x = Nxt_point_x+Dist
            elif(direction == 2): #-y
                Nxt_point_y = Nxt_point_y-Dist
            elif(direction == 3): #-x
                Nxt_point_x = Nxt_point_x-Dist
            msg_waypoint = ME439WaypointXY()
            msg_waypoint.x = Nxt_point_x
            msg_waypoint.y = Nxt_point_y
            pub_waypoint_xy.publish(msg_waypoint)
    else :
        pass # Graph solving and communication of points between robots would happen here

# Publish desired waypoints at the appropriate time.
def talker():
    global waypoints, waypoint_number, path_complete, pub_waypoint_xy
    # Launch this node with the name "set_waypoints"
    rospy.init_node('maze_navigation', anonymous=False)

    # Create a subscriber that listens for messages on the "waypoint_complete" and "waypoint_xy" topics
    sub_waypoint_complete = rospy.Subscriber(
        '/waypoint_complete', Bool, Maze_Nav)
    pub_waypoint_xy = rospy.Publisher(
        '/waypoint_xy', ME439WaypointXY, queue_size=1)

    # Declare the message to publish.
    msg_waypoint = ME439WaypointXY()
    msg_waypoint.x = 0
    msg_waypoint.y = 0

    sub_wall_front_distance = rospy.Subscriber('/wall_distance_front', Float32,store_front_dist)
    sub_wall_left_distance = rospy.Subscriber('/wall_distance_left', Float32,store_left_dist)
    sub_wall_right_distance = rospy.Subscriber('/wall_distance_right', Float32,store_right_dist)

    sub_target_distance = rospy.Subscriber('/target_distance', RobotDistance,target_seen)

    rospy.sleep(1)

    pub_waypoint_xy.publish(msg_waypoint)

    rospy.spin()

# =============================================================================
# # Function to publish waypoints in sequence:
# # A Callback for whenever the '/waypoint_complete' topic comes in.
# # This function increments the waypoint_number whenever one waypoint is satisfied.
# # NOTE it does Not actually publish the new waypoint,
# # because that's happening in the loop above.
# # This function also checks if the whole path is done (if there are not more waypoints)
# # If so it publishes a "path_complete" message with value True.
# =============================================================================
def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint

    # # If the message (stored in variable 'msg_in') tells us that '/waypoint_complete' is True,
    # # Then increment the waypoint number (variable 'waypoint_number'.
    if msg_in.data:
        waypoint_number = waypoint_number + 1

    # # Handle the special case of the last waypoint:
    # # If the last waypoint was reached, set "path_complete" and publish it
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
        # waypoint_number = waypoint_number - 1  # This line prevents an array out of bounds error to make sure the node stayed alive. By commenting, allow it to increment past the end, which will throw an exception (array out of bounds) the next time it publishes a waypoint and cause the node to die.
    else:
        path_complete.data = False

def target_seen(msg_in):
    global distance_to_target_x, distance_to_target_z, useSensorsForNavigation
    distance_to_target_x = msg_in.x
    distance_to_target_z = msg_in.z
    useSensorsForNavigation = 0

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
