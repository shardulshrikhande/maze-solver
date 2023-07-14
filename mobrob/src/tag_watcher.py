#!/usr/bin/env python3

# Utilize OpenCV to send x and y distance over topic /target_distance when a specific tag is seen
# Work should be done in a new file called tag_watcher.py
# Modify maze_navigation.py to close the distance based upon parameter controlling minimum 
#  distance from end tag
# A global variable (useSensorsForNavigation) should also be changed to 0 to disable movement 
# based upon sensors
# Kick off graph solving once distance has been closed using topic /solve_maze_graph
# Upon indication that the solution commands/points are processed (related to /maze_solution_sent),
#  trigger resume of movement using calculated waypoints using follow_solution_path method
#  (a global variable is likely needed here to modify behavior of subscriber to /waypoint_complete

import rospy
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import String
from mobrob_util.msg import RobotDistance

# Create Publishers that will eventually be used to publish distances to topics
publish_xz = rospy.Publisher('/target_distance', RobotDistance, queue_size=1)

def tag_watcher():
    #initialize the node
    rospy.init_node('tag_watcher', anonymous=False)

    #Subscribe to the aruco topic
    tag_watch = rospy.Subscriber('/aruco', String, callback_tag_distance)

    #spin to keep this node active
    rospy.spin()

def callback_tag_distance(msg_in):
    tag_msg = msg_in.data
    tag_msg = tag_msg.split(',')
    x_distance = tag_msg[1].strip()
    z_distance = tag_msg[3].strip()
    distance_msg = x_distance + "," + z_distance

    publish_xz.publish(distance_msg)


# Error handling code to run aruco tag watcher
if __name__ == '__main__':
    try: 
        tag_watcher()
    except: 
        traceback.print_exc()
        pass
