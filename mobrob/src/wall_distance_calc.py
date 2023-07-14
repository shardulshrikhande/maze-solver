#!/usr/bin/env python3
# Wall Distance Calculation Program

# Goals
#Process information from the ultrasonic sensors to determine the distance...
#  ...from the sensors to the maze walls
#Distances will ideally be in relation to the center of the robot to make controlling the robot easier
#Parameters for each sensor will ideally be used due to the potential need for calibration
#Information published to topics /wall_left_distance, /wall_right_distance, /wall_front_distance

import rospy
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Float32
from mobrob_util.msg import ME439SensorsProcessed

# Global Variables for Manual Calibration
# Code for gathering error values from robot_info.yaml file
analog_error_left = rospy.get_param('/analog_error_left')
analog_error_right = rospy.get_param('/analog_error_right')
analog_error_front = rospy.get_param('/analog_error_front')

# Create Publishers that will eventually be used to publish distances to topics
publish_dist_left = rospy.Publisher('/wall_distance_left', Float32, queue_size=1)
publish_dist_left_graph = rospy.Publisher('/wall_distance_left_graph', Float32, queue_size=1)
publish_dist_right = rospy.Publisher('/wall_distance_right', Float32, queue_size=1)
publish_dist_right_graph = rospy.Publisher('/wall_distance_right_graph', Float32, queue_size=1)
publish_dist_front = rospy.Publisher('/wall_distance_front', Float32, queue_size=1)
publish_dist_front_graph = rospy.Publisher('/wall_distance_front_graph', Float32, queue_size=1)

# Main code that will run and listen to the sensors
def sensors_listener():
    # Initialize the node
    rospy.init_node('wall_dist_calc', anonymous=False)

    # Code that will listen to sensors
    # Left Sensor is in A0, Right Sensor is in A1, Front Sensor is in A2
    subscribe_sensor_data = rospy.Subscriber('/sensors_data_processed', ME439SensorsProcessed, data_recieved)

    # Spin to prevent function from exiting
    rospy.spin()

def data_recieved(msg_in):
    callback_proc_left(msg_in.a0)
    callback_proc_right(msg_in.a1)
    callback_proc_front(msg_in.a2)

# Function to process sensor data from left sensor
def callback_proc_left(msg_in):
    # Reading analog value from msg
    analog_level = float(msg_in)
    # Converting analog signal into physical distance [m]
    analog_volts = (analog_level - analog_error_left) * (3.3/1024.)
    distance_inch = analog_volts / (3.3/512.)
    distance_meter = distance_inch * 0.0254

    # Message to hold outgoing data
    A0_proc = Float32()
    A0_proc.data = distance_meter
    publish_dist_left.publish(analog_level)
    publish_dist_left_graph.publish(A0_proc)

def callback_proc_right(msg_in):
    # Reading analog value from msg
    analog_level = float(msg_in)
    # Converting analog signal into physical distance [m]
    analog_volts = (analog_level - analog_error_right) * (3.3/1024.)
    distance_inch = analog_volts / (3.3/512.)
    distance_meter = distance_inch * 0.0254

    # Message to hold outgoing data
    A1_proc = Float32()
    A1_proc.data = distance_meter
    publish_dist_right.publish(analog_level)
    publish_dist_right_graph.publish(A1_proc)

def callback_proc_front(msg_in):
    # Reading analog value from msg
    analog_level = float(msg_in)
    # Converting analog signal into physical distance [m]
    analog_volts = (analog_level - analog_error_front) * (3.3/1024.)
    distance_inch = analog_volts / (3.3/512.)
    distance_meter = distance_inch * 0.0254

    # Message to hold outgoing data
    A2_proc = Float32()
    A2_proc.data = distance_meter
    publish_dist_front.publish(analog_level)
    publish_dist_front_graph.publish(A2_proc)


# Error handling code to run sensors processor
if __name__ == '__main__':
    try: 
        sensors_listener()
    except: 
        traceback.print_exc()
        pass
