#!/usr/bin/env python
"""
Author: Ben Marsh
        b.marsh@aims.gov.au

Description:
    ROS module for detection of Crown of Thorns starfish.

    It is comprised of:

    reefscan_cots_detector_simulated        A module to simulate detection by 
                                            receiving and sending dummy 
                                            messages.
    reefscan_cots_sequencer                 A module for restructuring COTS 
                                            detection messages and summarising 
                                            categorisation results.
"""



import os
import os.path
import rospy
import json
# ccip_msgs is a module which contains the messages that are sent when COTS is detected
from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections
from rospy_message_converter import message_converter

# Topic to subscribe to 
TOPIC_REEFSCAN_COTS_DETECTED = '/detections'

PARAM_DATA_FOLDER = "/reefscan_data_folder"

class ReefscanCotsWriteDetections:
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        rospy.loginfo("init.")
        self.sub_reefscan_cots_detected = rospy.Subscriber(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections,  self.write_cots_detection_message)




    # Function:     write_cots_detection_message(self, msg)
    # Description:  Receive detection message and write out JSON record of detection
    def write_cots_detection_message(self, msg):
        filename = msg.header.frame_id
        if filename in self.sequence_name_for_filename:
            sequence_name = self.sequence_name_for_filename[filename]
            if len(msg.results):
                self.write_cots_detection(sequence_name, os.path.splitext(filename)[0], msg)
            else:
                self.write_cots_detection(sequence_name, os.path.splitext(filename)[0] + "_no_detections", msg)

            del self.sequence_name_for_filename[filename]

        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_json = json.dumps(msg_dict)
        json_file = filename + ".json"

        with open(json_file, "w") as myfile:
            myfile.write(msg_json)



if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_cots_write_detections', anonymous=True)

    # Construct the ReefscanCotsDetectorSimulated that simulates COTS detection
    reefscan_cots_write_detections = ReefscanCotsWriteDetections()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
