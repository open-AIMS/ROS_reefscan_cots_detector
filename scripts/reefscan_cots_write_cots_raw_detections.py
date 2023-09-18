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
from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections, SequenceSummary, DetectionSummary
from reefscan.msg import Reefscan_status
from rospy_message_converter import message_converter

# Topic to subscribe to 
TOPIC_REEFSCAN_COTS_DETECTED = '/detections'
# Topic to read sequence_names from
TOPIC_REEFSCAN_SEQUENCE_INFO = '/reefscan_status'
# Name of the parameter that holds the reefscan data folder location
PARAM_DATA_FOLDER = "/reefscan_data_folder"

class ReefscanCotsWriteDetections:
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        rospy.loginfo("init.")
        self.sub_reefscan_cots_detected = rospy.Subscriber(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections,  self.write_cots_detection_message)
        self.sub_reefscan_sequence_info = rospy.Subscriber(TOPIC_REEFSCAN_SEQUENCE_INFO, Reefscan_status, self.read_reefscan_status)
        self.sequence_name_for_filename = {}

    # Function:     read_reefscan_status(self, msg)
    # Description:  Save a record of Reefscan sequence_name for each image
    def read_reefscan_status(self, msg):
        sequence_path = msg.sequence_path
        filename_string = msg.filename_string
        sequence_name = msg.sequence_name
        full_file_path = sequence_path + "/" + filename_string
        if full_file_path not in self.sequence_name_for_filename:
            # rospy.loginfo("inserting a path %s" % full_file_path)
            self.sequence_name_for_filename[full_file_path] = sequence_name

    # Function:     write_cots_detection_message(self, msg)
    # Description:  Receive detection message and retreive Reefscan sequence_name for the image
    def write_cots_detection_message(self, msg):
        filename = msg.header.frame_id
        if filename in self.sequence_name_for_filename:
            if msg.results:
                sequence_name = self.sequence_name_for_filename[filename]
                self.write_cots_detection(sequence_name, filename, msg)
            del self.sequence_name_for_filename[filename]

    # Function:     write_cots_detection(self, sequence_name, filename, msg)
    # Description:  write out JSON record of detection for particular image
    def write_cots_detection(self, sequence_name, filename, msg):
        sequence_name = self.sequence_name_for_filename[filename]
        image_name = os.path.splitext(os.path.basename(filename))[0]
        rospy.loginfo(image_name)
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        data_folder = rospy.get_param(PARAM_DATA_FOLDER)
        if data_folder:
            msg_json = json.dumps(msg_dict)
            json_file = "%s/%s/cots_image_detection_%s.json" % (data_folder, sequence_name, image_name)

            with open(json_file, "w") as myfile:
                myfile.write(msg_json)
        else:
            rospy.loginfo("Not writing cots detection information because reefscan data folder location is unknown")


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
