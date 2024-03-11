#!/usr/bin/env python
"""
Author: Ben Marsh
        b.marsh@aims.gov.au

Description:
    ROS module for detection of Crown of Thorns starfish.

    It is comprised of:

    reefscan_cots_detection_recorder	This is a node to write out cots detections as json to disk

"""
import os

import rospy
import json
from rospy_message_converter import message_converter
# ccip_msgs is a module which contains the messages that are sent when COTS is detected
from reefscan_cots_detector.msg import CotsSequenceWithoutImage, CotsDetection, CotsMaximumScore, CotsConfirmedClass
from reefscan_image_utils import resize_for_preview
from refscan_utils import write_csv_row, read_all_from_csv, get_ten_photos_around, get_destination_folder



# Topic to subscribe to COTS detection information
from sequence_writer_helper import get_filename_to_write, get_json_for_file_write

TOPIC_REEFSCAN_COTS_SEQUENCE_WITHOUT_IMAGES = '/reefscan_cots_sequence_without_images'
# reefscan_cots_write_cots_sequenced_detections.py}
PARAM_DATA_FOLDER = "/reefscan_data_folder"

class ReefscanCotsSequencedDetectionRecorder(object):
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        rospy.loginfo("initing")
        # Subscribe to topic for new images
        self.sub_reefscan_cots_sequence = rospy.Subscriber(TOPIC_REEFSCAN_COTS_SEQUENCE_WITHOUT_IMAGES, CotsSequenceWithoutImage,  self.write_cots_detection_information)
        self.data_folder, self.error_flag, self.error_message = get_destination_folder()

    
    # Function:     write_cots_detection_information(self, data)
    # Description:  Receive cots detection information and write out to the disk
    def write_cots_detection_information(self, cots_sequence):
        rospy.loginfo("Writing")

        json_file = get_filename_to_write(cots_sequence)

        msg_dict = message_converter.convert_ros_message_to_dictionary(cots_sequence)
        msg_json = get_json_for_file_write(msg_dict)

        with open(json_file, "w") as myfile:
            myfile.write(msg_json)



if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_cots_detection_recorder_class', anonymous=True)

    # Construct the ReefscanCotsDetectionRecorder that records COTS detection
    reefscan_cots_sequenced_detection_recorder = ReefscanCotsSequencedDetectionRecorder()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

