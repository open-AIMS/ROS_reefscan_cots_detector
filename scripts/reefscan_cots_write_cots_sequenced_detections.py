#!/usr/bin/env python
"""
Author: Ben Marsh
        b.marsh@aims.gov.au

Description:
    ROS module for detection of Crown of Thorns starfish.

    It is comprised of:

    reefscan_cots_detection_recorder	This is a node to write out cots detections as json to disk

"""



import rospy
import json
from rospy_message_converter import message_converter
# ccip_msgs is a module which contains the messages that are sent when COTS is detected
from reefscan_cots_detector.msg import CotsSequence, CotsDetection, CotsMaximumScore, CotsConfirmedClass
from reefscan_image_utils import resize_for_preview
from refscan_utils import write_csv_row, read_all_from_csv, get_ten_photos_around, get_destination_folder



# Topic to subscribe to COTS detection information
TOPIC_REEFSCAN_COTS_SEQUENCE = '/reefscan_cots_sequence'

PARAM_DATA_FOLDER = "/reefscan_data_folder"

class ReefscanCotsDetectionRecorder(object):
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        rospy.loginfo("initing")
        # Subscribe to topic for new images
        self.sub_reefscan_cots_sequence = rospy.Subscriber(TOPIC_REEFSCAN_COTS_SEQUENCE, CotsSequence,  self.write_cots_detection_information)
        self.data_folder, self.error_flag, self.error_message = get_destination_folder()

    
    # Function:     write_cots_detection_information(self, data)
    # Description:  Receive cots detection information and write out to the disk
    def write_cots_detection_information(self, msg):
        rospy.loginfo("Writing")
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        rospy.loginfo(str(msg_dict.keys()))
        sequence_name = msg_dict["sequence_name"]
        self.write_cots_detection(self.data_folder, msg_dict)
        
    # Function:     read_data_folder
    # Description:  read the parameter from confirmation that tells us where the data should be stored
    #               copied from reefscan_marks.py
    def read_data_folder(self):
        if self.data_folder == "":
            self.data_folder = rospy.get_param(PARAM_DATA_FOLDER)

        return self.data_folder


    # Function:     write_json(msg_dict)
    # Description:  write one row to a CSV file
    #               copied from refscan_utils.py
    # 
    # msg_dict is a dictionary which contains the data to be written
    # writes the sequence metadata to file returns the sequence name and the friendly name
    def write_cots_detection(self, data_folder, msg_dict):
        sequence_name = ""
        if "sequence_name" in msg_dict:
            sequence_name = msg_dict["sequence_name"]
            sequence_id = msg_dict["sequence_id"]
            msg_json = json.dumps(msg_dict)
            json_file = data_folder + "/" + sequence_name + "/detection_" + str(sequence_id) + ".json"

            with open(json_file, "w") as myfile:
                myfile.write(msg_json)
        return sequence_name

    

if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_cots_detection_recorder_class', anonymous=True)

    # Construct the ReefscanCotsDetectionRecorder that records COTS detection
    reefscan_cots_detection_recorder = ReefscanCotsDetectionRecorder()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

