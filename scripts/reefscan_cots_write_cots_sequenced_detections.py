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
# reefscan_cots_write_cots_sequenced_detections.py}
PARAM_DATA_FOLDER = "/reefscan_data_folder"

class ReefscanCotsSequencedDetectionRecorder(object):
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
        cots_sequence_id = cots_sequence.sequence_id
        reefscan_sequence_name = cots_sequence.sequence_name
        msg_dict = message_converter.convert_ros_message_to_dictionary(cots_sequence)
        for detection in msg_dict['detection']:
            rospy.loginfo("Image is overwritten")
            detection['image']['data'] = None
            
        # writes the sequence metadata to file returns the sequence name and the friendly name
        data_folder = rospy.get_param(PARAM_DATA_FOLDER)
        if data_folder:
            msg_json = json.dumps(msg_dict)
            json_file = "%s/%s/cots_sequence_detection_%06d.json" % (data_folder, sequence_name, sequence_id)

            with open(json_file, "w") as myfile:
                myfile.write(msg_json)
        else:
            rospy.loginfo("Not writing cots sequence because reefscan data folder location is unknown")


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

