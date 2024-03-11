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



import rospy
import time
import cv2
import json
from cv_bridge import CvBridge, CvBridgeError

# Random is used to change the time between simulated detections
import random
# ccip_msgs is a module which contains the messages that are sent when COTS is detected
from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections
from reefscan_cots_detector.msg import CotsSequence, CotsSequenceWithoutImage, CotsDetection, CotsMaximumScore
from reefscan_image_utils import resize_for_preview




# Topic to subscribe to
from sequencer_helper import SequencerHelper

TOPIC_REEFSCAN_COTS_DETECTED = '/detections'
# Topic to publish messages
TOPIC_REEFSCAN_COTS_SEQUENCE = '/reefscan_cots_sequence'

TOPIC_REEFSCAN_COTS_SEQUENCE_WITHOUT_IMAGES = '/reefscan_cots_sequence_without_images'


PARAM_DATA_FOLDER = "/reefscan_data_folder"

def compare_detection(old_detection, new_detection):
    if old_detection["detection_id"] != new_detection['detection_id'] or old_detection["left"] != new_detection['left'] or old_detection["top"] != new_detection['top'] or old_detection["width"] != new_detection['width'] or old_detection["height"] != new_detection['height'] or old_detection["scores"] != new_detection['scores']: 
        return False
    else:
        return True

def detection_is_different(new_detection, detections):
    for detection in detections.values():
        if compare_detection(new_detection, detection) == False:
            return True
    return False

class ReefscanCotsSequencer():
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        rospy.loginfo("init.")
        self.cv_bridge = CvBridge()
        self.helper = SequencerHelper(_read_image=self._read_image, publish_sequence_with_images=self.publish_sequence_with_images, publish_sequence_without_images=self.publish_sequence_without_images )
        self.sub_reefscan_cots_detected = rospy.Subscriber(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections,  self.helper.restructure_cots_detected)
        self.pub_reefscan_cots_sequence_with_images = rospy.Publisher(TOPIC_REEFSCAN_COTS_SEQUENCE, CotsSequence , queue_size=1)
        self.pub_reefscan_cots_sequence_without_images  = rospy.Publisher(TOPIC_REEFSCAN_COTS_SEQUENCE_WITHOUT_IMAGES, CotsSequenceWithoutImage, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.helper.publish_sequences)



    # Function:     _read_image(self, filename)
    # Description:  Helper function that loads the image from the disk for insertion 
    #               into the message.  The image resized for preview only and is 
    #               compressed to save bandwidth.
    def _read_image(self, filename):
        photo = cv2.imread(filename)
        # rospy.loginfo(filename + "`.")
        photo_preview = resize_for_preview(photo)
        photo_compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(photo_preview)
        photo_compressed_msg.header.frame_id = filename
        return photo_compressed_msg

    def publish_sequence_with_images(self, cots_sequence):
        self.pub_reefscan_cots_sequence_with_images.publish(cots_sequence)
    
    def publish_sequence_without_images(self, cots_sequence):
        self.pub_reefscan_cots_sequence_without_images.publish(cots_sequence)


if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_cots_sequencer', anonymous=True)

    # Construct the ReefscanCotsDetectorSimulated that simulates COTS detection
    reefscan_cots_sequencer = ReefscanCotsSequencer()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


