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
"""



import rospy
# Random is used to change the time between simulated detections
import random
# ccip_msgs is a module which contains the messages that are sent when COTS is detected
from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections
# sensor images contains IMage which is the message this module subscribes to in order 
# to receive images that capture the presense of COTS 
from sensor_msgs.msg import Image

# Topic to subscribe to for Images
TOPIC_REEFSCAN_PREVIEW = '/reefscan_preview'
# Topic to publish messages indicating COTS was detected
TOPIC_REEFSCAN_COTS_DETECTED = '/reefscan_cots_detected'


class ReefscanCotsDetectorSimulated(object):
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        # Images are counted and when this counter is greater than a 
        # threshold, a COTS detection is sent
        # initialise the counter
        self.counter = 0
        # initialise the image number at which cots detection is simulated
        self.cots_image_number = 0
        # Subscribe to topic for new images
        self.sub_reefscan_preview = rospy.Subscriber(TOPIC_REEFSCAN_PREVIEW, Image,  self.detect_cots_callback)
        # Create a publisher for the COTS detection message
        self.pub_reefscan_cots_detected = rospy.Publisher(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections, queue_size=1) #,  anonymous=True) 
    
    # Function:     detect_cots_callback(self, data)
    # Description:  Callback that ROS invokes when an Image is received.  Triggers the COTS detection simulated event
    def detect_cots_callback(self, data):

        # Create a DetectionResult messages with random data
        new_detection_result = DetectionResult()
        new_detection_result.class_id = 1
        new_detection_result.score = 0.33333

        # Create a Detection with random data that includes the 
        # DetectionResult message
        new_detection = Detection()
        new_detection.detection_id = 123
        new_detection.detection_results = [new_detection_result]
        new_detection.left_x = 123
        new_detection.top_y = 123
        new_detection.width = 123
        new_detection.height = 123

        # Create two SequencedDetection's with random data that includes the 
        # Detection message
        new_sequenced_detection = SequencedDetection()
        new_sequenced_detection.sequence_id = 123
        new_sequenced_detection.sequence_length = 1234567890
        new_sequenced_detection.sequence_length = 7890
        new_sequenced_detection.detection = new_detection

        new_sequenced_detection2 = SequencedDetection()
        new_sequenced_detection2.sequence_id = 255
        new_sequenced_detection2.sequence_length = 987654310
        new_sequenced_detection2.sequence_length = 7890
        new_sequenced_detection2.detection = new_detection


        # Increment the counter for each image 
        self.counter = self.counter + 1
        # When we receive enough images (ie enough time has passed simulate COTS detection
        if self.counter > self.cots_image_number:
            rospy.loginfo("Simulated COTS detection.")

            # Calculate when the next COTS even should occur
            self.cots_image_number = 4 * 60 * random.random()
            self.counter = 0

            # Construct the FrameDetections message that contains SequencedDetections
            thing = FrameDetections()
            thing.results = [new_sequenced_detection, new_sequenced_detection2]


            # Send the FrameDetection message
            self.pub_reefscan_cots_detected.publish(thing)

    

if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_cots_detector_simulated', anonymous=True)

    # Construct the ReefscanCotsDetectorSimulated that simulates COTS detection
    reefscan_cots_detector_simulated = ReefscanCotsDetectorSimulated()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

