#!/usr/bin/env python



# This is a node that takes images from reefscan and posts them to the CSIRO COTS detection model
# This model needs date and time and an image.
#
# Obtain images from /reefscan_preview
# This message type is sensor_msgs/CompressedImage

# This might have date and time in header but doesnt on my test machine
# THere is a frame_id in the header that is only populated when recording because it is the filename.


# The messages are to be published at  rostopic info /test_frames
# This message type is sensor_msgs/Image

import rospy
from ccip_msgs.msg import FrameDetections
import cv2
from cv_bridge import CvBridge, CvBridgeError


TOPIC_DETECTIONS = "/detections"
TOPIC_DETECTIONS_FILTERED = "/detections_filtered"


class ReefscanCotsDetectorBridge(object):

    def __init__(self):
        self.sub_reefscan_cots_detections = rospy.Subscriber(TOPIC_DETECTIONS , FrameDetections, self.subscriber_reefscan_cots_forward_detections)
        self.pub_reefscan_cots_detector = rospy.Publisher(TOPIC_DETECTIONS_FILTERED, FrameDetections, queue_size=1)
        # self.cv_bridge = CvBridge()


    def subscriber_reefscan_cots_forward_detections(self, msg):
        if len(msg.results) > 0:
            self.pub_reefscan_cots_detector.publish(msg)


if __name__ == "__main__":
    # Initialise the ROS node
    rospy.init_node('reefscan_cots_detector_bridge_node', anonymous=True)
    print("reefscan_cots_detector_bridge_node")
    # Initialise new class object from reefscan_acquire_class()
    reefscan = ReefscanCotsDetectorBridge()
    print("ReefscanCotsDetectorBridge initalised")
    rospy.loginfo("Detection information will be acquired from topic %s" % TOPIC_DETECTIONS)
    rospy.loginfo("Detections with cots detected will be sent to topic %s" % TOPIC_DETECTIONS_FILTERED)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


