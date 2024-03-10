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
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from ccip_msgs.msg import Image as CcipImage
from reefscan.msg import Reefscan_status
import cv2
from cv_bridge import CvBridge, CvBridgeError


TOPIC_REEFSCAN_PREVIEW = "/reefscan_preview"
TOPIC_REEFSCAN_STATUS = "/reefscan_status"
TOPIC_GPS_FIX = "/fix"
TOPIC_COTS_DETECTOR = "/ccip_images"
PARAM_DATA_FOLDER = "/reefscan_data_folder"


class ReefscanCotsObtainImages(object):

    def __init__(self):
        self.sub_reefscan_preview_images = rospy.Subscriber(TOPIC_REEFSCAN_PREVIEW, CompressedImage, self.subscriber_reefscan_cots_aquire_image)
        self.sub_gps_fix = rospy.Subscriber(TOPIC_GPS_FIX, NavSatFix, self.subscriber_acquire_gps_fix)
        self.sub_reefscan_status = rospy.Subscriber(TOPIC_REEFSCAN_STATUS, Reefscan_status, self.subscriber_acquire_reefscan_status)
        self.pub_reefscan_cots_detector = rospy.Publisher(TOPIC_COTS_DETECTOR, CcipImage, queue_size=1)
        self.cv_bridge = CvBridge()
        self.gps_fix = None
        self.reefscan_sequence_id = ""

    def subscriber_acquire_gps_fix(self, msg):
        self.gps_fix = msg

    def subscriber_acquire_reefscan_status(self, msg):
        self.reefscan_sequence_id = msg.sequence_name

    def subscriber_reefscan_cots_aquire_image(self, msg):
        if msg.header.frame_id:
            filename = msg.header.frame_id
            # Decompress the image message to an array
            self.cv_image_preview = self.cv_bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            # Construct new message
            new_image = self.cv_bridge.cv2_to_imgmsg(self.cv_image_preview, encoding="bgr8")
            # The full path to the filename is used as frame_id
            # This is important for the sequencer node to load the image for the tablet app
            new_image.header.frame_id = filename
            # Copy timestamp to new message.
            # Timestamps are very important for COTS detection.
            if msg.header.stamp.secs > 0:
                new_image.header.stamp.secs = msg.header.stamp.secs
                new_image.header.stamp.nsecs = msg.header.stamp.nsecs
            else:
                now = rospy.get_rostime()
                new_image.header.stamp.secs = now.secs
                new_image.header.stamp.nsecs = now.nsecs
            rospy.loginfo(type(new_image))
            new_msg = CcipImage()
            new_msg.original_image = new_image
            new_msg.geolocation = self.gps_fix
            new_msg.reefscan_sequence_id = self.reefscan_sequence_id

            self.pub_reefscan_cots_detector.publish(new_msg)





if __name__ == "__main__":
    # Initialise the ROS node: reefscan_acquire_node
    rospy.init_node('reefscan_cots_obtain_images_node', anonymous=True)
    print("reefscan_cots_obtain_images_node")
    # Initialise new class object from reefscan_acquire_class()
    reefscan = ReefscanCotsObtainImages()
    print("ReefScan Class Initialised")
    rospy.loginfo("Images will acquired from the topic %s", TOPIC_REEFSCAN_PREVIEW)
    rospy.loginfo("GPS info will acquired from the topic %s", TOPIC_GPS_FIX)
    rospy.loginfo("Reefscan Status info will acquired from the topic %s", TOPIC_REEFSCAN_STATUS)
    rospy.loginfo("Images will be published to the topic %s for COTS detection" % TOPIC_COTS_DETECTOR)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


