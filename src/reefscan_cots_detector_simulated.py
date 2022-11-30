#!/usr/bin/env python

import rospy
import random
from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections
from sensor_msgs.msg import Image

TOPIC_REEFSCAN_PREVIEW = '/reefscan_preview'
TOPIC_REEFSCAN_COTS_DETECTED = '/reefscan_cots_detected'


class ReefscanCotsDetectorSimulated(object):
    def __init__(self):
        # self.cv_bridge = CvBridge()

        #                                            # topic                 #message           #callback
        rospy.logerr("ReefscanCotsDetectorSimulated loaded")
        rospy.logerr("Build 5")
        self.counter = 0
        self.cots_image_number = 0
        self.sub_reefscan_preview = rospy.Subscriber(TOPIC_REEFSCAN_PREVIEW, Image,  self.detect_cots_callback)
        self.pub_reefscan_cots_detected = rospy.Publisher(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections, queue_size=1) #,  anonymous=True) 
    
    def detect_cots_callback(self, data):
        # try:
        #    cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as exception_error:
        #    print(exception_error)

        new_detection_result = DetectionResult()
        new_detection_result.class_id = 1
        new_detection_result.score = 0.33333

        new_detection = Detection()

        #rospy.logerr(str(dir(new_detection)))
        new_detection.detection_id = 123
        new_detection.detection_results = [new_detection_result]
        new_detection.left_x = 123
        new_detection.top_y = 123
        new_detection.width = 123
        new_detection.height = 123

        new_sequenced_detection = SequencedDetection()
        new_sequenced_detection.sequence_id = 123
        new_sequenced_detection.sequence_length = 1234567890
        new_sequenced_detection.sequence_length = 7890
        new_sequenced_detection.detection = [new_detection]



        
        self.counter = self.counter + 1
        if self.counter > self.cots_image_number:
            rospy.logerr("COTS !!**********************")

            self.cots_image_number = 4 * 60 * random.random()
            self.counter = 0
            thing = FrameDetections()
            thing.results = [new_sequenced_detection]


            self.pub_reefscan_cots_detected.publish(thing)
        # class_id = "1"
        # score = 0.3333333
        # self.pub_reefscan_cots_detected.publish(class_id, score)

    

if __name__ == '__main__':
    rospy.init_node('reefscan_cots_detector_simulated', anonymous=True)

    reefscan_cots_detector_simulated = ReefscanCotsDetectorSimulated()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

