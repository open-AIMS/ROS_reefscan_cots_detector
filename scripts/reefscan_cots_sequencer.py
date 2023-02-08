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
from cv_bridge import CvBridge, CvBridgeError

# Random is used to change the time between simulated detections
import random
# ccip_msgs is a module which contains the messages that are sent when COTS is detected
from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections, SequenceSummary, DetectionSummary
from reefscan_cots_detector.msg import CotsSequence, CotsDetection, CotsMaximumScore
from reefscan_image_utils import resize_for_preview


# Topic to subscribe to 
TOPIC_REEFSCAN_COTS_DETECTED = '/reefscan_cots_detected'
# Topic to publish messages
TOPIC_REEFSCAN_COTS_SEQUENCE = '/reefscan_cots_sequence'


class ReefscanCotsSequencer(object):
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        rospy.loginfo("init.")
        self.cv_bridge = CvBridge()
        self.sub_reefscan_cots_detected = rospy.Subscriber(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections,  self.restructure_cots_detected)
        self.pub_reefscan_cots_sequence = rospy.Publisher(TOPIC_REEFSCAN_COTS_SEQUENCE, CotsSequence , queue_size=1) 
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        self.sequences = {}
        self.max_scores = {}
        self.seen = {}


    def _read_image(self, filename):
        photo = cv2.imread(filename)
        rospy.loginfo(filename + "`.")
        photo_preview = resize_for_preview(photo)
        photo_compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(photo_preview)
        photo_compressed_msg.header.frame_id = filename
        return photo_compressed_msg
    
    # Function:     detect_cots_callback(self, data)
    # Description:  Callback that ROS invokes when an Image is received.  Triggers the COTS detection simulated event
    def restructure_cots_detected(self, data):
        rospy.loginfo("restructure.")
        #rospy.loginfo(data)
        rospy.loginfo(data.header.frame_id)
        filename = data.header.frame_id

        # Each result is a SequencedDetection
        for result in data.results:
            self.seen[str(result.sequence_id)] = time.time()
            if not str(result.sequence_id) in self.sequences:
                #rospy.loginfo("Not in seuqnces")
                self.sequences[str(result.sequence_id)] = {}
                self.sequences[str(result.sequence_id)]["sequence_length"] = result.sequence_length
                self.sequences[str(result.sequence_id)]["size"] = result.size
                self.sequences[str(result.sequence_id)]["detection"] = {}

                if not str(result.detection.detection_id) in self.sequences[str(result.sequence_id)]["detection"]:
                    self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)] = {}

                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["Image"] = self._read_image(filename)
                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["detection_id"] = result.detection.detection_id
                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["left"] = result.detection.left_x
                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["top"] = result.detection.top_y
                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["width"] = result.detection.width
                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["height"] = result.detection.height
                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["scores"] = []
                self.max_scores[str(result.sequence_id)] = {}
                #rospy.loginfo(self.sequences)


                for detection_result in result.detection.detection_results:
                    #rospy.loginfo(detection_result)
                    self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["scores"].append(
                        {
                                "class_id": detection_result.class_id, 
                                "score": detection_result.score
                        }
                    )
                    #rospy.loginfo(self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)]["scores"])
                    # This is the first time we have seen this sequence so the scores will be the max
                    self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score
            else:
                rospy.loginfo("Already in seuqnces")
                # Process the scores first
                scores = []
                for detection_result in result.detection.detection_results:
                    # Record the scores in memory
                    scores.append(
                        {
                            "class_id": detection_result.class_id, 
                            "score": detection_result.score
                        }
                    )
                    # If we have not recorded a maximum for the class id this score is the new maximum
                    if not str(detection_result.class_id) in self.max_scores[str(result.sequence_id)]:
                        self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score
                    # If we have recorded a maximum for the class id this score is only the new 
                    # maximum if it is larger than the previously recorded score
                    elif self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] < detection_result.score:
                        self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score

                self.sequences[str(result.sequence_id)]["detection"][str(result.detection.detection_id)] = {
                    "Image":  self._read_image(filename),
                    "detection_id": result.detection.detection_id,
                    "left": result.detection.left_x,
                    "top": result.detection.top_y,
                    "width": result.detection.width,
                    "height": result.detection.height,
                    "scores": scores
                }
                #rospy.loginfo(self.sequences)
# 

                
    def publish_status(self, data):
        delete = {}
        # Go through all the sequences and figure out
        rospy.loginfo("PUBLISH")
        for sequence_id in self.seen:
            if self.seen[sequence_id] < time.time() - 1:
                rospy.loginfo(sequence_id)
                sequence = self.sequences[sequence_id]
                cots_sequence = CotsSequence()
                cots_sequence.sequence_id = int(sequence_id)
                cots_sequence.sequence_length = sequence['sequence_length']
                cots_sequence.size = sequence['size']
                cots_sequence.detection = []
                for detection_id in sequence['detection']:
                    detection = sequence['detection'][detection_id]
                    cots_detection = CotsDetection()
                    cots_detection.detection_id = int(detection_id)
                    #cots_detection.image = detection["Image"]
                    cots_detection.detection_id = detection["detection_id"]
                    cots_detection.left = detection["left"]
                    cots_detection.top = detection["top"]
                    cots_detection.width = detection["width"]
                    cots_detection.height = detection["height"]
                    cots_detection.detection_results = []

                    for score in detection['scores']:
                        d = DetectionResult()
                        d.class_id = score['class_id']
                        d.score = score['score']
                        cots_detection.detection_results.append(d)
                cots_sequence.detection.append(cots_detection)

                #CotsMaximumScore[] maximum_scores
                maximum_scores = []
                for class_id in self.max_scores[str(sequence_id)]:
                # self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score
                    cots_maximum_score = CotsMaximumScore()
                    cots_maximum_score.class_id = int(class_id)
                    cots_maximum_score.maximum_score = self.max_scores[str(sequence_id)][class_id]
                    maximum_scores.append(cots_maximum_score)
                cots_sequence.maximum_scores = maximum_scores
                    
                #rospy.loginfo(cots_sequence)

                # Send the FrameDetection message
                self.pub_reefscan_cots_sequence.publish(cots_sequence)
                delete[sequence_id] = True
        for sequence_id in delete:
            rospy.loginfo("Deleting" + sequence_id)
            del self.seen[sequence_id]
            del self.sequences[sequence_id]

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


