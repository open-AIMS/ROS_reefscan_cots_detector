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
from reefscan.msg import Reefscan_status
include_images = True


# Topic to subscribe to 
TOPIC_REEFSCAN_COTS_DETECTED = '/detections'
# Topic to publish messages
TOPIC_REEFSCAN_COTS_SEQUENCE = '/reefscan_cots_sequence'
# Topic to read sequence_names from
TOPIC_REEFSCAN_SEQUENCE_INFO = '/reefscan_status'

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
        self.sub_reefscan_cots_detected = rospy.Subscriber(TOPIC_REEFSCAN_COTS_DETECTED, FrameDetections,  self.restructure_cots_detected)
        self.pub_reefscan_sequence_info = rospy.Subscriber(TOPIC_REEFSCAN_SEQUENCE_INFO, Reefscan_status, self.read_reefscan_status)
        self.pub_reefscan_cots_sequence = rospy.Publisher(TOPIC_REEFSCAN_COTS_SEQUENCE, CotsSequence , queue_size=1) 
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        self.sequences = {}
        self.max_scores = {}
        self.seen = {}
        self.updating = False
        self.sequence_name_for_filename = {}


    def read_reefscan_status(self, msg):
        # rospy.loginfo("inserting a path %s" % msg)

        sequence_path = msg.sequence_path
        filename_string = msg.filename_string
        sequence_name = msg.sequence_name
        full_file_path = sequence_path + "/" + filename_string
        if full_file_path not in self.sequence_name_for_filename:
            rospy.loginfo("inserting a path %s" % full_file_path)
            self.sequence_name_for_filename[full_file_path] = sequence_name

    # Function:     _read_image(self, filename)
    # Description:  Helper function that loads the image from the disk for insertion 
    #               into the message.  The image resized for preview only and is 
    #               compressed to save bandwidth.
    def _read_image(self, filename):
        photo = cv2.imread(filename)
        rospy.loginfo(filename + "`.")
        photo_preview = resize_for_preview(photo)
        photo_compressed_msg = self.cv_bridge.cv2_to_compressed_imgmsg(photo_preview)
        photo_compressed_msg.header.frame_id = filename
        return photo_compressed_msg
    
    # Function:     restructure_cots_detected(self, data)
    # Description:  Callback that ROS invokes when an Image is received.  Triggers the COTS detection simulated event
    def restructure_cots_detected(self, data):
        rospy.loginfo("restructure.")
        if self.updating == True:
            return
        self.updating = True
        #rospy.loginfo(data)
        rospy.loginfo(data.header.frame_id)
        filename = data.header.frame_id
        if filename in self.sequence_name_for_filename:
            sequence_name = self.sequence_name_for_filename[filename]
        else:
            sequence_name = "20230204_060005_Seq01-Tern-RE011"

        rospy.loginfo("restructure. frame id: %s" % filename)

        # Each result is a SequencedDetection
        for result in data.results:
            self.seen[str(result.sequence_id)] = time.time()
            if not str(result.sequence_id) in self.sequences:
                #rospy.loginfo("Not in seuqnces")
                rospy.loginfo("restructure. first time I have seen sequence id : %d" % result.sequence_id)
                new_cots_sequence = {}
                new_cots_sequence["sequence_length"] = result.sequence_length
                new_cots_sequence["sequence_name"] = sequence_name
                new_cots_sequence["size"] = result.size
                new_detection = {}

                rospy.loginfo("restructure. first time I have seen sequence id : %s: adding detection id %d " % (result.sequence_id, result.detection.detection_id))

                if include_images == True:
                    new_detection["Image"] = self._read_image(filename)
                else:
                    new_detection["Image"] = ""

                new_detection["detection_id"] = result.detection.detection_id
                new_detection["left"] = result.detection.left_x
                new_detection["top"] = result.detection.top_y
                new_detection["width"] = result.detection.width
                new_detection["height"] = result.detection.height
                new_detection["scores"] = []
                self.max_scores[str(result.sequence_id)] = {}
                #rospy.loginfo(self.sequences)


                for detection_result in result.detection.detection_results:
                    #rospy.loginfo(detection_result)
                    new_detection["scores"].append(
                        {
                                "class_id": detection_result.class_id, 
                                "score": detection_result.score
                        }
                    )
                    # This is the first time we have seen this sequence so the scores will be the max
                    self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score

                new_cots_sequence["detection"] = [ new_detection ] 
                self.sequences[str(result.sequence_id)] = new_cots_sequence

            else:
                rospy.logdebug("Already in seen sequences")
                rospy.loginfo("restructure. already seen sequence id : %d" % result.sequence_id)
                editing_cots_sequence = self.sequences[str(result.sequence_id)]
                
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
                    rospy.logdebug(str(self.max_scores))
                    
                    if not str(detection_result.class_id) in self.max_scores[str(result.sequence_id)]:
                        self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score
                    # If we have recorded a maximum for the class id this score is only the new 
                    # maximum if it is larger than the previously recorded score
                    elif self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] < detection_result.score:
                        self.max_scores[str(result.sequence_id)][str(detection_result.class_id)] = detection_result.score

                rospy.logdebug(str(result.sequence_id))
                rospy.logdebug(str(result.detection.detection_id))
                rospy.loginfo("restructure. already seen sequence id : %d overwriting detection with id %d" % (result.sequence_id, result.detection.detection_id))
                new_detection = {
                    "detection_id": result.detection.detection_id,
                    "left": result.detection.left_x,
                    "top": result.detection.top_y,
                    "width": result.detection.width,
                    "height": result.detection.height,
                    "scores": scores
                }

                if include_images == True:
                    new_detection["Image"] = self._read_image(filename)
                else:
                    new_detection["Image"] = ""

                editing_cots_sequence["detection"].append(new_detection)
                #print(self.sequences)

        self.updating = False

                
    # Function:     restructure_cots_detected(self, data)
    # Description:  Callback that ROS invokes when an timer event occurs. Publishes sequences to '/reefscan_cots_sequence'
    def publish_status(self, data):
        delete = {}
        # Go through all the sequences and figure out
        rospy.loginfo("PUBLISH")
        if self.updating == True:
            return

        self.updating = True

        for sequence_id in self.seen:
            if self.seen[sequence_id] < time.time() - 1:
                rospy.loginfo(sequence_id)
                sequence = self.sequences[sequence_id]
                cots_sequence = CotsSequence()
                cots_sequence.sequence_id = int(sequence_id)
                cots_sequence.sequence_length = sequence['sequence_length']
                cots_sequence.sequence_name = sequence['sequence_name']
                cots_sequence.size = sequence['size']
                cots_sequence.detection = []
                for detection in sequence['detection']:
                    cots_detection = CotsDetection()
                    cots_detection.detection_id = detection['detection_id']
                    if "Image" in detection:
                        cots_detection.image = detection["Image"]
                    else:
                        rospy.loginfo("image missing from cots_detection during publish. sequence id %d" % sequence_id)
                        rospy.loginfo(str(cots_detection))
                        
                    if "detection_id" in detection:
                        cots_detection.detection_id = detection["detection_id"]
                    else:
                        rospy.loginfo("detection_id missing from cots_detection during publish")
                        rospy.loginfo(str(cots_detection))
                    cots_detection.left = detection["left"]
                    cots_detection.top = detection["top"]
                    cots_detection.width = detection["width"]
                    cots_detection.height = detection["height"]
                    cots_detection.detection_results = []
                    rospy.loginfo(cots_detection)

                    for score in detection['scores']:
                        d = DetectionResult()
                        d.class_id = score['class_id']
                        d.score = score['score']
                        cots_detection.detection_results.append(d)
                    cots_sequence.detection.append(cots_detection)

                # Calculate the maximum score for each class that is detected and insert into message
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
                #rospy.loginfo(cots_sequence)
                if include_images:
                    self.pub_reefscan_cots_sequence.publish(cots_sequence)
                else:
                    rospy.loginfo(cots_sequence)


                # Record the sequences that we intend to send so they can be removed from our list of sequences we have seen
                delete[sequence_id] = True

        # Remove sequences that we just sent out.
        for sequence_id in delete:
            rospy.loginfo("Deleting" + sequence_id)
            del self.seen[sequence_id]
            del self.sequences[sequence_id]

        self.updating = False

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


