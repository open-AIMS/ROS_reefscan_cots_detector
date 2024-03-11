import time

from ccip_msgs.msg import DetectionResult, Detection, SequencedDetection, FrameDetections
from reefscan_cots_detector.msg import CotsSequence, CotsSequenceWithoutImage, CotsDetection, CotsDetectionWithoutImage, CotsMaximumScore

# Most of the functionality from the COTS sequencer has been extracted to this helper class
# This class has no reference to any ROS specific code so it can be tested independantly of ROS
include_images = True
class SequencerHelper():
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self, _read_image, publish_sequence_with_images, publish_sequence_without_images):
        self._read_image = _read_image
        self.publish_sequence_with_images = publish_sequence_with_images
        self.publish_sequence_without_images = publish_sequence_without_images
        self.sequences = {}
        self.max_scores = {}
        self.seen = {}
        self.updating = False
        self.last_time = 0


    def stamp_to_float(self, stamp):
        return stamp.secs + stamp.nsecs/1000000

    def publish_sequence(sequence, with_images=True):
        if with_images:
            cots_sequence = CotsSequence()
        else:
            cots_sequence = CotsSequenceWithoutImage()
        cots_sequence.sequence_id = int(sequence_id)
        cots_sequence.sequence_length = sequence['sequence_length']
        cots_sequence.size = sequence['size']
        cots_sequence.detection = []
        for detection in sequence['detection']:
            if with_images:
                cots_detection = CotsDetection()
                cots_detection.image = detection["Image"]
            else:
                cots_detection = CotsDetectionWithoutImage()

            cots_detection.detection_id = detection['detection_id']
            cots_detection.filename = detection["filename"]
            cots_detection.left = detection["left"]
            cots_detection.top = detection["top"]
            cots_detection.width = detection["width"]
            cots_detection.height = detection["height"]
            cots_detection.detection_results = []
            # rospy.loginfo(cots_detection)

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

        # rospy.loginfo(cots_sequence)

        # Send the FrameDetection message
        # rospy.loginfo(cots_sequence)
        if with_images:
            self.publish_sequence_with_images(cots_sequence)
        else:
            self.publish_sequence_without_images(cots_sequence)


    def restructure_cots_detected(self, frame_detection):
        # rospy.loginfo("restructure.")
        if self.updating == True:
            return
        self.updating = True
        # rospy.loginfo(data)
        # rospy.loginfo(data.header.frame_id)
        filename = frame_detection.header.frame_id
        time_stamp = self.stamp_to_float(frame_detection.header.stamp)
        self.last_time = time_stamp

        # rospy.loginfo("restructure. frame id: %s" % filename)

        # Each result is a SequencedDetection
        for sequenced_detection in frame_detection.results:
            self.seen[str(sequenced_detection.sequence_id)] = time_stamp
            if not str(sequenced_detection.sequence_id) in self.sequences:
                # rospy.loginfo("Not in seuqnces")
                # rospy.loginfo("restructure. first time I have seen sequence id : %d" % result.sequence_id)
                new_cots_sequence = {}
                new_cots_sequence["sequence_length"] = sequenced_detection.sequence_length
                new_cots_sequence["size"] = sequenced_detection.size
                new_detection = {}
                new_detection["filename"] = filename

                # rospy.loginfo("restructure. first time I have seen sequence id : %s: adding detection id %d " % (result.sequence_id, result.detection.detection_id))

                if include_images == True:
                    new_detection["Image"] = self._read_image(filename)
                else:
                    new_detection["Image"] = ""

                new_detection["detection_id"] = sequenced_detection.detection.detection_id
                new_detection["left"] = sequenced_detection.detection.left_x
                new_detection["top"] = sequenced_detection.detection.top_y
                new_detection["width"] = sequenced_detection.detection.width
                new_detection["height"] = sequenced_detection.detection.height
                new_detection["scores"] = []
                self.max_scores[str(sequenced_detection.sequence_id)] = {}
                # rospy.loginfo(self.sequences)

                for detection_result in sequenced_detection.detection.detection_results:
                    # rospy.loginfo(detection_result)
                    new_detection["scores"].append(
                        {
                            "class_id": detection_result.class_id,
                            "score": detection_result.score
                        }
                    )
                    # This is the first time we have seen this sequence so the scores will be the max
                    self.max_scores[str(sequenced_detection.sequence_id)][
                        str(detection_result.class_id)] = detection_result.score

                new_cots_sequence["detection"] = [new_detection]
                self.sequences[str(sequenced_detection.sequence_id)] = new_cots_sequence

            else:
                # rospy.logdebug("Already in seen sequences")
                # rospy.loginfo("restructure. already seen sequence id : %d" % result.sequence_id)
                editing_cots_sequence = self.sequences[str(sequenced_detection.sequence_id)]
                editing_cots_sequence["sequence_length"] = sequenced_detection.sequence_length
                editing_cots_sequence["size"] = sequenced_detection.size

                # Process the scores first
                scores = []
                for detection_result in sequenced_detection.detection.detection_results:
                    # Record the scores in memory
                    scores.append(
                        {
                            "class_id": detection_result.class_id,
                            "score": detection_result.score
                        }
                    )
                    # If we have not recorded a maximum for the class id this score is the new maximum
                    # rospy.logdebug(str(self.max_scores))

                    if not str(detection_result.class_id) in self.max_scores[str(sequenced_detection.sequence_id)]:
                        self.max_scores[str(sequenced_detection.sequence_id)][
                            str(detection_result.class_id)] = detection_result.score
                    # If we have recorded a maximum for the class id this score is only the new
                    # maximum if it is larger than the previously recorded score
                    elif self.max_scores[str(sequenced_detection.sequence_id)][
                        str(detection_result.class_id)] < detection_result.score:
                        self.max_scores[str(sequenced_detection.sequence_id)][
                            str(detection_result.class_id)] = detection_result.score

                # rospy.logdebug(str(result.sequence_id))
                # rospy.logdebug(str(result.detection.detection_id))
                # rospy.loginfo("restructure. already seen sequence id : %d overwriting detection with id %d" % (result.sequence_id, result.detection.detection_id))
                new_detection = {
                    "filename": filename,
                    "detection_id": sequenced_detection.detection.detection_id,
                    "left": sequenced_detection.detection.left_x,
                    "top": sequenced_detection.detection.top_y,
                    "width": sequenced_detection.detection.width,
                    "height": sequenced_detection.detection.height,
                    "scores": scores
                }

                if include_images == True:
                    new_detection["Image"] = self._read_image(filename)
                else:
                    new_detection["Image"] = ""

                editing_cots_sequence["detection"].append(new_detection)
                # print(self.sequences)

        self.updating = False

    # Function:     restructure_cots_detected(self, data)
    # Description:  Callback that ROS invokes when an timer event occurs. Publishes sequences to '/reefscan_cots_sequence'
    def publish_sequences(self, data):
        delete = {}
        # Go through all the sequences and figure out
        # rospy.loginfo("PUBLISH")
        if self.updating == True:
            return

        self.updating = True
        for sequence_id in self.seen:
            if self.seen[sequence_id] < self.last_time - 1:
                # rospy.loginfo(sequence_id)
                sequence = self.sequences[sequence_id]

                self.publish_sequence(sequence, with_images=True)
                self.publish_sequence(sequence, with_images=False)

                # Record the sequences that we intend to send so they can be removed from our list of sequences we have seen
                delete[sequence_id] = True

        # Remove sequences that we just sent out.
        for sequence_id in delete:
            # rospy.loginfo("Deleting" + sequence_id)
            del self.seen[sequence_id]
            del self.sequences[sequence_id]

        self.updating = False

