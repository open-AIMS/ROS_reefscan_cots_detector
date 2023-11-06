from ros_msgs import Header

# Python classes which simulate the CCIP ROS message types to allow testing independently of ROS
class  DetectionResult(object):
    def __init__(self):
        self.class_id = None
        self.score = None

    def from_dict(self, dict):
        self.class_id = dict["class_id"]
        self.score = dict["score"]

    def as_dict(self):
        return vars(self)

class  Detection(object):
    def __init__(self):
        self.detection_id = None
        self.detection_results = []
        self.left_x = None
        self.top_y = None
        self.width = None
        self.height = None

    def from_dict(self, dict):
        self.detection_id = dict["detection_id"]
        self.left_x = dict["left_x"]
        self.top_y = dict["top_y"]
        self.width = dict["width"]
        self.height = dict["height"]
        self.detection_results = []
        detection_results_dicts = dict["detection_results"]
        for detection_results_dict in detection_results_dicts:
            detection_result = DetectionResult()
            detection_result.from_dict(detection_results_dict)
            self.detection_results.append(detection_result)


class  SequencedDetection(object):
    def __init__(self):
        self.sequence_id = None
        self.sequence_length = None
        self.size = None
        self.detection = None

    def from_dict(self, dict):
        self.sequence_id = dict["sequence_id"]
        self.sequence_length = dict["sequence_length"]
        self.size = dict["size"]
        self.detection = Detection()
        self.detection.from_dict(dict["detection"])


class  FrameDetections(object):
    def __init__(self):
        self.header = None
        self.results = []

    def from_dict(self, dict):
        self.header = Header()
        self.header.from_dict(dict["header"])
        results_dicts = dict["results"]
        self.results = []
        for result_dict in results_dicts:
            result = SequencedDetection()
            result.from_dict(result_dict)
            self.results.append(result)
