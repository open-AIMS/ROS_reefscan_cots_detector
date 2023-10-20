from ccip_msgs.msg import DetectionResult

# Python classes which simulate our ROS message types to allow testing independently of ROS

class  CotsDetection(object):
    def __init__(self):
        self.image = None
        self.filename = None
        self.detection_id = None
        self.detection_results = []
        self.left = None
        self.top = None
        self.width = None
        self.height = None

    def as_dict(self):
        dict = vars(self)
        detection_results_dict = []
        result: DetectionResult
        for result in self.detection_results:
            detection_results_dict.append(result.as_dict())
        dict["detection_results"] = detection_results_dict
        return dict

class  CotsSequence(object):
    def __init__(self):
        self.sequence_id = None
        self.sequence_length = None
        self.size = None
        self.detection= []
        self.maximum_scores=[]

    def as_dict(self):
        dict = vars(self)
        max_scores_dicts = []
        max_score: CotsMaximumScore
        for max_score in self.maximum_scores:
            max_scores_dicts.append(max_score.as_dict())
        dict["maximum_scores"] = max_scores_dicts

        detection_dicts = []
        detection: CotsDetection()
        for detection in self.detection:
            detection_dicts.append(detection.as_dict())
        dict["detection"] = detection_dicts

        return dict

class  CotsMaximumScore(object):
    def __init__(self):
        self.class_id = None
        self.maximum_score = None

    def as_dict(self):
        return vars(self)

