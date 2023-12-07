class  Header(object):
    def __init__(self):
        self.stamp = None
        self.frame_id = None
        self.seq = None
    def from_dict(self, dict):
        self.frame_id = dict["frame_id"]
        self.seq = dict["seq"]
        self.stamp = Stamp()
        self.stamp.from_dict(dict["stamp"])


class Stamp(object):
    def __init__(self):
        self.secs = None
        self.nsecs = None
    def from_dict(self, dict):
        self.secs = dict["secs"]
        self.nsecs = dict["nsecs"]
