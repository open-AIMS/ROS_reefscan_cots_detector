import json
import os

from ccip_msgs.msg import FrameDetections
from reefscan_cots_detector.msg import CotsSequence
from sequence_writer_helper import get_filename_to_write, get_json_for_file_write
from sequencer_helper import SequencerHelper

# This can be used to test the sequencer if you have a folder of detection JSON objects. These would be the files
# output by reefscan_cots_write_cots_raw_detections.py
def read_detection_file(filename):
    with open(filename, "r") as file:
        jsonStr = file.read()
        dict = json.loads(jsonStr)

    frame_detection = FrameDetections()
    frame_detection.from_dict(dict)
    return frame_detection

def _read_image(filename):
    pass

def publish_sequence(sequence: CotsSequence):
    print(get_filename_to_write(sequence))
    print(get_json_for_file_write(sequence.as_dict()))

if __name__ == '__main__':
    helper = SequencerHelper(_read_image=_read_image, publish_sequence=publish_sequence)
    dir = "E:/reefscan/20210726_153637_Seq01/"
    files = os.listdir(dir)
    files = sorted(files)
    for file in files:
        if file.endswith(".json"):
            frame_detection = read_detection_file(f"{dir}/{file}")
            helper.restructure_cots_detected(frame_detection)
            helper.publish_sequences(None)
