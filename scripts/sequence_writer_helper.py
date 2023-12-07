import json
import os

from reefscan_cots_detector.msg import CotsSequence


def get_filename_to_write( cots_sequence):
    cots_sequence_id = cots_sequence.sequence_id
    first_photo = cots_sequence.detection[0].filename
    folder = os.path.dirname(first_photo)
    json_file = "%s/cots_sequence_detection_%06d.json" % (folder, cots_sequence_id)
    return json_file

def get_json_for_file_write(msg_dict):
    for detection in msg_dict['detection']:
        del detection['image']

    msg_json = json.dumps(msg_dict)
    return msg_json
