#!/usr/bin/env python
"""
Author: Ben Marsh
        b.marsh@aims.gov.au

Description:
    ROS module for detection of Crown of Thorns starfish.

    It is comprised of:

    reefscan_cots_confirmed_class       A module to record the confirmed 
                                        class for machine learning detections 
"""



import rospy

# Topic to subscribe to confirmations
TOPIC_REEFSCAN_COTS_CONFIRMATIONS = '/reefscan_cots_sequence_confirmed'


class ReefscanCotsConfirmedClass(object):
    # Function:     __init(self)
    # Description:  Initialise counters and create ROS publisher and scubscriber objects
    def __init__(self):
        # Subscribe to topic for new images
        self.sub_reefscan_preview = rospy.Subscriber(TOPIC_REEFSCAN_COTS_CONFIRMATIONS, CotsConfirmedClass,  self.write_cots_class_confirmation)
    
    # Function:     write_cots_class_confirmation(self, data)
    # Description:  Receive cots detection confirmations of detected classes and write out to the disk
    def write_cots_class_confirmation(self, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        sequence_name = msg_dict["sequence_name"]
        folder_sequence = self.read_data_folder() + "/" + sequence_name
        write_csv_row(folder_sequence, "cots_class_confirmations.csv", msg_dict)
        
    # Function:     read_data_folder
    # Description:  read the parameter from confirmation that tells us where the data should be stored
    #               copied from reefscan_marks.py
    def read_data_folder(self):
        if self.data_folder == "":
            self.data_folder = rospy.get_param(PARAM_DATA_FOLDER)

        return self.data_folder

    # Function:     wriet_csv_row
    # Description:  write one row to a CSV file
    #               copied from refscan_utils.py
    # 
    # msg_dict is a dictionary which contains the data to be written
    def write_csv_row(directory, file_name, msg_dict):
        if not os.path.isdir(directory):
            try:
                os.makedirs(directory)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise
        file_name = directory + "/" + file_name
        field_names = sorted(msg_dict.keys())
        lock = FileLock(file_name + '.lock')
        with lock:
            file_exists = os.path.exists(file_name)
            if file_exists:
                mode = 'a'
            else:
                mode = 'w'

            with open(file_name, mode) as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=field_names, lineterminator="\n")
                if not file_exists:
                    writer.writeheader()
                writer.writerow(msg_dict)



    

if __name__ == '__main__':
    # Initialise node with rospy
    rospy.init_node('reefscan_cots_confirmed_class', anonymous=True)

    # Construct the ReefscanCotsDetectorSimulated that simulates COTS detection
    reefscan_cots_confirmed_class = ReefscanCotsConfirmedClass()

    # Go into the spin() loop so rospy can
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


