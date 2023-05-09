import csv
import errno
import glob
import json
import os
from os.path import expanduser
import getpass

import rospy
from filelock import FileLock

IMAGE_FOLDER_NAME				= 'images'

# this functions reads the file reefscan_id.txt from the home folder
# to obtain the ID of the device
# TODO move the reefscan_id to a configuration file which also includes the lens parameters which are stored in reefscan_image_utils
def get_reefscan_id():
    home = expanduser("~")
    fname = home + "/reefscan_id.txt"
    if os.path.exists(fname):
        with open(fname, "r") as f:
            lines = f.readlines()
            id = " ".join(lines).replace('\n', '')
    else:
        id = "Unknown"

    return id


# write one row to a CSV file
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

# Reads and entire CSV file
# returns a list of dictionaries
def read_all_from_csv (file):
    if os.path.exists(file):
        all_dicts = []
        with open(file, "r") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                all_dicts.append(row)

        return all_dicts
    else:
        return []


# This is used by the mark. Gets the photo passed to the function and the 9 previous photos as well
def get_ten_photos_around(filename):
    directory = os.path.dirname(filename)
    all_files = []
    if filename.endswith(".jpg"):
        length = len(filename)
        sequence = int(filename[length-8:length-4])
        for i in range(10):
            pattern = directory + "/*_" + str(sequence - i).zfill(4) + ".jpg"
            files = glob.glob(pattern)
            for file in files:
                all_files.append(file)
    return all_files

# Function: 	get_destination_folder(event=None)
# Description:	Function for creating destination folder and path tree for saving images
def get_destination_folder():
    # Check if destination folder exists and if not, create /media/USER/<SDCard>/images folder
    # login = os.getlogin()
    # os.getlogin does not work in windows subsystem for linux getpass.getuser() does work
    login = getpass.getuser()
    media_path = os.path.join('/media', login)
    external_drives = os.listdir(media_path)

    error_flag = False
    error_message = "No Errors"

    # Find an External Drive that we can write images to
    drive_to_use = None
    for drive_name in external_drives:
        # Check for a drive that is not the "Linux for Tegra Readme Drive" that is automatically mounted.
        if drive_name not in 'L4T-README':
            drive_to_use = drive_name
    if drive_to_use is not None:
        # If we have found a valid drive, check if the images folder path exists
        images_path = os.path.join('/media', login, drive_to_use, IMAGE_FOLDER_NAME)
        if os.path.exists(images_path):
            # The destination folder exists, set self.sequence_path to this folder
            error_flag = False
            error_message = "No Errors"
            return(images_path, error_flag, error_message)
        else:
            # If the Images folder doesn't exist, create one with path tree
            try:
                os.makedirs(images_path)
            except Exception as error_message:
                # Error creating new folder tree
                print(error_message)
                rospy.logerr("Cannot create folder " + images_path + " " + error_message.message)
                error_flag = True
                error_message = "Error creating folder."
                return("", error_flag, error_message)
            else:
                # Folder was successfully created, set self.sequence_path to this folder
                error_flag = False
                error_message = "No Errors"
                return(images_path, error_flag, error_message)
    else:
        # No drives were found that we could write images to.
        error_flag = True
        error_message = "No drive to write to."
        rospy.logerr(error_message)
        return("", error_flag, error_message)


# if __name__ == '__main__':
#     print(get_ten_photos_around("C:/temp/reefcloud_bcakup/20220225_015704_Seq01/20220225_015710_300_0010.jpg"))
