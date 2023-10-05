import os

import rospy
import cv2
from exif_utils import write_exif_to_file, generate_exif_dictionary

JPEG_COMPRESSION_QUALITY		= 92
# TODO Maybe move these parameters to a configurable parameters file
#LENS PARAMETERS
LENS_FOCAL_LENGTH               = 12
LENS_F_NUMBER                   = 5.6
CAMERA_WHITE_BALANCE            = 1     # For Exif Data:  0 = Auto, 1 = Manual

MAX_PREVIEW_WIDTH               = 1920  # Full HD Screen width = 1920 pixels
MAX_PREVIEW_HEIGHT              = 1080  # Full HD Screen height = 1080 pixels


# Create a resized preview image for publishing this to a topic, "/reefscan_preview"
def resize_for_preview(cv_image):
    percentage_width = float(MAX_PREVIEW_WIDTH) / cv_image.shape[1]
    percentage_height = float(MAX_PREVIEW_HEIGHT) / cv_image.shape[0]
    if percentage_width < percentage_height:
        # Width determines resize factor
        preview_width = int(cv_image.shape[1] * percentage_width)
        preview_height = int(cv_image.shape[0] * percentage_width)
    else:
        # Height determines resize factor
        preview_width = int(cv_image.shape[1] * percentage_height)
        preview_height = int(cv_image.shape[0] * percentage_height)
    # preview_width = int(cv_image.shape[1] * PREVIEW_SCALE_PERCENTAGE / 100)
    # preview_height = int(cv_image.shape[0] * PREVIEW_SCALE_PERCENTAGE / 100)
    preview_dimensions = (preview_width, preview_height)
    cv_image_preview = cv2.resize(cv_image, preview_dimensions, interpolation=cv2.INTER_NEAREST)
    return cv_image_preview


# write the image to file. the path and filename are determined by the data_folder and the state (:reefscan_state)
# returns the full filename of the image written and an error flag and message to capture any errors
def write_image_to_file(cv_image, state, data_folder):
    # initialise output variables
    write_filename = ""
    error_flag = False
    error_message = ""
    
    # If a path has been created by a new sequence, continue to writing the image file
    if state.sequence_path is not None:
        # Check if the destination path exists, or still exists since starting a new sequence
        if data_folder != "":
            # The Images Drive or Destination Folder exists, so re-create the sequence path
            if not os.path.exists(state.sequence_path):
                try:
                    os.makedirs(state.sequence_path)
                except (IOError, FileExistsError) as error_message:
                    # Error creating new folder tree
                    print(error_message)
                    rospy.logerr(error_message)
                    error_flag = True
                    error_message = "Error creating images folder."
        else:
            error_flag = True
            error_message = "Error getting images folder."
            rospy.logerr(error_message)

        if os.path.exists(state.sequence_path):
            # Compose Filename for writing output including the path and sequence name for the folder
            # Create the filename based on a valid timestamp
            # Create Filename including path
            write_filename = os.path.join(state.sequence_path, state.filename_string())

            # Write image to disk...
            # print("Writing...", end='')
            print("Writing...")

            try:
                write_file_result = cv2.imwrite(write_filename, cv_image,
                                                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_COMPRESSION_QUALITY])
            except thisError as error_message:
                # Failed to write image to file
                print("FAILED")
                rospy.logerr(error_message)
                error_flag = True
                error_message = "Failed to write file."
            if write_file_result is False:
                error_flag = True
                error_message = "Failed to write file."
            else:
                print("Complete")
                error_flag = False
                error_message = "No Errors"
            # rospy.loginfo("Image written: {0},{1},{2},{3}".format(write_filename, self.altitude, self.latitude, self.longitude))
            # Encode EXIF data to image
            generated_exif_dictionary, messages = generate_exif_dictionary(state.latitude,
                                                                           state.longitude,
                                                                           state.time_secs,
                                                                           state.ping_depth,
                                                                           state.pressure_depth,
                                                                           state.exposure_time,
                                                                           state.sensor_gain,
                                                                           LENS_FOCAL_LENGTH,
                                                                           LENS_F_NUMBER,
                                                                           CAMERA_WHITE_BALANCE,
                                                                           state.get_device_id()
                                                                           )
            if (messages != ""):
                rospy.logerr(messages)
            write_exif_to_file(generated_exif_dictionary, write_filename)

    return write_filename, error_flag, error_message
