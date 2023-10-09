#!/usr/bin/python
from datetime import datetime
import piexif 


import rospy
import os
import os.path
from sensor_msgs.msg import NavSatFix, NavSatStatus,Image, TimeReference

all_image_info = []
import glob

TOPIC_REEFSCAN_TIME_REFERENCE= "/time_reference"
TOPIC_REEFSCAN_CAMERA_RAW = "/camera/image_raw"
TOPIC_REEFSCAN_GPS_FIX = "/fix"

import cv2
from cv_bridge import CvBridge, CvBridgeError


def decimal_degrees_from_dms(decimal_number):
    degrees_value = int(abs(decimal_number))
    minutes_value = int((abs(decimal_number) - degrees_value) * 60)
    seconds_value = round((((abs(decimal_number) - degrees_value)) * 60 - minutes_value) * 60, 5)
    return (convert_to_rational(degrees_value),
            convert_to_rational(minutes_value),
            convert_to_rational(seconds_value))

def dms_to_decimal_degrees(dms):
    degrees_value = convert_from_rational(dms[0]) 
    minutes_value = convert_from_rational(dms[1])
    seconds_value = convert_from_rational(dms[2])
    return degrees_value + (float(minutes_value)/60) + (float(seconds_value)/3600)

# Nested Function that returns a rational number (used for exif parameters)
def convert_to_rational(number):
    if number < 0:
        print("{} is negative. How did that happen??".format(number))
        return 0,0
    rational_fraction = Fraction(str(number))
    return (rational_fraction.numerator, rational_fraction.denominator)

def convert_from_rational(fraction):
    numerator, denominator = fraction
    return float(numerator)/denominator

MAX_PREVIEW_WIDTH               = 1920  # Full HD Screen width = 1920 pixels
MAX_PREVIEW_HEIGHT              = 1080  # Full HD Screen height = 1080 pixels

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

class ImageInfo(object):
    def __init__(self, image_fn, altitude, latitude, longitude, time_secs, time_msecs, pressure_depth, image_id, filename_string, sequence_name, sequence_path):
        self.image_fn = image_fn
        self.altitude = altitude
        self.latitude = latitude
        self.longitude = longitude
        self.time_secs = time_secs
        self.time_msecs = time_msecs
        self.pressure_depth = pressure_depth
        self.image_id = image_id
        self.filename_string = filename_string
        self.sequence_name = sequence_name
        self.sequence_path = sequence_path

class ReefscanFakeStatusClass(object):
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.counter = 0
        self.gps_publisher = rospy.Publisher(TOPIC_REEFSCAN_GPS_FIX, NavSatFix, queue_size=1)
        self.image_publisher = rospy.Publisher(TOPIC_REEFSCAN_CAMERA_RAW, Image, queue_size=1)
        self.time_publisher = rospy.Publisher(TOPIC_REEFSCAN_TIME_REFERENCE, TimeReference, queue_size=1)
        self.altitude=""
        self.latitude=""
        self.longitude=""
        self.time_secs=""
        self.time_msecs=""
        self.pressure_depth=0
        self.image_id=""
        self.filename_string=""
        self.sequence_name=""
        self.sequence_path=""
        self.last_file_written=""

    def navsatfix_message(self):
        message = NavSatFix()
        message.latitude = self.latitude
        message.longitude = self.longitude
        message.altitude = self.altitude
        message.status = NavSatStatus()
        message.status.status = 1
        return message

    def timereference_message(self):
        message = TimeReference()
        message.header.stamp.secs = self.time_secs
        message.header.stamp.nsecs = self.time_msecs * 1000000
        message.header.frame_id = "/gps"
        message.time_ref.secs = self.time_secs
        message.time_ref.nsecs = self.time_msecs * 1000000
        message.source = "gps"
        return message



    def camera_message(self, image_info):
        cv_image = cv2.imread(image_info.image_fn)
        cv_image_preview = resize_for_preview(cv_image)
        msg_preview_image = self.cv_bridge.cv2_to_imgmsg(cv_image_preview, encoding="bgr8")
        msg_preview_image.header.stamp.secs = int(self.time_secs)
        msg_preview_image.header.stamp.nsecs = int(self.time_msecs) * 1000000
        msg_preview_image.header.frame_id = image_info.image_fn
        return msg_preview_image


    def update_some_parts(self, altitude, latitude, longitude, time_secs, time_msecs, pressure_depth, image_id, filename_string, sequence_name, sequence_path):
        self.altitude=altitude
        self.latitude=latitude
        self.longitude=longitude
        self.time_secs=time_secs
        self.time_msecs=time_msecs
        self.pressure_depth=pressure_depth
        self.image_id=image_id
        self.filename_string=filename_string
        self.sequence_name=sequence_name
        self.sequence_path=sequence_path
        self.last_file_written=filename_string


    def publish_status(self, event=None):
        image_info = all_image_info[self.counter]
        self.counter = self.counter + 1
        #if self.counter == len(all_image_info):
        #    self.counter = 0
        if self.counter < len(all_image_info):
            print(image_info.filename_string)
            self.update_some_parts(image_info.altitude, image_info.latitude, image_info.longitude, image_info.time_secs, image_info.time_msecs, image_info.pressure_depth, image_info.image_id, image_info.filename_string, image_info.sequence_name, image_info.sequence_path)
            self.gps_publisher.publish(self.navsatfix_message())
            self.time_publisher.publish(self.timereference_message())
            self.image_publisher.publish(self.camera_message(image_info))


def load_images():
    DATASET_DIR = '/media/jetson/data/20230204_060005_Seq01-Tern-RE011/'
    dataset_directory = rospy.get_param('~dataset_directory', DATASET_DIR)
    print('Reading JPG files from {%s}'% dataset_directory)
    print('%s/49??.jpg' % dataset_directory)
    #image_fn_list = sorted(glob.glob('%s/*_569[3-7].jpg' % dataset_directory))
    #image_fn_list = sorted(glob.glob('/media/jetson/data/20230204_060005_Seq01-Tern-RE011/20230204_*_000_571?.jpg'))
    image_fn_list = []
    for idx in range(4635, 4915):
        matched_files = sorted(glob.glob("%s/*_%04d.jpg" % (dataset_directory, idx)))
        for matched_file in matched_files:
            image_fn_list.append(matched_file)

    bridge = CvBridge()

    old_time = 0
    fake_msecs = 0
    for image_fn in image_fn_list:

        sequence_id = os.path.basename(os.path.dirname(image_fn))

        exif_dict = piexif.load(image_fn)
        actual_filename = os.path.basename(image_fn)
        without_extension = actual_filename.split(".")[0]
        filename_parts = without_extension.split("_")
        datestamp = filename_parts[0]
        year = int(datestamp[0:4])
        month = int(datestamp[4:6])
        day = int(datestamp[6:8])
        timestamp = filename_parts[1]
        hours = int(timestamp[0:2])
        minutes = int(timestamp[2:4])
        seconds = int(timestamp[4:6])
        new_datetime = datetime(year, month, day, hours, minutes, seconds)
        epoch_time = (new_datetime - datetime(1970, 1, 1)).total_seconds()

        total_depth = convert_from_rational(exif_dict["GPS"][piexif.GPSIFD.GPSAltitude])
        altitude = float(total_depth)
        ping_depth = convert_from_rational(exif_dict["Exif"][piexif.ExifIFD.SubjectDistance])
        pressure_depth = total_depth - ping_depth


        gps_datestamp = exif_dict["GPS"][piexif.GPSIFD.GPSDateStamp]

        latitude = dms_to_decimal_degrees(exif_dict["GPS"][piexif.GPSIFD.GPSLatitude])
        longitude = dms_to_decimal_degrees(exif_dict["GPS"][piexif.GPSIFD.GPSLongitude])
        if exif_dict["GPS"][piexif.GPSIFD.GPSLatitudeRef] == "S":
            latitude = - latitude

        if exif_dict["GPS"][piexif.GPSIFD.GPSLongitudeRef] == "W":
            longitude = -longitude

        sequence_name = os.path.basename(os.path.dirname(image_fn))
        sequence_path = os.path.dirname(image_fn)
        filename_string = os.path.basename(image_fn)
        time_secs = int(epoch_time)
        if old_time == time_secs:
            fake_msecs = fake_msecs + 250
        else:
            fake_msecs = 0
        if fake_msecs >= 1000:
            fake_msecs = 0
        old_time = time_secs
        time_msecs = int(filename_parts[-2])
        image_id = int(filename_parts[-1])
        image_info = ImageInfo( image_fn, altitude, latitude, longitude, time_secs, fake_msecs, pressure_depth, image_id, filename_string, sequence_name, sequence_path)
        all_image_info.append(image_info)

        

if __name__ == "__main__":


    # Initialise the ROS node: reefscan_acquire_node
    rospy.init_node('reefscan_cots_detector_fake_stuff ', anonymous=True)
    print("reefscan_cots_detector fake_stuff Initialised")
    # asdkljas;ldjk asdl;askd jlkasj

    load_images()

    # Initialise new class object from reefscan_acquire_class()
    reefscan = ReefscanFakeStatusClass()
    print("ReefScan Class Initialised")
    # Start Reefscan Publisher Timer - with callback function (Set for 5 Hz - 2.0/10.0 )
    rospy.Timer(rospy.Duration(2.0 / 100.0), reefscan.publish_status)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
