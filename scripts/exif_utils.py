# Module piexif is used for .jpeg geotagging and timestamp
import logging

import piexif
from piexif import helper
import os
# Module datetime used for Unix time to DateTime conversion
from datetime import datetime
from fractions import Fraction


# CONSTANTS
MICROSECONDS_IN_A_SECOND 	= 1000000.0


# Function:  	generate_exif_dictionary(self, altitude, latitude, longitude, time_stamp, exposure_time=1000, gain=1)
# Description: 	Generate the exif dictionary to be used for geotagging
def generate_exif_dictionary(
    latitude,
    longitude, 
    time_stamp, 
    ping_depth, 
    pressure_depth, 
    exposure_time,
    sensor_gain,
    focal_length,
    f_number,
    white_balance, id):
    # Nested Function to output Degrees, Minutes and Seconds from Decimal Degrees
    # Input :	number representing decimal degrees
    # Return:	tuple containing degrees, minutes and seconds


    print("generate exif for latitude: {} longitude {} time_stamp {}, ping_depth {} pressure_depth {}".format(latitude, longitude, time_stamp, ping_depth, pressure_depth))
    def decimal_degrees_to_dms(decimal_number):
        degrees_value = int(abs(decimal_number))
        minutes_value = int((abs(decimal_number) - degrees_value) * 60)
        seconds_value = round((((abs(decimal_number) - degrees_value)) * 60 - minutes_value) * 60, 5)
        return (convert_to_rational(degrees_value),
                convert_to_rational(minutes_value),
                convert_to_rational(seconds_value))

    # Nested Function that returns a rational number (used for exif parameters)
    def convert_to_rational(number):
        if number < 0:
            print("{} is negative. How did that happen??".format(number))
            return 0,0
        rational_fraction = Fraction(str(number))
        return (rational_fraction.numerator, rational_fraction.denominator)

    gpsifd_dict = {
        piexif.GPSIFD.GPSVersionID: (2, 1, 0, 0)
    }
    exif_dict = {
    }
    zeroth = {
    }

    messages = ""
    try:
        ping_depth_m = round(ping_depth / 1000.0, 2)
        pressure_depth_m = round(pressure_depth, 2)
        total_depth = ping_depth_m + pressure_depth_m
        gpsifd_dict [piexif.GPSIFD.GPSAltitude] = convert_to_rational(total_depth),
        exif_dict[piexif.ExifIFD.SubjectDistance]= convert_to_rational(ping_depth_m),
        # it is not really clear what the depth columns mean so an explanation is necessary
        user_comment = "The depth at this location is {total_depth} metres as shown in the field GPSAltitude. The camera was {pressure_depth} metres below the surface. The distance from the camera to the bottom was {ping_depth} metres as shown in the field SubjectDistance."
        user_comment = user_comment.format(total_depth=total_depth, pressure_depth=pressure_depth_m,
                                           ping_depth=ping_depth_m)
        user_comment = unicode(user_comment, 'utf-8')
        user_comment = piexif.helper.UserComment.dump(user_comment)

        exif_dict[piexif.ExifIFD.UserComment] =  user_comment
    except:
        messages = messages + " Cannot find Depth data"

    try:
        # Generate GPS Time String from time_stamp
        gps_time_string = datetime.utcfromtimestamp(time_stamp).strftime('%Y:%m:%d %H:%M:%S')
        gpsifd_dict[piexif.GPSIFD.GPSDateStamp] = gps_time_string
        exif_dict[piexif.ExifIFD.DateTimeDigitized] = gps_time_string
        exif_dict[piexif.ExifIFD.DateTimeOriginal] = gps_time_string
        exif_dict[piexif.ExifIFD.MakerNote] = id
        zeroth[piexif.ImageIFD.CameraSerialNumber] = id
        zeroth[piexif.ImageIFD.DateTime] = gps_time_string
    except:
        messages = messages + " Cannot get time"

    try:
        exif_dict[piexif.ExifIFD.ExposureTime] = convert_to_rational(exposure_time)
    except:
        messages = messages + " Cannot get exposure time"

    try:
        exif_dict[piexif.ExifIFD.ISOSpeedRatings] = int(sensor_gain)
    except:
        messages = messages + " Cannot get gain"

    try:
        exif_dict[piexif.ExifIFD.FocalLength] = convert_to_rational(focal_length)
    except:
        messages = messages + " Cannot get focal length"

    try:
        exif_dict[piexif.ExifIFD.FNumber] = convert_to_rational(f_number)
    except:
        messages = messages + " Cannot get exposure time"

    try:
        exif_dict[piexif.ExifIFD.WhiteBalance] = int(white_balance)
    except:
        messages = messages + " Cannot get exposure time"


     # if latitude is not None and longitude is not None and not isNan(latitude) and not isNan(longitude):
    try:
    # Identify GPS Location Reference, i.e. "N" or "S", "E" or "W" based on sign of lat/lon
        if latitude < 0:
            gpsifd_dict[piexif.GPSIFD.GPSLatitudeRef] = "S"
        else:
            gpsifd_dict[piexif.GPSIFD.GPSLatitudeRef] = "N"
        if longitude < 0:
            gpsifd_dict[piexif.GPSIFD.GPSLongitudeRef] = "W"
        else:
            gpsifd_dict[piexif.GPSIFD.GPSLongitudeRef] = "E"

        gpsifd_dict[piexif.GPSIFD.GPSLatitude] = decimal_degrees_to_dms(latitude)
        gpsifd_dict[piexif.GPSIFD.GPSLongitude] = decimal_degrees_to_dms(longitude)

    except:
        messages = messages + " Cannot find GPS"

    return {"Exif": exif_dict, "GPS": gpsifd_dict, "0th": zeroth}, messages


def write_exif_to_file(generated_exif_dictionary, write_filename):
    # Use piexif to generate a dictionary used for creating exif data
    # print("write exif to file " + write_filename)
    # print(str(generated_exif_dictionary))

    # Generate exif data as bytes from piexif.dum()
    exif_bytes = piexif.dump(generated_exif_dictionary)
    # Insert exif data into .jpeg file already written
    if os.path.isfile(write_filename):
        piexif.insert(exif_bytes, write_filename)
