#!/usr/bin/env python
"""Publish data to ROS topic
"""
import rospy
import yaml

from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from video import Video

from cv_bridge import CvBridge

def yaml_to_CameraInfo(yaml_fname):
    """
    From https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data
    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg


'''def _create_header(msg): # this can go
    """ Create ROS message header

    Args:
        msg (ROS message): ROS message with header
    """
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = model_base_link'''


model_base_link = '/base_link' # no clue what this is for but its part of the header?

# sets the camera info file handle, should be able to change this as an import parameter from the launch
camera_info_msg = yaml_to_CameraInfo('/home/rtmcclai/thesis/Thesis/catkin_ws/src/udpvid/config/camera_info.yaml')
video_bridge = CvBridge() # I dont understand why I had to do this, but it alleviates the issue with not providing an argument
video = Video() # same as above

# cammsg = {} # I think I can get rid of this also


def getimage():  # gets the image from CV
    if not video.frame_available():
        return
    frame = video.frame()
    image_msg = Image()
    #_create_header(image_msg)
    height, width, channels = frame.shape
    image_msg.width = width
    image_msg.height = height
    image_msg.encoding = 'bgr8'
    image_msg.data = frame
    cmsg = video_bridge.cv2_to_imgmsg(frame, "bgr8")
    cmsg.step = int(cmsg.step)
    return cmsg


# set the topics up
infopub = rospy.Publisher('ROVcam/camera_info', CameraInfo, queue_size=1)
campub = rospy.Publisher('ROVcam/image_raw', Image, queue_size=1)


rospy.init_node('ROVcam')  # initialize the node


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            if video.frame_available():
                cammsg = getimage()  # grabs the image
                msgs = [cammsg, camera_info_msg]  # sets up list for the time setting loop
                timenow = rospy.Time.now()  # grabs the current time and makes it the stamp
                for msg in msgs:
                    msg.header.stamp = timenow  # sets the header time stamp, this is how it synchronizes i think

                # Publish the messages
                campub.publish(cammsg)
                infopub.publish(camera_info_msg)
            else:
                pass

        except rospy.ROSInterruptException as error:
            print('pubs error with ROS: ', error)
            exit(1)
