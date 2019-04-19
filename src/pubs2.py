#!/usr/bin/env python
"""Publish data to ROS topic
"""
import argparse
import rospy
import time
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

'''class Pubs(object):
    """Class that control publish data to ROS

    Attributes:
        data (dict): Dict that contains all data available of all topics
        topics (list): list of topics structs
    """
    def __init__(self, camera_info_yaml=None):
        # Dict with all data
        self.data = {}
        # Get data from topic list
        self.topics = []
        # Load yaml file
        self.camera_info_msg = None
        #if camera_info_yaml:
        self.camera_info_msg = yaml_to_CameraInfo('/home/rtmcclai/thesis/Thesis/catkin_ws/src/udpvid/config/camera_info.yaml')



        self.subscribe_topics()

    def get_data(self):
        """Return data dict

        Returns:
            dict: Data from all topics
        """
        return self.data

    def set_data(self, path, value={}, pub=None):
        """Add topic to dict and add data on it

        Args:
            path (string): Topic
            value (dict, optional): Data of topic
            pub (None, optional): rospy.Publisher
        """

        # The first item will be empty
        keys = path.split('/')[1:]
        current_level = self.data
        infopub = rospy.Publisher('BlueRov2/camera/camera_info', CameraInfo, queue_size=1)
        for part in keys:
            # If dict don't have the path, create it !
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]

        # Publish data (if it exist) to ros topic (path)
        # I think that this is what actually publishes the message. but not sure
        if value is not {} and 'pub' in current_level:
            try:
                current_level['pub'].publish(value)
                infopub.publish(self.camera_info_msg)

            except Exception as error:
                print(error)

        # Add publisher to dict
        if pub is not None:
            current_level.update({'pub': pub})
            infopub.publish(self.camera_info_msg)

    def subscribe_topic(self, topic, msg_type, queue=1):
        """Update dict with topic

        Args:
            topic (string): Topic path
            msg_type (ros msg type): Message type
            queue (int, optional): number of messages in queue
        """
        self.set_data(topic, pub=rospy.Publisher(
                topic, msg_type, queue_size=queue))

    def subscribe_topics(self):
        """Create dict to access publisher

        Args:
            init (bool, optional): init node
        """

        # Get item in topics and populate dict with publisher
        for topic, msg_type, queue in self.topics:
            self.subscribe_topic(topic, msg_type, queue)

    def callback(self, data, topic):
        """ROS callback

        Args:
            data (string): Data from ROS topic
            topic (string): ROS topic name
        """
        self.set_data(topic, data)

    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link


if __name__ == '__main__':
    try:
        rospy.init_node('set_mav_data')
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)

    pub = Pubs()
    pub.subscribe_topic('/mavros/rc/override', mavros_msgs.msg.OverrideRCIn)

    def rc():
        pub.set_data('/mavros/rc/override',
                     [1201, 1200, 1200, 1200, 1200, 1200, 1200, 1205])

    while not rospy.is_shutdown():
        rc()
        time.sleep(1)'''


model_base_link = '/base_link'
camera_info_msg = yaml_to_CameraInfo('/home/rtmcclai/thesis/Thesis/catkin_ws/src/udpvid/config/camera_info.yaml')
nopub=False

def getimage(self):

    if not video.frame_available():
        global nopub
        nopub=True
        return

    frame = self.video.frame()
    image_msg = Image()
    self._create_header(image_msg)
    height, width, channels = frame.shape
    image_msg.width = width
    image_msg.height = height
    image_msg.encoding = 'bgr8'
    image_msg.data = frame
    cammsg = self.video_bridge.cv2_to_imgmsg(frame, "bgr8")
    self._create_header(cammsg)
    cammsg.step = int(cammsg.step)
    global nopub
    nopub=False
    return cammsg


infopub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)
campub = rospy.Publisher('image_raw', Image, queue_size=1)

rospy.init_node('ROVcam', anonymous=True)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        if nopub is False:
            try:
                getimage()
                campub.publish(cammsg)
                infopub.publish(camera_info_msg)

            except rospy.ROSInterruptException as error:
                print('pubs error with ROS: ', error)
                exit(1)