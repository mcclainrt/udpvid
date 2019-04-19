#!/usr/bin/env python

import rospy
import time
import yaml

from pubs import Pubs
from video import Video

from cv_bridge import CvBridge


from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String


'''def yaml_to_CameraInfo(yaml_fname):
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
    return camera_info_msg'''


class RovMsgs():
    def __init__(self, camera_info_yaml=None):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """

        # super(RovMsgs, self).__init__()

        '''# Load yaml file
        self.camera_info_msg = None
        if camera_info_yaml:
            self.camera_info_msg = yaml_to_CameraInfo(camera_info_yaml)'''

        self.pub = Pubs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        self.video = Video()
        self.video_bridge = CvBridge()

        self.pub_topics = [

            [   self._create_camera_msg,
                '/camera/image_raw',
                Image,
                1
            ],
            [   self.pass_function,
                '/test',
                String,
                1
            ]
            #[
            #    self.pass_function,
            #    '/camera/camera_info',
            #    CameraInfo,
            #    1
            #]
        ]

        self.msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

    def _callback_from_topic(topic):
        """ Create callback function name

        Args:
            topic (str): Topic name

        Returns:
            str: callback name
        """
        return topic.replace('/', '_') + '_callback'

    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
        """ Subscribe to a topic using the publisher

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
        """
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    '''def _create_camera_info_msg(self):
        if self.camera_info_msg:
            self.pub.set_data('/camera/camera_info', self.camera_info_msg)'''

    def _create_camera_msg(self):
        if not self.video.frame_available():
            return
        frame = self.video.frame()
        image_msg = Image()
        self._create_header(image_msg)
        height, width, channels = frame.shape
        image_msg.width = width
        image_msg.height = height
        image_msg.encoding = 'bgr8'
        image_msg.data = frame
        msg = self.video_bridge.cv2_to_imgmsg(frame, "bgr8")
        self._create_header(msg)
        msg.step = int(msg.step)
        self.pub.set_data('/camera/image_raw', msg) #publish camerainfo

    def pass_function(self):
        pass

    def publish(self):
        """ Publish the data in ROS topics
        """
        #self.update()
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.msg_available[topic] = time.time()
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    rovmsg = RovMsgs()

    while not rospy.is_shutdown():
        rovmsg.publish()
