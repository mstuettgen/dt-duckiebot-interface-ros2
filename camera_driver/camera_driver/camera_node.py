#!/usr/bin/env python3

import io
import os
import yaml
import copy
import numpy as np
from threading import Thread
import _thread as thread
from time import sleep

import sys
import rclpy
from rclpy.node import Node

from picamera import PiCamera
from sensor_msgs.msg import CompressedImage, CameraInfo
#from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse
from sensor_msgs.srv import SetCameraInfo

#from duckietown.dtros import DTROS, NodeType, TopicType

#class CameraNode(DTROS):
class CameraNode(Node):
    """Handles the imagery.

    The node handles the image stream, initializing it, publishing frames
    according to the required frequency and stops it at shutdown.
    `Picamera <https://picamera.readthedocs.io/>`_ is used for handling the image stream.

    Note that only one :obj:`PiCamera` object should be used at a time.
    If another node tries to start an instance while this node is running,
    it will likely fail with an `Out of resource` exception.

    The configuration parameters can be changed dynamically while the node is running via
    `rosparam set` commands.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~framerate (:obj:`float`): The camera image acquisition framerate, default is 30.0 fps
        ~res_w (:obj:`int`): The desired width of the acquired image, default is 640px
        ~res_h (:obj:`int`): The desired height of the acquired image, default is 480px
        ~exposure_mode (:obj:`str`): PiCamera exposure mode, one of `these <https://picamera.readthedocs.io/en/latest/api_camera.html?highlight=sport#picamera.PiCamera.exposure_mode>`_, default is `sports`

    Publisher:
        ~image/compressed (:obj:`CompressedImage`): The acquired camera images

    Service:
        ~set_camera_info:
            Saves a provided camera info
            to `/data/config/calibrations/camera_intrinsic/HOSTNAME.yaml`.

            input:
                camera_info (`CameraInfo`): The camera information to save

            outputs:
                success (`bool`): `True` if the call succeeded
                status_message (`str`): Used to give details about success

    """

#    def __init__(self, node_name):
        # # Initialize the DTROS parent class
        # super(CameraNode, self).__init__(
        #     node_name=node_name,
        #     node_type=NodeType.DRIVER,
        #     help="Reads a stream of images from a Pi Camera and publishes the frames over ROS"
        # )

    def __init__(self):
        super().__init__('camera_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.node_name = self.get_name()
        self.log = self.get_logger()
        self._declare_parameters()

        # Add the node parameters to the parameters dictionary and load their default values
        # self._framerate = rospy.get_param(
        #     '~framerate',
        #     dt_help="Framerate at which images frames are produced"
        # )
        self._framerate = self.get_parameter("framerate").get_parameter_value().integer_value
        self.log.info("framerate: " + str(self._framerate))

        # self._res_w = rospy.get_param(
        #     '~res_w',
        #     dt_help="Horizontal resolution (width) of the produced image frames."
        # )
        self._res_w = self.get_parameter("res_w").get_parameter_value().integer_value
        self.log.info("res_w: " + str(self._res_w))

        # self._res_h = rospy.get_param(
        #     '~res_h',
        #     dt_help="Vertical resolution (height) of the produced image frames."
        # )
        self._res_h = self.get_parameter("res_h").get_parameter_value().integer_value
        self.log.info("res_h: " + str(self._res_h))

        # self._exposure_mode = rospy.get_param(
        #     '~exposure_mode',
        #     dt_help="Exposure mode of the camera. Supported values are listed on "
        #             "https://picamera.readthedocs.io/en/release-1.13/api_camera.html#picamera.PiCamera.exposure_mode"
        # )
        self._exposure_mode = self.get_parameter("exposure_mode").get_parameter_value().string_value
        self.log.info("exposure_mode: " + str(self._exposure_mode))

        
        # Setup PiCamera
        self.image_msg = CompressedImage()
        self.camera = PiCamera()
        self.camera.resolution = (self._res_w,self._res_h)
        self.camera.framerate = self._framerate
        self.camera.exposure_mode = self._exposure_mode

        self.log.info("Camera warmup...")
        sleep(2)

        # For intrinsic calibration
        #self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = self.get_namespace().strip('/') + '/camera_optical_frame'
        #self.cali_file = self.cali_file_folder + self.get_namespace().strip("/") + ".yaml"

        # Locate calibration yaml file or use the default otherwise
#        if not os.path.isfile(self.cali_file):
#            self.log.warn("Calibration not found: %s.\n Using default instead." % self.cali_file)
#            self.cali_file = (self.cali_file_folder + "default.yaml")

        # # Shutdown if no calibration file not found
        # if not os.path.isfile(self.cali_file):
        #     self.log.error("Found no calibration file ... aborting")
        #     rclpy.shutdown()

        # # Load the calibration file
        self.current_camera_info=CameraInfo()
        self.current_camera_info.height = self._res_h
        self.current_camera_info.width = self._res_w
        self.current_camera_info.distortion_model = "plumb_bob"
        self.current_camera_info.header.frame_id = "camera_optical_frame"
        self.current_camera_info.d = [-0.2944667743901807, 0.0701431287084318, 0.0005859930422629722, -0.0006697840226199427, 0.0]
        self.current_camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.current_camera_info.p = [220.2460277141687, 0.0, 301.8668918355899, 0.0, 0.0, 238.6758484095299, 227.0880056118307, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.current_camera_info.k = [305.5718893575089, 0.0, 303.0797142544728, 0.0, 308.8338858195428, 231.8845403702499, 0.0, 0.0, 1.0]
        


        
        # self.original_camera_info = self.load_camera_info(self.cali_file)
        # self.original_camera_info.header.frame_id = self.frame_id
        # self.current_camera_info = copy.deepcopy(self.original_camera_info)
        # self.update_camera_params()
        # self.log("Using calibration file: %s" % self.cali_file)

        # Setup publishers
        self.has_published = False
        # self.pub_img = rospy.Publisher(
        #     "~image/compressed",
        #     CompressedImage,
        #     queue_size=1,
        #     dt_topic_type=TopicType.DRIVER,
        #     dt_help="The stream of JPEG compressed images from the camera"
        # )
        self.pub_img = self.create_publisher(
            CompressedImage,
            "image/compressed",
            1
        )
        
        # self.pub_camera_info = rospy.Publisher(
        #     "~camera_info",
        #     CameraInfo,
        #     queue_size=1,
        #     dt_topic_type=TopicType.DRIVER,
        #     dt_help="The stream of camera calibration information, the message content is fixed"
        # )

        self.pub_camera_info = self.create_publisher(
            CameraInfo,
            "camera_info",
            1
            )

        # Setup service (for camera_calibration)
        # self.srv_set_camera_info = rospy.Service(
        #     "~set_camera_info",
        #     SetCameraInfo,
        #     self.srv_set_camera_info_cb
        # )
        self.srv_set_camera_info = self.create_service(
            SetCameraInfo,
            "set_camera_info",
            self.srv_set_camera_info_cb
        )
        
        self.stream = io.BytesIO()
        self.is_shutdown = False
        self.log.info("Initialized.")

         
    def _declare_parameters(self):
        self.declare_parameter('res_w', 640) #tested on Pi4
        self.declare_parameter('res_h', 480) #tested on Pi4
        self.declare_parameter('framerate', 10) #tested on Pi4
        self.declare_parameter('exposure_mode','sports')

        
    def start_capturing(self):
        """Initialize and closes image stream.

            Begin the camera capturing. When the node shutdowns, closes the
            image stream. If it detects StopIteration exception from the `grab_and_publish`
            generator due to parameter change, will update the parameters and
            restart the image capturing.
        """
        self.log.info("Start capturing.")
        while not self.is_shutdown:
            gen = self.grab_and_publish(self.stream)
            try:
                self.camera.capture_sequence(
                    gen,
                    'jpeg',
                    use_video_port=True,
                    splitter_port=0,
                    burst=False,
                )
            except StopIteration:
                pass

        self.camera.close()
        self.log("Capture Ended.")

    def grab_and_publish(self, stream):
        """Captures a frame from stream and publishes it.

            If the stream is stable (no parameter updates or shutdowns),
            grabs a frame, creates the image message and publishes it.
            If there is a paramter change, it does raises StopIteration exception
            which is caught by `start_capturing`.
            It updates the camera parameters and restarts the recording.

            Args:
                stream (:obj:`BytesIO`): imagery stream
        """
        self.log.info("grab_and_publish()")


        while not self.is_shutdown:
            yield stream
            # Construct image_msg
            # Grab image from stream
            stamp = self.get_clock().now().to_msg()
            stream.seek(0)
            stream_data = stream.getvalue()

            # Generate and publish the compressed image
            image_msg = CompressedImage()
            image_msg.format = "jpeg"
            image_msg.data = stream_data
            image_msg.header.stamp = stamp
            image_msg.header.frame_id = self.frame_id
            self.pub_img.publish(image_msg)

#            # Publish the CameraInfo message
            self.current_camera_info.header.stamp = stamp
            self.pub_camera_info.publish(self.current_camera_info)

            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                self.log.info("Published the first image.")
                self.has_published = True

            sleep(0.001)
#            rospy.sleep(rospy.Duration.from_sec(0.001))

    def srv_set_camera_info_cb(self, req):
        self.log("[srv_set_camera_info_cb] Callback!")
        filename = self.cali_file_folder + rospy.get_namespace().strip("/") + ".yaml"
        response = SetCameraInfoResponse()
        response.success = self.save_camera_info(req.camera_info, filename)
        response.status_message = "Write to %s" % filename
        return response

    def save_camera_info(self, camera_info_msg, filename):
        """Saves intrinsic calibration to file.

            Args:
                camera_info_msg (:obj:`CameraInfo`): Camera Info containg calibration
                filename (:obj:`str`): filename where to save calibration
        """
        # Convert camera_info_msg and save to a yaml file
        self.log("[save_camera_info] filename: %s" % filename)

        # Converted from camera_info_manager.py
        calib = {
            'image_width': camera_info_msg.width,
            'image_height': camera_info_msg.height,
            'camera_name': rospy.get_name().strip("/"),  # TODO check this
            'distortion_model': camera_info_msg.distortion_model,
            'distortion_coefficients': {
                'data': camera_info_msg.D,
                'rows': 1,
                'cols': 5
            },
            'camera_matrix': {
                'data': camera_info_msg.K,
                'rows': 3,
                'cols': 3
            },
            'rectification_matrix': {
                'data': camera_info_msg.R,
                'rows': 3,
                'cols': 3
            },
            'projection_matrix': {
                'data': camera_info_msg.P,
                'rows': 3,
                'cols': 4
            }
        }

        self.log("[save_camera_info] calib %s" % calib)

        try:
            f = open(filename, 'w')
            yaml.safe_dump(calib, f)
            return True
        except IOError:
            return False

    def update_camera_params(self):
        """ Update the camera parameters based on the current resolution.

        The camera matrix, rectification matrix, and projection matrix depend on
        the resolution of the image.
        As the calibration has been done at a specific resolution, these matrices need
        to be adjusted if a different resolution is being used.

        TODO: Test that this really works.
        """

        scale_width = float(self._res_w) / self.original_camera_info.width
        scale_height = float(self._res_h) / self.original_camera_info.height

        scale_matrix = np.ones(9)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[4] *= scale_height
        scale_matrix[5] *= scale_height

        # Adjust the camera matrix resolution
        self.current_camera_info.height = self._res_h
        self.current_camera_info.width = self._res_w

        # Adjust the K matrix
        self.current_camera_info.K = np.array(self.original_camera_info.K) * scale_matrix

        # Adjust the P matrix (done by Rohit)
        scale_matrix = np.ones(12)
        scale_matrix[0] *= scale_width
        scale_matrix[2] *= scale_width
        scale_matrix[5] *= scale_height
        scale_matrix[6] *= scale_height
        self.current_camera_info.P = np.array(self.original_camera_info.P) * scale_matrix

    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info



def main(args = None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = CameraNode()
    thread.start_new_thread(node.start_capturing, ())

    try:
#        node.start_capturing()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
