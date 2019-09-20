#!/usr/bin/env python
import sys, numpy as np
import cv2
import rospy, rosbag, cv_bridge, sensor_msgs.msg, yaml
import tf2_ros, geometry_msgs.msg
import pdb

import two_d_guidance.trr.rospy_utils  as trr_rpu
import two_d_guidance.trr.vision.utils  as trr_vu

def yaml_to_CameraInfo(yaml_fname):
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = sensor_msgs.msg.CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg



import smocap.utils as smu

class Node( trr_rpu.PeriodicNode):
    def __init__(self):
        trr_rpu.PeriodicNode.__init__(self, 'trr_fake_camera_node')
        self.img = cv2.imread('/home/poine/work/robot_data/christine/vedrines_1/frame_000027.png', cv2.IMREAD_COLOR)
        self.img = cv2.imread('/home/poine/work/robot_data/christine/vedrines_1/frame_001475.png', cv2.IMREAD_COLOR)
        self.img = cv2.cvtColor(self.img, cv2.COLOR_RGB2BGR)

        cam_intr_file = '/home/poine/.ros/camera_info/christine_camera_road_front.yaml'
        self.camera_info_msg = yaml_to_CameraInfo(cam_intr_file)
        self.cam_intr_pub = rospy.Publisher("/camera_road_front/camera_info", sensor_msgs.msg.CameraInfo, queue_size=10)


        cam_extr_file = '/home/poine/work/oscar/oscar/oscar_description/cfg/christine_cam_road_front_extr.yaml'
        self.cam = trr_vu.load_cam_from_files(cam_intr_file, cam_extr_file, cam_name='camera_road_front', alpha=1.)

        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.static_transformStamped = geometry_msgs.msg.TransformStamped()
        self.static_transformStamped.header.stamp = rospy.Time.now()
        self.static_transformStamped.header.frame_id = "base_link_footprint"
        self.static_transformStamped.child_frame_id = "camera_road_front_optical_frame"
        smu._position_and_orientation_from_T(self.static_transformStamped.transform.translation, self.static_transformStamped.transform.rotation, self.cam.cam_to_world_T)
        
        self.broadcaster.sendTransform(self.static_transformStamped)
        
        self.img_pub =  trr_rpu.ImgPublisher(cam=self.cam, img_topic='/camera_road_front/image_raw')
        

        
    def periodic(self):
        self.img_pub.publish(self, self.cam)
        self.cam_intr_pub.publish(self.camera_info_msg)

    def draw_debug(self, cam):
        return self.img
        
        
def main(args):
    Node().run(freq=10)

if __name__ == '__main__':
    main(sys.argv)
