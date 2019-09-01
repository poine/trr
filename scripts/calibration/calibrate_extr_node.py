#!/usr/bin/env python
import os, sys
import math, numpy as np
import rospy, cv2

import pdb

import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.trr.vision.utils as trr_vu
import smocap.rospy_utils
#
# Display camera image overlayed on horizon camera image
#

class ImgPublisher(trr_rpu.DebugImgPublisher):
    def __init__(self, img_topic):
        trr_rpu.DebugImgPublisher.__init__(self, img_topic, '/trr/extrinsic_calibration/image_debug')

    def _draw(self, img_bgr, model, data):
        y0=20; font_color=(128,0,255)
        f, h1, h2, c, w = cv2.FONT_HERSHEY_SIMPLEX, 1.25, 0.9, font_color, 2
        cv2.putText(img_bgr, 'Extrinsic Calibration', (y0, 40), f, h1, c, w)

        pts_world = np.array([[0.41, 0., 0.], [0.82, 0., 0.],
                              [0.41, 0., 0.], [0.41, 0.1, 0.], [0.41, 0.2, 0.], [0.41, 0.305, 0.]])
        pts_img = model.cam.project(pts_world)
        for i in range(len(pts_img)-1):
            try:
                cv2.line(img_bgr, tuple(pts_img[i].squeeze().astype(int)), tuple(pts_img[i+1].squeeze().astype(int)), (0,128,0), 4)
            except OverflowError:
                pass
        cp_img_pts = model.cam.project(model.calib_pattern.pts_world).squeeze()
        for cp_img_pt, w_pt in zip(cp_img_pts, model.calib_pattern.pts_world):
            print cp_img_pt, w_pt
            cv2.circle(img_bgr, tuple(cp_img_pt.astype(np.int)), 4, (0,0,255), -1)
            cv2.putText(img_bgr, '{}'.format(w_pt[:2]), tuple(cp_img_pt.astype(np.int)+[0, 25]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)

class CalibPattern:
    def __init__(self):
        self.pts_world = np.array([(0.6, 0, 0), (0.6, -0.3, 0), (0.6, 0.3, 0), (0.4, 0., 0)])

            
class Node(trr_rpu.PeriodicNode):
    def __init__(self):
        trr_rpu.PeriodicNode.__init__(self, 'trr_extrinsic_calibration_node')
        robot_name = rospy.get_param('~robot_name', '')
        def prefix(robot_name, what): return what if robot_name == '' else '{}/{}'.format(robot_name, what)
        cam_name = rospy.get_param('~camera', prefix(robot_name, 'camera_road_front'))
        self.im_pub = ImgPublisher(cam_name)

        if 0:
            intr_path = '/home/poine/.ros/camera_info/christine_camera_road_front.yaml'
            extr_path = '/home/poine/work/oscar/oscar/oscar_description/cfg/christine_cam_road_front_extr.yaml'
            self.cam = trr_vu.load_cam_from_files(intr_path, extr_path, cam_name=cam_name, alpha=1.)
        else:
            cam_names = rospy.get_param('~cameras', prefix(robot_name, 'camera_road_front')).split(',')
            ref_frame = 'base_link_footprint'
            self.cam_sys = smocap.rospy_utils.CamSysRetriever().fetch(cam_names, fetch_extrinsics=True, world=ref_frame)
            self.cam = self.cam_sys.cameras[0]; self.cam.set_undistortion_param(alpha=1.)

        self.calib_pattern = CalibPattern()
            
        
    def periodic(self):
        self.im_pub.publish(self, None)



def main(args): Node().run(10)

if __name__ == '__main__':
    main(sys.argv)
