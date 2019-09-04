#!/usr/bin/env python
import os, sys
import math, numpy as np
import rospy, cv2

import pdb

import two_d_guidance.trr.vision.utils  as trr_vu

def create_blf_pattern(be_param, draw=True):
    x0, x1, y = 0.4, 1.35, 0.25
    pts_blf = np.array([[x0, -y, 0], [x1, -y, 0], [x1, y, 0], [x0, y, 0]], dtype=np.float32)
    print('blf pattern\n{}'.format(pts_blf))
    if draw:
        #img_bgr = np.zeros((cam.h, cam.w, 3), dtype=np.float32) 
        pass
    
    return pts_blf


#
# Project base link footprint points to camera image
#
def test_blf_to_img(cam, pts_blf, draw=True):
    # project blf points to img (in pixels)
    pts_img = cam.project(pts_blf)
    if draw:
        img_bgr = np.zeros((cam.h, cam.w, 3), dtype=np.float32)
        cv2.polylines(img_bgr, pts_img.astype(np.int32).reshape(1,-1, 2), isClosed=True, color=(0, 0, 255), thickness=2)
        cv2.imshow('image', img_bgr)
        cv2.waitKey(0)
    return pts_img
    
#
# Recovers base link footprint points from image points
#
def test_img_to_blf(cam, pts_img):
    if 0:
        # undistorted points (image plan) in pixels
        pts_imp = cam.undistort_points(pts_img)
        # points in image plan
        pts_cam = [np.dot(cam.inv_undist_K, [u, v, 1]) for (u, v) in pts_imp.squeeze()]
        # points projected on floor plan (in cam frame)
        pts_fp_cam = trr_vu.get_points_on_plane(pts_cam, cam.fp_n, cam.fp_d)
        # points projected on floor plan (in blf frame)
        pts_fp_blf = np.array([np.dot(cam.cam_to_world_T[:3], p.tolist()+[1]) for p in pts_fp_cam])
    else:
        fpi = trr_vu.FloorPlaneInjector()
        pts_fp_blf = fpi.compute(pts_img, cam)
        
    print('recovered blf pattern\n{}'.format(pts_fp_blf))
    
#
# Recovers base link footprint points from image points using homography
#
def test_img_tp_blf_be(cam, pts_img, be_param):
    bet = trr_vu.BirdEyeTransformer(cam, be_param)
    # undistorted points (image plan) in pixels
    pts_imp = cam.undistort_points(pts_img)
    if 0:
        # points in bird eye image (in pixels)
        pts_be = bet.points_imp_to_be(pts_imp)
        # points projected on floor plan (in blf frame)
        pts_blf = bet.unwarped_to_fp(cam, pts_be)
    else:
        pts_blf = bet.points_imp_to_blf(pts_imp)
    print('recovered blf pattern\n{}'.format(pts_blf.squeeze()))
    
def main(args):
    robot_caroline, robot_christine = range(2)
    robot_names = ['caroline', 'christine']
    robot_id = robot_christine
    if robot_id == robot_caroline:
        intr_cam_calib_path = '/home/poine/.ros/camera_info/caroline_camera_road_front.yaml'
        extr_cam_calib_path = '/home/poine/work/roverboard/roverboard_description/cfg/caroline_cam_road_front_extr.yaml'
    elif robot_id == robot_christine:
        intr_cam_calib_path = '/home/poine/.ros/camera_info/christine_camera_road_front.yaml'
        extr_cam_calib_path = '/home/poine/work/oscar/oscar/oscar_description/cfg/christine_cam_road_front_extr.yaml'
    cam = trr_vu.load_cam_from_files(intr_cam_calib_path, extr_cam_calib_path)

    be_param = trr_vu.NamedBirdEyeParam(robot_names[robot_id])
    pts_blf = create_blf_pattern(be_param)
    pts_img = test_blf_to_img(cam, pts_blf, draw=False)
    test_img_to_blf(cam, pts_img)

    test_img_tp_blf_be(cam, pts_img, be_param)
    
    
    
if __name__ == '__main__':
    np.set_printoptions(precision=3, linewidth=300, suppress=True)
    main(sys.argv)
