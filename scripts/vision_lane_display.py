#!/usr/bin/env python
import os, sys
import math, numpy as np
import rospy, cv2

import pdb

import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.trr.utils as trr_u
import two_d_guidance.trr.vision.utils as trr_vu
import smocap

class BirdEyePublisher(trr_rpu.ContourPublisher):
    def __init__(self, frame_id, be_param, topic='trr_vision/lane/bird_eye'):
        trr_rpu.ContourPublisher.__init__(self, frame_id, topic, be_param.va_bf, rgba=(1.,0.,0.,1.))


def make_2d_line(p0, p1, spacing=200, endpoint=True):
    dist = np.linalg.norm(p1-p0)
    n_pt = dist/spacing
    if endpoint: n_pt += 1
    return np.stack([np.linspace(p0[j], p1[j], n_pt, endpoint=endpoint) for j in range(2)], axis=-1)

def get_points_on_plane(rays, plane_n, plane_d):
    return np.array([-plane_d/np.dot(ray, plane_n)*ray for ray in rays])

class FOVPublisher(trr_rpu.ContourPublisher):
    def __init__(self, frame_id, cam, topic='trr_vision/lane/fov', bet=None):
        y0 = 0
        img_corners = np.array([[0., 0+y0], [cam.w, 0+y0], [cam.w, cam.h], [0, cam.h], [0, 0+y0]])
        self.borders_img = np.zeros((0,2))
        for i in range(len(img_corners)-1):
            self.borders_img = np.append(self.borders_img, make_2d_line(img_corners[i], img_corners[i+1], spacing=1, endpoint=True), axis=0)
        if 0:
            #fov = np.array([(0, 0, 0), (1,0, 0), (1, 1, 0), (0, 1, 0)])
            # ideal border of image ( aka undistorted ) in pixels
            borders_undistorted = cv2.undistortPoints(self.borders_img.reshape((1, -1, 2)), cam.K, cam.D, None, cam.K)
            # border of image in optical plan
            borders_cam = [np.dot(cam.invK, [u, v, 1]) for (u, v) in borders_undistorted.squeeze()]
            # border of image projected on floor plane (in cam frame)
            #pdb.set_trace()
            borders_floor_plane_cam = get_points_on_plane(borders_cam, cam.fp_n, cam.fp_d)
            #pdb.set_trace()
            #print max(borders_floor_plane_cam[:,2])
            in_frustum_idx = np.logical_and(borders_floor_plane_cam[:,2]>0, borders_floor_plane_cam[:,2]<10)
            borders_floor_plane_cam = borders_floor_plane_cam[in_frustum_idx,:]
            # border of image projected on floor plane (in world frame)
            borders_floor_plane_world = np.array([np.dot(cam.cam_to_world_T[:3], p.tolist()+[1]) for p in borders_floor_plane_cam])
            #db.set_trace()
            trr_rpu.ContourPublisher.__init__(self, frame_id, topic, borders_floor_plane_world, rgba=(1.,1.,0.,1.))
        else:
            trr_rpu.ContourPublisher.__init__(self, frame_id, topic, bet.borders_floor_plane_world, rgba=(1.,1.,0.,1.))
            #trr_rpu.ContourPublisher.__init__(self, frame_id, topic, bet.borders_isect_be_cam, rgba=(1.,1.,0.,1.))
        
        


        
class ImgPublisher(trr_rpu.DebugImgPublisher):
    def __init__(self, img_topic, cam_name):
        trr_rpu.DebugImgPublisher.__init__(self, cam_name, img_topic)
    def _draw(self, img_bgr, model, data):
        if 1:
            unwarped_img = model.bet.process(img_bgr)
            h, w = img_bgr.shape[:2]
            img_bgr[:,:,:] = trr_vu.change_canvas(unwarped_img, h, w)
        else:
            be_area_img = model.fov_pub.borders_img.reshape((1, -1, 2)).astype(np.int)
            pdb.set_trace()
            be_area_img = model.bet.va_img.reshape((1, -1, 2)).astype(np.int)
            cv2.polylines(img_bgr, be_area_img, isClosed=True, color=(0, 0, 255), thickness=2)

        y0=20; font_color=(128,0,255)
        f, h1, h2, c, w = cv2.FONT_HERSHEY_SIMPLEX, 1.25, 0.9, font_color, 2
        cv2.putText(img_bgr, 'Lane:', (y0, 40), f, h1, c, w)
        
        
   
#
# Display Lane Node
#
class Node(trr_rpu.PeriodicNode):
    def __init__(self):
        trr_rpu.PeriodicNode.__init__(self, 'vision_lane_display_node')
        robot_name = rospy.get_param('~robot_name', '')
        def prefix(robot_name, what): return what if robot_name == '' else '{}/{}'.format(robot_name, what)
        self.cam_names = rospy.get_param('~cameras', prefix(robot_name, 'camera_road_front')).split(',')
        self.ref_frame = rospy.get_param('~ref_frame', prefix(robot_name, 'base_link_footprint'))
        # we publish markers for rviz
        self.lane_model_marker_pub = trr_rpu.LaneModelMarkerPublisher('/trr/vision/lane/lane_markers', ref_frame=self.ref_frame)
        # bird eye
        be_param = trr_vu.NamedBirdEyeParam(robot_name)
        self.be_pub = BirdEyePublisher(self.ref_frame, be_param, '/trr/vision/lane/bird_eye')

        # field of view
        self.cam_sys = smocap.rospy_utils.CamSysRetriever().fetch(self.cam_names, fetch_extrinsics=True, world=self.ref_frame)
        self.cam = self.cam_sys.cameras[0]; self.cam.set_undistortion_param(alpha=1.)
        self.bet = trr_vu.BirdEyeTransformer(self.cam, be_param)
        self.fov_pub = FOVPublisher(self.ref_frame, self.cam, '/trr/vision/lane/fov', self.bet)

        # we publish an image
        self.img_pub = ImgPublisher("/trr/vision/lane/image_debug2", self.cam_names[0])
        
        self.lane_model_sub = trr_rpu.LaneModelSubscriber('/trr_vision/lane/detected_model')
        self.lane = trr_u.LaneModel()

        
    def periodic(self):
        try:
            self.lane_model_sub.get(self.lane)
            self.lane_model_marker_pub.publish(self.lane)
        except trr_rpu.NoRXMsgException:
            rospy.loginfo_throttle(1., 'trr_vision_lane_display: NoRXMsgException')
        except trr_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., 'trr_vision_lane_display: RXMsgTimeoutException')
        self.be_pub.publish()
        self.fov_pub.publish()
        self.img_pub.publish(self, None)
    
def main(args): Node().run(10)

if __name__ == '__main__':
    main(sys.argv)
