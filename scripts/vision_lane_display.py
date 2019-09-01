#!/usr/bin/env python
import os, sys
import math, numpy as np
import rospy, cv2

import pdb

import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.trr.utils as trr_u
import two_d_guidance.trr.vision.utils as trr_vu
#import smocap.rospy_utils

class BirdEyePublisher(trr_rpu.ContourPublisher):
    def __init__(self, frame_id, be_param, topic='trr_vision/lane/bird_eye'):
        trr_rpu.ContourPublisher.__init__(self, frame_id, topic, be_param.va_bf, rgba=(1.,0.,0.,1.))

#
# Display Lane Node
#
class Node(trr_rpu.PeriodicNode):
    def __init__(self):
        trr_rpu.PeriodicNode.__init__(self, 'vision_lane_display_node')
        robot_name = rospy.get_param('~robot_name', '')
        def prefix(robot_name, what): return what if robot_name == '' else '{}/{}'.format(robot_name, what)
        cam_name = rospy.get_param('~camera', prefix(robot_name, 'camera_road_front'))
        # we publish markers for rviz
        self.ref_frame = 'caroline/base_link_footprint'
        self.lane_model_marker_pub = trr_rpu.LaneModelMarkerPublisher('/trr/vision/lane/lane_markers', ref_frame=self.ref_frame)
        self.lane_model_sub = trr_rpu.LaneModelSubscriber('/trr_vision/lane/detected_model')
        self.lane = trr_u.LaneModel()

        be_param = trr_vu.NamedBirdEyeParam('caroline')
        self.be_pub = BirdEyePublisher(self.ref_frame, be_param, '/trr/vision/lane/bird_eye')
        #self.fov_pub = smocap.rospy_utils.FOVPublisher(self.cam_sys, self.ref_frame, '/trr_vision/lane/fov')

        
    def periodic(self):
        try:
            self.lane_model_sub.get(self.lane)
            self.lane_model_marker_pub.publish(self.lane)
        except trr_rpu.NoRXMsgException:
            rospy.loginfo_throttle(1., 'trr_vision_lane_display: NoRXMsgException')
        except trr_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., 'trr_vision_lane_display: RXMsgTimeoutException')
        self.be_pub.publish()

    
def main(args): Node().run(10)

if __name__ == '__main__':
    main(sys.argv)
