#!/usr/bin/env python
import os, sys
import math, numpy as np
import rospy, cv2

import pdb

import two_d_guidance.trr.rospy_utils as trr_rpu

#
# Display Race Manager Status overlayed on horizon camera image
#

class ImgPublisher(trr_rpu.DebugImgPublisher):
    def __init__(self, img_topic):
        trr_rpu.DebugImgPublisher.__init__(self, img_topic, '/trr/race_manager/image_debug')

    def _draw(self, img_bgr, model, data):
        y0=20; font_color=(128,0,255)
        f, h1, h2, c, w = cv2.FONT_HERSHEY_SIMPLEX, 1.25, 0.9, font_color, 2
        cv2.putText(img_bgr, 'Race', (y0, 40), f, h1, c, w)
        try:
            _m, _cl, _tl, _times = model.get()
            _s, _si, _v, _ds, _df = data.get()
            real_lap = min(_cl, _tl)
            cv2.putText(img_bgr, 'mode: {}'.format(trr_rpu.RaceManagerStatusStr(_m)), (y0, 90), f, h2, c, w)
            cv2.putText(img_bgr, 'v: {:.1f}m/s'.format(_v), (y0, 140), f, h2, c, w)
            cv2.putText(img_bgr, 'lap: {}/{}'.format(real_lap, _tl), (y0, 190), f, h2, c, w)
            for i,_t in enumerate(_times[:real_lap+1]):
                cv2.putText(img_bgr, ' {}: {:.2f}s'.format(i, _t), (y0, 240+45*i), f, h2, c, w)
                
        except trr_rpu.NoRXMsgException: pass
        except trr_rpu.RXMsgTimeoutException: pass

class Node(trr_rpu.PeriodicNode):
    def __init__(self):
        trr_rpu.PeriodicNode.__init__(self, 'race_manager_display_node')
        robot_name = rospy.get_param('~robot_name', '')
        def prefix(robot_name, what): return what if robot_name == '' else '{}/{}'.format(robot_name, what)
        cam_name = rospy.get_param('~camera', prefix(robot_name, 'camera_road_front'))
        self.im_pub = ImgPublisher(cam_name)
        self.race_sta_sub = trr_rpu.RaceManagerStatusSubscriber(what='race manager display node')
        self.state_est_sub = trr_rpu.TrrStateEstimationSubscriber(what='race_manager')

    def periodic(self):
        self.im_pub.publish(self.race_sta_sub, self.state_est_sub)
    
def main(args): Node().run(10)

if __name__ == '__main__':
    main(sys.argv)
