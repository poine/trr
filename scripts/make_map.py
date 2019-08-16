#!/usr/bin/env python


import sys, os, time, logging, math, numpy as np, yaml
import rospy, rospkg
import PIL.Image, PIL.ImageDraw

import pdb

import two_d_guidance as tdg
import two_d_guidance.trr.state_estimation

def main():
    size_px=(2000, 1000); res=0.005; origin=[-5., -2.5, 0.0]
    _map = tdg.ROSMap(size_px=size_px, resolution=res, origin=origin)

    _map.draw_line_world([-0.1, 0], [0.1, 0], 255, 5)
    _map.draw_line_world([0, -0.1], [0, 0.1], 255, 5)

    tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
    default_path_filename = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real.npz')
    path_filename = rospy.get_param('~path_filename', default_path_filename)
    rospy.loginfo(' loading path: {}'.format(path_filename))
    path = two_d_guidance.trr.state_estimation.StateEstPath(path_filename)
    path.report()

    for p1, p2 in zip(path.points[:-1], path.points[1:]):
        #print p1, p2
        _map.draw_line_world(p1, p2, 255, 5)
    
    trr_dir = rospkg.RosPack().get_path('trr')
    _map.save(os.path.join(trr_dir, 'maps/expe_z'), 'map')
    
    

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
