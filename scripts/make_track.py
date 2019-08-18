#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os, logging, numpy as np, matplotlib.pyplot as plt
import rospkg
import two_d_guidance as tdg
import two_d_guidance.trr.utils as trr_u
import pdb

def make_z_room_path(res=0.01):
   #x0, x1 = -0.75, 2.25
   x0, x1 = 0., 2.25  # moved start of path to center
   c1, r = [x1,0], 1.25
   line1 = tdg.make_line_path([x0, r], [x1, r], res=res)
   circle1 = tdg.make_circle_path(c1, -r, np.pi/2, np.pi, res=res)
   x2, y2 = 1.581, -0.35
   line2 = tdg.make_line_path([x1, -r], [x2, -r], res=0.01)
   c2, r2, th2 = [x2, y2], 0.9, np.deg2rad(61.8)
   circle2 = tdg.make_circle_path(c2, r2, -np.pi/2, -th2, res=res)
   c3, r3 = [0, -1.21], 0.9
   circle3 = tdg.make_circle_path(c3, r3, np.pi/2-th2, 2*th2, res=res)
   c4, r4 = [-x2, y2], 0.9
   circle4 = tdg.make_circle_path(c4, r4, -(np.pi/2-th2), -th2, res=res)
   line3 = tdg.make_line_path([-x2, -r], [-x1, -r], res=res)
   circle5 = tdg.make_circle_path([-x1, 0], -r, -np.pi/2, np.pi, res=res)
   line4 = tdg.make_line_path([-x1, r], [x0, r], res=res)
   line1.append([circle1, line2, circle2, circle3, circle4, line3, circle5, line4])
   p = line1
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real.npz')
   p.save(fname)
   return fname, p

def filter(vel_sps):
   ref = tdg.utils.SecOrdLinRef(omega=3, xi=0.9)
   out = np.zeros(len(vel_sps))
   dt = 0.01 #WTF...
   phi = 50
   vel_sps2 = np.zeros(len(vel_sps))
   vel_sps2[:-phi] = vel_sps[phi:]
   vel_sps2[-phi:] = vel_sps[-phi] # cst....
   out[0] = vel_sps2[0]; ref.reset(np.array([out[0], 0, 0]))
   for i in range(1,len(vel_sps)):
      out[i] = ref.run(dt, vel_sps2[i])[0]
   return out, vel_sps2

# velocity profile proportional to path curvature
def add_vel_profile(fname, v0=1.75, kc=0.75):
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real.npz')
   path = trr_u.TrrPath(fname)
   for i, c in enumerate(path.curvatures):
      path.vels[i] = v0 - kc*abs(c) # why not ...
   fltrd, delayed_curv = filter(path.vels)
   if 1:
      plt.plot(fltrd)
      plt.plot(delayed_curv, alpha=0.5)
      plt.plot(path.vels)
      plt.show()
   path.vels = fltrd
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real_vel.npz')
   path.save(fname)
   p2 = trr_u.TrrPath(fname)
   return p2
   





if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(linewidth=300, suppress=True)
    fname, p = make_z_room_path()
    p2 = add_vel_profile(fname)
    #tdg.draw_path(plt.gcf(), plt.gca(), p)
    #plt.figure()
    tdg.draw_path_curvature(plt.gcf(), plt.gca(), p)
    plt.figure()
    tdg.draw_path_vel(plt.gcf(), plt.gca(), p2)
    plt.show()

