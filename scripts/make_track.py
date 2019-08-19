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

def filter(vel_sps, omega=3., phi=50):
   ref = tdg.utils.SecOrdLinRef(omega=omega, xi=0.9)
   out = np.zeros((len(vel_sps), 3))
   dt = 0.01 #WTF...
   # time shift
   ts_lim = 800
   vel_sps2 = np.zeros(len(vel_sps))
   vel_sps2[:ts_lim] = vel_sps[phi:ts_lim+phi]
   vel_sps2[ts_lim:] = vel_sps[ts_lim:]
   #vel_sps2[:ts_lim+phi] = vel_sps[phi:ts_lim+phi]
   #vel_sps2[-phi:] = vel_sps[-phi] # cst....
   # run ref
   out[0, 0] = vel_sps2[0]; ref.reset(out[0])
   for i in range(1,len(vel_sps)):
      out[i] = ref.run(dt, vel_sps2[i])
   return out, vel_sps2

# velocity profile proportional to path curvature
def make_curvature_based_vel_profile(fname, v0=1.75, kc=0.75, _plot=False):
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real.npz')
   path = trr_u.TrrPath(fname)
   for i, c in enumerate(path.curvatures):
      path.vels[i] = v0 - kc*abs(c) # why not ...
   fltrd, delayed_curv = filter(path.vels)
   if _plot:
      plt.subplot(3,1,1)
      plt.plot(fltrd[:,0])
      plt.plot(delayed_curv, alpha=0.5)
      plt.plot(path.vels)
      plt.subplot(3,1,2)
      plt.plot(fltrd[:,1])
      plt.subplot(3,1,3)
      plt.plot(fltrd[:,2])
      plt.show()
   path.vels = fltrd[:,0]
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real_vel.npz')
   path.save(fname)
   p2 = trr_u.TrrPath(fname)
   return p2
   

def make_custom_vel_profile(path, _plot=True):
   for i, (c1, c2) in enumerate(zip(path.curvatures[:-1], path.curvatures[1:])):
      if c1 != c2: print i, c1, c2
   print len(path.curvatures)
   vel_sps = np.zeros(len(path.points))
   s_l1 = slice(0, 225)
   s_c1 = slice(225, 617)
   s_l2 = slice(617, 683)
   s_c2c3c4 = slice(683, 1071)
   s_l3 = slice(1071, 1137)
   s_c5 = slice(1137, 1529)
   s_l4 = slice(1529, 1754)
   #v1, v2, v3, v4 = 2.25, 1.8, 1.4, 0.9# works with lookahead time at 0.5
   v1, v2, v3, v4 = 1., 1., 1., 1.
   vel_sps[s_l1] = v1
   vel_sps[s_c1] = v2
   vel_sps[s_l2] = v3
   vel_sps[s_c2c3c4] = v4
   vel_sps[s_l3] = v3
   vel_sps[s_c5] = v2
   vel_sps[s_l4] = v1
   fltrd, delayed_curv = filter(vel_sps, omega=4., phi=100)
   path.vels = fltrd[:,0]
   path.accels = fltrd[:,1]
   path.jerks = fltrd[:,2]
   path.time = np.zeros(len(path.points))
   for i in range(1, len(path.time)):
      d = path.dists[i]-path.dists[i-1]
      path.time[i] = path.time[i-1] + d/path.vels[i-1]
   print('lap time: {:.2f}s'.format(path.time[-1]))
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real_vel_1.npz')
   path.save(fname)
   if _plot:
      plt.title('custom vel profile')
      plt.subplot(3,1,1)
      plt.plot(fltrd[:,0])
      plt.plot(delayed_curv, alpha=0.5)
      plt.plot(vel_sps)
      plt.subplot(3,1,2)
      plt.plot(fltrd[:,1])
      plt.subplot(3,1,3)
      plt.plot(fltrd[:,2])
      plt.figure()
      plt.plot(path.time)
      plt.show()

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(linewidth=300, suppress=True)
    fname, p = make_z_room_path()
    p2 = make_curvature_based_vel_profile(fname)
    make_custom_vel_profile(p2)
    #tdg.draw_path(plt.gcf(), plt.gca(), p)
    #plt.figure()
    tdg.draw_path_curvature(plt.gcf(), plt.gca(), p)
    plt.figure()
    tdg.draw_path_vel(plt.gcf(), plt.gca(), p2)
    plt.show()

