#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys, os, logging, numpy as np, matplotlib.pyplot as plt
import rospkg
import two_d_guidance as tdg
import two_d_guidance.trr.utils as trr_u
import pdb


#
#  Horizontal shapes
#

def make_z_room_path(res=0.01):
   #x0, x1 = -0.75, 2.25
   x0, x1 = 0., 2.25  # moved start of path to center
   c1, r = [x1,0], 1.25
   line1 = tdg.make_line_path([x0, r], [x1, r], res=res)
   circle1 = tdg.make_circle_path(c1, -r, np.pi/2, np.pi, res=res)
   x2, y2 = 1.581, -0.35
   line2 = tdg.make_line_path([x1, -r], [x2, -r], res=0.01)

   c2, r2, th2 = [x2, y2], 0.9, np.deg2rad(61.8)
   circle2 = tdg.make_circle_path(c2, -r2, -np.pi/2, th2, res=res)

   c3, r3 = [0, -1.21], 0.9
   circle3 = tdg.make_circle_path(c3, r3, np.pi/2-th2, 2*th2, res=res)

   c4, r4 = [-x2, y2], 0.9
   circle4 = tdg.make_circle_path(c4, -r4, -(np.pi/2-th2), th2, res=res)
      
   line3 = tdg.make_line_path([-x2, -r], [-x1, -r], res=res)
   circle5 = tdg.make_circle_path([-x1, 0], -r, -np.pi/2, np.pi, res=res)
   line4 = tdg.make_line_path([-x1, r], [x0, r], res=res)
   line1.append([circle1, line2, circle2, circle3, circle4, line3, circle5, line4])
   p = line1
   fname = '/tmp/path_tmp.npz'
   p.save(fname)

   # add landmarks and velocity profile
   p1 =  trr_u.TrrPath(fname, v=1.)
   p1.lm_s = [p.dists[-1]-1.25, 1.25] # start and finish are located -1.25 and 1.25 from path origin
   
   vel_prof_id = 0
   vels = [[1., 1., 1., 1., 1., 1., 1., 1., 1.], 
           [1.2, 1., 1., 1., 1., 1., 1., 1., 1.2],
           [1.4, 1.2, 1.1, 1., 1., 1., 1.1, 1.2, 1.4],
           [1.5, 1.3, 1.2, 1.0, 1.0, 1.0, 1.2, 1.3, 1.5],
           [1.75, 1.4, 1.2, 1.0, 1.0, 1.0, 1.2, 1.4, 1.75],
           [2.0, 1.4, 1.4, 1., 1., 1., 1.4, 1.4, 2.0]]

   make_vel_profile(p1, vels[vel_prof_id], a_max=0, omega_ref=5.5, braking_delay=100)
   p1.report()
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   #fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_sim_{}.npz'.format(vel_prof_id))
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real_{}.npz'.format(vel_prof_id))
   p1.save(fname)
   
   return fname, p1

def make_vedrines_path(res=0.01):
   # origin of path is at the center of the long straight line
   lw = 1.5                # lane width
   x0, x1 = 0., 22.5       # start and end of first line (step0)
   c1, r1 = [x1,0], 1.75   # center and radius of first circle (step1)
   line1 = tdg.make_line_path([x0, r1], [x1, r1], res=res)
   circle1 = tdg.make_circle_path(c1, -r1, np.pi/2, np.pi, res=res)

   d1 = 3.83
   r2 = 2
   x2, y2 = 2*d1, r2-r1
   line2 = tdg.make_line_path([x1, -r1], [x2, -r1], res=0.01)

   c2, th2 = [x2, y2], np.deg2rad(51.1)  # 51 < th2 < 51.15
   circle2 = tdg.make_circle_path(c2, -r2, -np.pi/2, th2, res=res)

   r3 = 2.927
   x3, y3 = d1, -r1-1.1
   c3, th3, dth3 = [x3, y3], np.pi/2-th2, 2*th2
   circle3 = tdg.make_circle_path(c3, r3, th3, dth3, res=res)

   r4, c4, th4, dth4 = r2, [0, y2], -np.pi/2+th2, 2*th2
   circle4 = tdg.make_circle_path(c4, -r4, th4, dth4, res=res)

   r5, c5, th5, dth5 = r3, [-x3, y3], th3, dth3
   circle5 = tdg.make_circle_path(c5, r5, th5, dth5, res=res)

   r6, c6, th6, dth6 = r2, [-x2, y2], -np.pi/2+th2, th2
   circle6 = tdg.make_circle_path(c6, -r6, th6, dth6, res=res)

   line3 = tdg.make_line_path([-x2, -r1], [-x1, -r1], res=0.01)

   r7, c7, th7, dth7 = r1, [-x1, 0], -np.pi/2, np.pi
   circle7 = tdg.make_circle_path(c7, -r7, th7, dth7, res=res)

   line4 = tdg.make_line_path([-x1, r1], [-x0, r1], res=res)
   
   line1.append([circle1, line2, circle2, circle3, circle4, circle5, circle6, line3, circle7, line4])
   p = line1
 
   fname = '/tmp/path_tmp.npz'
   p.save(fname)

   # add landmarks and velocity profile
   p1 =  trr_u.TrrPath(fname, v=1.)
   p1.lm_s = [0., 16.66] # start and finish are located at 0 and 16.66 m from path origin
   idx_velp = 1
   vels = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
           [3, 2, 3, 2, 2, 2, 2, 2, 3, 2, 3],
           [4, 2, 4, 2, 2, 2, 2, 2, 4, 2, 4],
           [6, 2.5, 6, 2, 2.5, 2, 2.5, 2, 6, 2.5, 6],
           [7, 2.5, 7, 2, 2.5, 2, 2.5, 2, 7, 2.5, 7],
           [8, 2.5, 8, 2, 2.5, 2, 2.5, 2, 8, 2.5, 8]]
   for idx_velp in range(len(vels)):
      make_vel_profile(p1, vels[idx_velp], a_max=0, omega_ref=3., braking_delay=100)
      p1.report()
      tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
      fname = os.path.join(tdg_dir, 'paths/vedrines/track_trr_{}.npz'.format(idx_velp))
      p1.save(fname)
   return fname, p1


# velocity profile proportional to path curvature
# (for reference)
def make_curvature_based_vel_profile(fname, v0=1.75, kc=0.75, _plot=False):
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   fname = os.path.join(tdg_dir, 'paths/demo_z/track_trr_real.npz')
   path = trr_u.TrrPath(fname)
   for i, c in enumerate(path.curvatures):
      path.vels[i] = v0 - kc*abs(c) # why not ...
   fltrd, delayed_curv = trr_u.filter(path.vels, path.dists)
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
   


def find_curvature_jumps(path):
   discs = []
   for i, (c1, c2) in enumerate(zip(path.curvatures[:-1], path.curvatures[1:])):
      if c1 != c2: discs.append((i, c1, c2))
   print(' discontinuities ({}): {}'.format(len(discs), discs))
   return discs

def _saturate_vel(path, a_max):
   path.time = np.zeros(len(path.points))
   for i in range(1, len(path.time)):
      dist = path.dists[i]-path.dists[i-1]
      dt = dist/path.vels[i-1]
      path.time[i] = path.time[i-1] + dt
      dv = path.vels[i] - path.vels[i-1]
      if dt > 0:
         a = min(a_max, dv/dt) if dv>=0 else max(-a_max, dv/dt)
         path.vels[i] = path.vels[i-1] + a*dt
      else:
         path.vels[i] = path.vels[i-1]


def _set_piecewise_setpoints(discs, vels, path):
   idx_vel = 0
   vel_sps = np.zeros(len(path.points))
   for i in range(len(vel_sps)):
      vel_sps[i] = vels[idx_vel]
      if idx_vel<len(discs) and i == discs[idx_vel][0]: idx_vel+=1
   return vel_sps


def make_vel_profile(path, vels, a_max, omega_ref=3, braking_delay=100):
   print('building velocity profile')
   # path is comprised of (first order) discontinuous lines and circles.
   # record transitions
   discs = find_curvature_jumps(path)
   # make a piecewise constant velocity profile
   vel_sps = _set_piecewise_setpoints(discs, vels, path)
   path.vels = vel_sps
   # saturation... not so much....
   if a_max>0:
      _saturate_vel(path, a_max=a_max)
   # anticipate braking
   for i, disc in enumerate(discs):
      if vels[i] > vels[i+1]: 
         path.vels[disc[0]-braking_delay:disc[0]] = vels[i+1]
   if 1: # reference model
      fltrd = trr_u.filter(path.vels, path.dists, omega=omega_ref)
      path.vels = fltrd[:,0]
      path.accels = fltrd[:,1]
      path.jerks = fltrd[:,2]

   path.compute_time()
   print('lap time: {:.2f}s'.format(path.time[-1]))


def plot_vedrines():
   tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
   fname = os.path.join(tdg_dir, 'paths/vedrines/track_trr.npz')
   plot_path(trr_u.TrrPath(fname))
   
def plot_path(_p):
   #tdg.draw_path_curvature(plt.gcf(), plt.gca(), p)
   #tdg.draw_path_vel(plt.gcf(), plt.gca(), _p)
   #tdg.draw_path_accel(plt.gcf(), plt.gca(), _p)
   #tdg.draw_path_vel_profile(plt.gcf(), _p)
   tdg.draw_path_vel_profile_chrono(plt.gcf(), _p)
   #plt.figure()
   #tdg.draw_path(plt.gcf(), plt.gca(), _p)
   plt.figure()
   ax1 = plt.subplot(3,1,1)
   tdg.draw_path_vel_2D(plt.gcf(), ax1, _p)
   ax2 = plt.subplot(3,1,2)
   tdg.draw_path_accel_2D(plt.gcf(), ax2, _p)
   ax3 = plt.subplot(3,1,3)
   tdg.draw_path_lat_accel_2D(plt.gcf(), ax3, _p)

   
if __name__ == '__main__':
   logging.basicConfig(level=logging.INFO)
   np.set_printoptions(linewidth=300, suppress=True)
   #fname, p = make_z_room_path()
   fname, p = make_vedrines_path()
   if 1:
      plot_path(p)
      plt.show()
   if 0:
      plot_vedrines()
      plt.show()
   if 0:
      p2 = make_curvature_based_vel_profile(fname)
      
