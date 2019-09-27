#!/usr/bin/env python
import argparse, logging, sys, os, math, numpy as np, cv2, gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, GLib, GObject
import matplotlib
import pdb
import threading

# /home/poine/work/smocap/smocap_gazebo/scripts/hog_remote.py

# glade

import rospy, sensor_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, nav_msgs.msg


import two_d_guidance.trr.utils as trr_u


class GUI:
    def __init__(self):
        self.last_dir = os.getcwd()
        self.b = Gtk.Builder()
        gui_xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'track_editor_gui.xml')
        self.b.add_from_file(gui_xml_path)
        self.window = self.b.get_object("window")
        self.window.set_title('BlobDetector')
        
        scale = self.b.get_object("scale_gamma")
        adj = Gtk.Adjustment(1., 0.1, 2., 0.05, 0.1, 0)
        scale.set_adjustment(adj)

        
        self.b.get_object("button_detect").connect("clicked", self.on_detect_clicked)
        self.window.connect("delete-event", self.quit)

        self.window.show_all()
        
    def on_detect_clicked(self, button):
        #self.run_detector()
        print('clicked')


    def run(self):
        Gtk.main()

    def quit(self, a, b):
        Gtk.main_quit() 
        
def msgPoint(x, y, z): p = geometry_msgs.msg.Point(); p.x=x; p.y=y;p.z=z; return p

class Node:
    def __init__(self):
        path_filename= '/home/poine/work/two_d_guidance/paths/vedrines/track_trr_0.npz'
        #tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
        #fname = os.path.join(tdg_dir, 'paths/vedrines/track_trr_{}.npz'.format(idx_velp))
        self.path =  trr_u.TrrPath(path_filename)

        self.pub_path = rospy.Publisher('track_editor/path', visualization_msgs.msg.Marker, queue_size=1)
        
        self.path_msg = visualization_msgs.msg.Marker()
        self.path_msg.header.frame_id = 'world'
        self.path_msg.type = self.path_msg.LINE_STRIP
        self.path_msg.action = self.path_msg.ADD
        self.path_msg.id = 0
        self.path_msg.text = 'foo'
        rgba = (1., 1., 0, 1.)
        s = self.path_msg.scale; s.x, s.y, s.z = 0.01, 0.2, 0.2
        c = self.path_msg.color; c.r, c.g, c.b, c.a = rgba
        o = self.path_msg.pose.orientation; o.w, o.x, o.y, o.z = 1, 0, 0, 0
        #pdb.set_trace()
        for x, y in self.path.points:
            self.path_msg.points.append(msgPoint(x, y, 0))

        
    def periodic(self):
        self.pub_path.publish( self.path_msg)  

    

    def run(self, freq=10):
        rate = rospy.Rate(freq)
        try:
            while not rospy.is_shutdown():
                self.periodic()
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass

        

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=3, linewidth=300)
    #parser = argparse.ArgumentParser(description='Tune Blob Detector.')
    rospy.init_node('fake_detector_node')
    gui = GUI()

    node = Node()
    ros_thread = threading.Thread(target=node.run)
    ros_thread.start()

    gui.run()
    
    
