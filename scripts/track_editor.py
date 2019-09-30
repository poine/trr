#!/usr/bin/env python
import argparse, logging, sys, os, math, numpy as np, cv2, gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, GdkPixbuf, GLib, GObject
import matplotlib
import pdb
import threading
import make_track

# /home/poine/work/smocap/smocap_gazebo/scripts/hog_remote.py

# glade

# roscore

import rospy, sensor_msgs.msg, visualization_msgs.msg, geometry_msgs.msg, std_msgs.msg, nav_msgs.msg


import two_d_guidance.trr.utils as trr_u


class GUI:
    def __init__(self):
        self.last_dir = os.getcwd()
        self.b = Gtk.Builder()
        gui_xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'track_editor_gui.xml')
        self.b.add_from_file(gui_xml_path)
        self.window = self.b.get_object("window")
        self.window.set_title('Track Editor')
        
        self.position_init()
        self.speed_init()
        self.offset_init()

        self.b.get_object("button_add_speed").connect("clicked", self.on_add, Point.TYPE_SPEED)
        self.b.get_object("button_add_offset").connect("clicked", self.on_add, Point.TYPE_OFFSET)
        self.b.get_object("button_remove").connect("clicked", self.on_remove)
        self.b.get_object("button_show").connect("clicked", self.on_show)
        self.b.get_object("button_save").connect("clicked", self.on_save)
        self.window.connect("delete-event", self.quit)

        self.list_points = self.b.get_object("list_points")
        self.list_points.connect("row-selected", self.on_selection)
        self.list_points.set_sort_func(self.row_sorter)

        self.window.connect("delete-event", self.quit)

        self.window.show_all()
        self.position_visibility(False)
        self.speed_visibility(False)
        self.offset_visibility(False)
        self.active_point = None

    def set_app(self, app):
        self.app = app

    def add_point(self, point):
        self.list_points.add(point.label)
        self.list_points.show_all()

    def remove_point(self, point):
        self.list_points.remove(point.label.get_parent())
        
    def activate(self, point):
        self.active_point = point
        self.position_set_value(point.position)
        self.position_visibility(True)
        self.speed_set_value(point.speed)
        self.speed_visibility(point.point_type == Point.TYPE_SPEED)
        self.offset_size_set_value(point.offset_size)
        self.offset_length_set_value(point.offset_length)
        self.offset_visibility(point.point_type == Point.TYPE_OFFSET)

    def row_sorter(list, row1, row2):
        return row1.get_child().get_text() > row2.get_child().get_text()
        
    def on_selection(self, list, row):
        if row is None:
            self.active_point = None
            self.position_visibility(False)
            self.speed_visibility(False)
            self.offset_visibility(False)
        else:
            label = row.get_child()
            self.activate(label.point)
        
    def on_add(self, button, point_type):
        point = self.app.create_point(point_type)
        self.activate(point)
        self.list_points.select_row(self.active_point.label.get_parent())

    def on_remove(self, button):
        if self.active_point is not None:
            self.app.delete_point(self.active_point)

    def on_show(self, button):
        self.app.compute(save=False)
        
    def on_save(self, button):
        self.app.compute(save=True)
        
    def on_position_changed(self, scale):
        self.active_point.set_position(scale.get_value())
        self.list_points.invalidate_sort()
        
    def on_speed_changed(self, scale):
        self.active_point.set_speed(scale.get_value())
        self.list_points.invalidate_sort()
        
    def on_offset_size_changed(self, scale):
        self.active_point.set_offset_size(scale.get_value())
        self.list_points.invalidate_sort()
        
    def on_offset_length_changed(self, scale):
        self.active_point.set_offset_length(scale.get_value())
        self.list_points.invalidate_sort()
        
    def run(self):
        Gtk.main()

    def quit(self, a, b):
        rospy.signal_shutdown("just because")
        self.ros_thread.join()
        Gtk.main_quit() 

    def position_init(self):
        self.position = self.b.get_object("scale_position")
        adj = Gtk.Adjustment(0., 0., 110., 0.05, 2., 0.)
        self.position.set_adjustment(adj)
        self.position.connect("value-changed", self.on_position_changed)

    def position_visibility(self, visible):
        self.position.get_parent().set_visible(visible)

    def position_set_value(self, value):
        self.position.set_value(value) 

    def set_path_length(self, value):
        self.path_length = value
        self.position.get_adjustment().set_upper(value)
        
    def speed_init(self):
        self.speed = self.b.get_object("scale_speed")
        adj = Gtk.Adjustment(1., 0.1, 8., 0.05, 1., 0)
        self.speed.set_adjustment(adj)
        self.speed.connect("value-changed", self.on_speed_changed)
        
    def speed_visibility(self, visible):
        self.speed.get_parent().set_visible(visible)

    def speed_set_value(self, value):
        self.speed.set_value(value)

    def offset_init(self):
        self.offset_size = self.b.get_object("scale_offset_size")
        self.offset_size.set_property("digits", 2)
        adj = Gtk.Adjustment(0., -0.5, 0.5, 0.01, 0.1, 0)
        self.offset_size.set_adjustment(adj)
        self.offset_size.connect("value-changed", self.on_offset_size_changed)

        self.offset_length = self.b.get_object("scale_offset_length")
        adj = Gtk.Adjustment(3., 0.2, 15, 0.1, 1., 0)
        self.offset_length.set_adjustment(adj)
        self.offset_length.connect("value-changed", self.on_offset_length_changed)

    def offset_visibility(self, visible):
        self.offset_size.get_parent().set_visible(visible)

    def offset_size_set_value(self, value):
        self.offset_size.set_value(value)

    def offset_length_set_value(self, value):
        self.offset_length.set_value(value)


def msgPoint(x, y, z): p = geometry_msgs.msg.Point(); p.x=x; p.y=y;p.z=z; return p
def msgColor(r, g, b, a): c = std_msgs.msg.ColorRGBA(); c.r = r; c.g = g; c.b = b; c.a = a; return c

class Node:
    def __init__(self):
        self.suspend_periodic = False
        self.is_periodic_running = False
        self.trr_path = None
        self.path_points = None
        self.track_points = None
        self.points = None
        self.text_count = 0
        self.path_msg_track_points = []
        self.path_msg_path_points = []
        
        self.pub_path = rospy.Publisher('track_editor/path', visualization_msgs.msg.Marker, queue_size=1)
        self.path_msg = visualization_msgs.msg.Marker()
        self.path_msg.header.frame_id = 'world'
        self.path_msg.type = self.path_msg.LINE_STRIP
        self.path_msg.action = self.path_msg.ADD
        self.path_msg.id = 0
        self.path_msg.text = 'path points and track points'
        o = self.path_msg.pose.orientation; o.w, o.x, o.y, o.z = 1, 0, 0, 0

        self.pub_points = rospy.Publisher('track_editor/points', visualization_msgs.msg.Marker, queue_size=1)
        self.points_msg = visualization_msgs.msg.Marker()
        self.points_msg.header.frame_id = 'world'
        self.points_msg.type = self.points_msg.LINE_LIST
        self.points_msg.action = self.points_msg.ADD
        self.points_msg.id = 0
        self.points_msg.text = 'Position for edition points'
        rgba = (1., 1., 0, 1.)
        s = self.points_msg.scale; s.x, s.y, s.z = 0.1, 0.0, 0.0
        c = self.points_msg.color; c.r, c.g, c.b, c.a = rgba
        o = self.points_msg.pose.orientation; o.w, o.x, o.y, o.z = 1, 0, 0, 0

        self.pub_speed = rospy.Publisher('track_editor/speed', visualization_msgs.msg.Marker, queue_size=1)
        self.speed_msg = visualization_msgs.msg.Marker()
        self.speed_msg.header.frame_id = 'world'
        self.speed_msg.type = self.speed_msg.TEXT_VIEW_FACING
        self.speed_msg.text = 'Text value for speed limit'
        rgba = (1., 1., 0, 1.)
        s = self.speed_msg.scale; s.x, s.y, s.z = 0.0, 0.0, 0.7
        c = self.speed_msg.color; c.r, c.g, c.b, c.a = rgba
        o = self.speed_msg.pose.orientation; o.w, o.x, o.y, o.z = 1, 0, 0, 0

    def set_trr_path(self, trr_path):
        self.trr_path = trr_path
        
    def set_points(self, points):
        self.points = points
        
    def update_path(self, path_points):
        self.path_points = path_points
        self.path_msg_path_points = []
        for x, y in path_points:
            self.path_msg_path_points.append(msgPoint(x, y, 0))

    def update_track(self, track_points):
        self.track_points = track_points
        self.path_msg_track_points = []
        for x, y in track_points:
            self.path_msg_track_points.append(msgPoint(x, y, 0))

    def update_points(self):
        self.points_msg.points = []
        self.points_msg.colors = []
        speed_color = msgColor(1, 1, 1, 1)
        offset_color = msgColor(0, 0.7, 0, 1)
        selected_color = msgColor(1, 0, 0, 1)
        if self.path_points is not None and self.points is not None:
            for point in self.points:
                x, y = self.track_points[int(point.position*100)]
                if point.point_type == Point.TYPE_SPEED:
                    std_color = speed_color
                else:
                    std_color = offset_color
                self.points_msg.points.append(msgPoint(x, y, 0))
                self.points_msg.points.append(msgPoint(x, y, 1.5))
                self.points_msg.colors.append(std_color)
                if point.is_selected():
                    self.points_msg.colors.append(selected_color)
                else:
                    self.points_msg.colors.append(std_color)

    def publish_speeds(self):
        self.speed_msg.action = self.speed_msg.ADD
        id = 0
        if self.track_points is not None and self.points is not None:
            for point in self.points:
                if point.point_type == Point.TYPE_SPEED:
                    self.speed_msg.id = id
                    id += 1
                    self.speed_msg.text = str(point.speed)
                    x, y = self.track_points[int(point.position*100)]
                    self.speed_msg.pose.position = msgPoint(x, y, 1.7)
                    self.pub_speed.publish(self.speed_msg)
        self.speed_msg.action = self.speed_msg.DELETE
        for i in range(id, self.text_count):
            self.speed_msg.id = i
            self.pub_speed.publish(self.speed_msg)
        self.text_count = id

    def set_suspend_periodic(self, value):
        self.suspend_periodic = value

    def periodic(self):
        if self.trr_path is None or self.suspend_periodic:
            self.is_periodic_running = False
            return
        self.is_periodic_running = True
        self.trr_path.offset_points = []
        for point in self.points:
            if point.point_type == Point.TYPE_OFFSET:
                self.trr_path.offset_points.append([point.position, point.offset_size, point.offset_length])
        self.trr_path.build_path_only()
        self.update_path(self.trr_path.points)
        self.path_msg.ns = 'track'
        self.path_msg.scale.x = 1.5
        self.path_msg.pose.position.z = -0.78
        self.path_msg.color = msgColor(0.3, 0.3, 0.3, 1)
        self.path_msg.points = self.path_msg_track_points
        self.pub_path.publish(self.path_msg)

        self.path_msg.ns = 'white line'
        self.path_msg.scale.x = 0.05
        self.path_msg.pose.position.z = 0
        self.path_msg.color = msgColor(1, 1, 1, 1)
        self.path_msg.points = self.path_msg_track_points
        self.pub_path.publish(self.path_msg)

        self.path_msg.ns = 'car'
        self.path_msg.scale.x = 0.27
        self.path_msg.pose.position.z = 0.
        self.path_msg.color = msgColor(0.3, 0.6, 1.0, 0.6)
        self.path_msg.points = self.path_msg_path_points
        self.pub_path.publish(self.path_msg)

        self.update_points()
        self.pub_points.publish(self.points_msg)  
        self.publish_speeds()

    def run(self, freq=10):
        rate = rospy.Rate(freq)
        try:
            while not rospy.is_shutdown():
                self.periodic()
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass


class Point:
    TYPE_SPEED, TYPE_OFFSET, TYPE_COUNT = range(3)
    def __init__(self, point_type, position=0., speed=0., offset_size=0., offset_length=0.):
        self.point_type = point_type
        self.position = position
        self.speed = speed
        self.offset_size = offset_size
        self.offset_length = offset_length
        self.label = Gtk.Label()
        self.label.point = self
        self.update()
        
    def as_text(self):
        if self.point_type == Point.TYPE_SPEED:
            return "position: {:5.1f}  --  speed limit: {:3.1f}".format(self.position, self.speed)
        else:
            return "position: {:5.1f}  --  offset size: {:5.2f}  offset length: {:4.1f}".format(self.position, self.offset_size, self.offset_length)

    def set_position(self, value):
        self.position = value
        self.update()

    def set_speed(self, value):
        self.speed = value
        self.update()
        
    def set_offset_size(self, value):
        self.offset_size = value
        self.update()

    def set_offset_length(self, value):
        self.offset_length = value
        self.update()

    def is_selected(self):
        row = self.label.get_parent()
        if row is None:
            return False
        return row.is_selected()

    def update(self):
        self.label.set_text(self.as_text())
        
class App:
    def __init__(self, gui, node):
        self.gui = gui
        self.node = node
        self.points = []
        node.set_points(self.points)
        self.load_path()
        gui.set_app(self)
        
    def load_path(self):
        #path_filename= '/home/poine/work/two_d_guidance/paths/vedrines/track_trr_0.npz'
        path_filename= '/tmp/ph.npz' #TODO remove
        #tdg_dir = rospkg.RosPack().get_path('two_d_guidance')
        #fname = os.path.join(tdg_dir, 'paths/vedrines/track_trr_{}.npz'.format(idx_velp))
        self.trr_path = trr_u.TrrPath(path_filename)
        self.node.set_trr_path(self.trr_path)
        self.node.update_track(self.trr_path.track_points)
        self.gui.set_path_length((len(self.trr_path.points) - 2) / 100)
        self.points = []
        for position, speed in self.trr_path.speed_points:
            self.create_point(Point.TYPE_SPEED, position=position, speed=speed)
        for position, offset_size, offset_length in self.trr_path.offset_points:
            self.create_point(Point.TYPE_OFFSET, position=position, offset_size=offset_size, offset_length=offset_length)
        node.set_points(self.points)

    def compute(self, save=False):
        self.node.set_suspend_periodic(True)
        while self.node.is_periodic_running: pass
        self.trr_path.speed_points = []
        self.trr_path.offset_points = []
        for point in self.points:
            if point.point_type == Point.TYPE_SPEED:
                self.trr_path.speed_points.append([point.position, point.speed])
            else:
                self.trr_path.offset_points.append([point.position, point.offset_size, point.offset_length])
        self.trr_path.compute_all()
        self.trr_path.report()
        if save:
            self.trr_path.save('/tmp/ph.npz')
        make_track.plot_path(self.trr_path)
        self.node.set_suspend_periodic(False)
        matplotlib.pyplot.show() #TODO fix mainloop problem
                
    def create_point(self, point_type, position=0., speed=0., offset_size=0., offset_length=0.):
        point = Point(point_type, position=position, speed=speed, offset_size=offset_size, offset_length=offset_length)
        self.points.append(point)
        self.gui.add_point(point)
        return point

    def delete_point(self, point):
        self.gui.remove_point(point)
        self.points.remove(point)
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=3, linewidth=300)
    #parser = argparse.ArgumentParser(description='Tune Blob Detector.')
    rospy.init_node('fake_detector_node')
    gui = GUI()

    node = Node()
    ros_thread = threading.Thread(target=node.run)
    ros_thread.start()
    gui.ros_thread = ros_thread

    App(gui, node)
    
    gui.run()
    
    
