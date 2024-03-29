#!/usr/bin/env python
import os, sys, roslib, rospy, rospkg, rostopic
import dynamic_reconfigure.client as dyn_rec_clt,  dynamic_reconfigure.server as dyn_rec_srv

import pdb

import two_d_guidance.trr.utils as trr_u
import two_d_guidance.trr.rospy_utils as trr_rpu
import two_d_guidance.srv

import trr.cfg.race_managerConfig
import trr.race_manager as trr_rm

# TODO: add traffic light display

class Node(trr_rpu.PeriodicNode):
    def __init__(self, autostart=False):
        trr_rpu.PeriodicNode.__init__(self, 'race_manager_node')
        # this is our model
        self.race_manager = trr_rm.RaceManager()
        # we publish our status
        self.status_pub = trr_rpu.RaceManagerStatusPublisher()
        # we expose a service to be informed ( by state estimation) when landmarks are passed
        self.lm_service = rospy.Service('LandmarkPassed', two_d_guidance.srv.LandmarkPassed, self.on_landmark_passed)
        # we manipulate parameters exposed by the guidance node
        guidance_client_name = "trr_guidance_node"
        self.guidance_cfg_client = dyn_rec_clt.Client(guidance_client_name, timeout=30,
                                                      config_callback=self.guidance_cfg_callback)
        rospy.loginfo(' guidance_client_name: {}'.format(guidance_client_name))
        # we will expose some parameters to users
        self.race_manager_cfg_srv = dyn_rec_srv.Server(trr.cfg.race_managerConfig,
                                                       self.race_manager_cfg_callback)
        # we start either racing or idle
        self.dyn_cfg_update_race_mode(self.race_manager.mode_racing if autostart else self.race_manager.mode_staging)
        # we subscribe to state estimator and traffic light
        self.state_est_sub = trr_rpu.TrrStateEstimationSubscriber(what='race_manager')
        self.traffic_light_sub = trr_rpu.TrrTrafficLightSubscriber()

        # we expose a service for loading a velocity profile
        self.lm_service = rospy.Service('RaceManagerLoadPath', two_d_guidance.srv.GuidanceLoadVelProf, self.on_load_path)

    def on_load_path(self, req):
        path_filename = ''.join(req.path_filename)
        print('on_load_path {}'.format(path_filename))
        err = 0#self.guidance.load_vel_profile(path_filename)
        return two_d_guidance.srv.GuidanceLoadVelProfResponse(err)
    
    def on_landmark_passed(self, req):
        rospy.loginfo('  on landmark passed {}'.format(req.id))
        if req.id == trr_u.TrrPath.LM_FINISH:
            self.race_manager.next_lap(rospy.Time.now())
        return two_d_guidance.srv.LandmarkPassedResponse(0)
        
    def dyn_cfg_update_race_mode(self, mode):
        self.race_manager_cfg_srv.update_configuration({'mode': mode})

    # remove that for now
    # def dyn_cfg_update_cur_lap(self, _v):
    #     print('dyn_cfg_update_cur_lap {}'.format(_v))
    #     self.race_manager.set_cur_lap(_v)
    #     self.race_manager_cfg_srv.update_configuration({'cur_lap': self.cur_lap})

    def exit(self): self.set_guidance_mode(0) # set guidance is to idle when exiting
        
    # reconfigure (dyn config) race_mode    
    def set_race_mode(self, mode):
        mode_name = ['Staging', 'Ready', 'Racing', 'Finished', 'JoinStart']
        rospy.loginfo('  set race mode to {}'.format(mode_name[mode]))
        self.race_manager.set_mode(mode, self, rospy.Time.now())
        
    #def set_cur_lap(self, v): self.race_manager.set_cur_lap(v)

    def race_manager_cfg_callback(self, config, level):
        rospy.loginfo("  Race Manager Reconfigure Request:")
        #pdb.set_trace()
        #print config, level
        self.set_race_mode(config['mode'])
        self.race_manager.set_nb_lap(config['nb_lap'])
        #self.cur_lap = config['cur_lap']
        return config

    # I get a confirmation when guidance config is changed
    def guidance_cfg_callback(self, config): pass
        
    def periodic(self):
        try:
            self.race_manager.periodic(self.state_est_sub, self.traffic_light_sub, self.dyn_cfg_update_race_mode, rospy.Time.now())
        except trr_rpu.NoRXMsgException:
            rospy.loginfo_throttle(1., 'NoRXMsgException')
        except trr_rpu.RXMsgTimeoutException:
            rospy.loginfo_throttle(1., 'RXMsgTimeoutException')
        self.status_pub.publish(self.race_manager)
            
    def set_guidance_mode(self, mode):
        mode_name = ['Idle', 'Stop', 'Drive']
        rospy.loginfo('   set guidance mode to {}'.format(mode_name[mode]))
        if self.guidance_cfg_client is not None:
            self.guidance_cfg_client.update_configuration({"guidance_mode":mode})

    def set_guidance_vel_ctl_mode(self, mode):
        mode_name = ['Cst', 'Profile', 'Curv']
        rospy.loginfo('   set guidance vel ctl mode to {}'.format(mode_name[mode]))
        self.guidance_cfg_client.update_configuration({"vel_ctl_mode":mode})

    def set_guidance_vel_sp(self, _v):
        rospy.loginfo('   set guidance vel sp to {}'.format(_v))
        self.guidance_cfg_client.update_configuration({"vel_sp":_v})
        
        
def main(args):
    Node().run(20)


if __name__ == '__main__':
    main(sys.argv)
