import numpy as np
import rospy # traces...

class RaceManager:
    mode_staging, mode_ready, mode_racing, mode_finished, mode_join_start = range(5)
    def __init__(self):
        self.mode = RaceManager.mode_staging
        self.cur_lap, self.nb_lap = 0, 2
        self.lap_times = []

    def next_lap(self, stamp):
        self.lap_times[-1] = (stamp - self.lap_start_stamp).to_sec()
        self.lap_start_stamp = stamp
        self.lap_times.append(0.)
        self.cur_lap += 1
        print 'lap time {} cur_lap {}'.format(self.lap_times[-2], self.cur_lap)

    def set_cur_lap(self, _v):
        self.cur_lap = _v
        print 'cur_lap {}'.format(self.cur_lap)

    def set_mode(self, _m, world, stamp):
        self.mode = _m
        cbks = [self.enter_staging, self.enter_ready, self.enter_racing, self.enter_finished, self.enter_join_start]
        cbks[self.mode](world, stamp)

    def periodic(self, state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp):
        cbks = [self.periodic_staging, self.periodic_ready, self.periodic_racing, self.periodic_finished, self.periodic_join_start]
        cbks[self.mode](state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp)

    # Race modes behaviour

    # -Staging: we do nothing
    #
    def enter_staging(self, car, stamp):
        car.set_guidance_mode(0)      # guidance is idle when staging

    def periodic_staging(self, state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp): pass  # and we do nothing
    
    # -Ready: we wait for green light
    #
    def enter_ready(self, car, stamp):
        car.set_guidance_mode(1)     # guidance mode is stopped

    def periodic_ready(self, state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp):
        self.cur_lap = 0
        self.lap_times = []
        tl_red, _, tl_green = traffic_light_sub.get()
        if tl_green and not tl_red : # green light and no red, we race
            update_race_mode_cbk(RaceManager.mode_racing)

    # -Racing: we check for end of race
    #
    def enter_racing(self, car, stamp):
        rospy.loginfo('    entering racing: lap ({}/{})'.format(self.cur_lap, self.nb_lap))
        self.cur_lap = 0
        self.lap_start_stamp = stamp
        self.lap_times = [0.]
        car.set_guidance_vel_ctl_mode(1) # profile
        car.set_guidance_mode(2) # guidance is driving when racing

    def periodic_racing(self, state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp):
        #s_est, v_est, cur_lap, dist_to_start, dist_to_finish = state_est_sub.get()
        #rospy.loginfo('racing_periodic: finish crossed {}'.format(self.finish_crossed))
        #if self.finish_crossed:
            #rospy.loginfo('racing_periodic: finish crossed {}/{}'.format(self.cur_lap, self.nb_lap))
        self.lap_times[-1] = (stamp - self.lap_start_stamp).to_sec()
        if self.cur_lap > self.nb_lap: # we brake
            rospy.loginfo('final lap ({}/{}): braking'.format(self.cur_lap, self.nb_lap))
            update_race_mode_cbk(RaceManager.mode_finished)
            #else:
                #self.dyn_cfg_update_cur_lap(self.cur_lap+1) # or we pass to next lap
            #    self.cur_lap += 1
            #    rospy.loginfo('starting lap {}'.format(self.cur_lap))
            #self.finish_crossed = False

    # -Finished: we do nothing
    #
    def enter_finished(self, car, stamp):
        car.set_guidance_mode(1) # guidance mode is stopped

    def periodic_finished(self, state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp): pass
                
    # -Join Start: we check for approaching start line 
    #
    def enter_join_start(self, car,stamp):
        car.set_guidance_vel_ctl_mode(0) # Cst
        car.set_guidance_mode(2) # driving when going to start

    def periodic_join_start(self, state_est_sub, traffic_light_sub, update_race_mode_cbk, stamp, dist_to_stop_at=0.15):
        s_est, is_est, v_est, dist_to_start, dist_to_finish = state_est_sub.get()
        if dist_to_start < dist_to_stop_at:
            rospy.loginfo('start joined, going to ready')
            update_race_mode_cbk(RaceManager.mode_ready)
        
                
