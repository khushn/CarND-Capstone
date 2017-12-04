#!/usr/bin/env python

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time
import rospy

GAS_DENSITY = 2.858 
ONE_MPH = 0.44704 # 1 mile/hr = 0.44704 met/s
MAX_SPEED = 40.0 # 4o mils/hr 
WEIGHT_PERSON = 75


class TwistController(object):
    def __init__(self, wheel_base, wheel_radius, vehicle_mass, steer_ratio, max_lat_accel, max_steer_angle,throttle_PID_Para,
            decel_limit,accel_limit,fuel_capacity,brake_deadband):
        # TODO: Implement
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.vehicle_mass = vehicle_mass
        self.steer_ratio = steer_ratio
        self.max_lat_accel = max_lat_accel
        self.max_steer_angle = max_steer_angle
        self.brake_deadband = brake_deadband
        
        self.throttle_PID_Para =throttle_PID_Para
        self.throttle_pid = PID(self.throttle_PID_Para[0],self.throttle_PID_Para[1],self.throttle_PID_Para[2],)        
        
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.fuel_capacity = fuel_capacity
        
        self.filter = LowPassFilter(0.2,0.1)
        self.last_t = None
        
        self.tot_veh_mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY + 2 * WEIGHT_PERSON 

    def control(self, target_linear_vel, target_angular_vel, current_vel,dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        if self.last_t is None or not dbw_enabled:
            self.last_t = rospy.get_time()
            return 0.0, 0.0, 0.0
        
        
        dt = rospy.get_time() - self.last_t
        
        #rospy.loginfo('DBW_NODE:dt=%f',dt) 
        error_v = min(target_linear_vel, MAX_SPEED*ONE_MPH) - current_vel 
        
        #rospy.loginfo('DBW_NODE:error_v=%f',error_v) 
        
        #applying acceleration and decelartion limit to error for smoothness  

        
        error_v = max(self.decel_limit*dt, min(self.accel_limit*dt, error_v))
        
        #rospy.loginfo('DBW_NODE:error_v post limit =%f',error_v)
        
        throttle = self.throttle_pid.step(error_v, dt)
        
        #rospy.loginfo('DBW_NODE:throttle post pid =%f',throttle)
        
        #applying limits to throttle
        throttle = max(0.0, min(1.0, throttle))
        
        #rospy.loginfo('DBW_NODE:throttle post limit =%f',throttle)
                        
        
        #min_speed = min(current_vel, target_linear_vel)
        min_speed = 1.0*0.447        
        
        yaw_controller = YawController(
            self.wheel_base, self.steer_ratio, min_speed, self.max_lat_accel, self.max_steer_angle
        )
        steer = yaw_controller.get_steering(
            target_linear_vel, target_angular_vel, current_vel
        )
        
        steer = self.filter.filt(steer)        
        
        if error_v/dt < -self.brake_deadband:
            brake = -throttle * self.tot_veh_mass * self.wheel_radius
            throttle = 0
        else:
            brake = 0 
        
        
        self.last_t = time.time()
        #rospy.loginfo('DBW_NODE:throttle=%f,brake=%f,steer=%f',throttle,brake,steer)
        
        return throttle, brake, steer