#twist_controller.py
#Within this class, you can import and use the provided pid.py and lowpass.py if needed for acceleration,
# and yaw_controller.py for steering

from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.__dict__.update(kwargs)

        self.min_speed=0.1;

        self.throttlepid = PID(kp=1, ki=0.1, kd=0.1, mn=self.decel_limit, mx=self.accel_limit)
        self.brakepid = PID(kp=3, ki=0.1, kd=0.1, mn=self.decel_limit, mx=self.accel_limit)
        self.yawcontroller = YawController(wheel_base=self.wheel_base,steer_ratio=self.steer_ratio,min_speed=self.min_speed,max_lat_accel=self.max_lat_accel,max_steer_angle=self.max_steer_angle)
        self.throttlefilter = LowPassFilter(tau = 3, ts = 1)
        self.brakefilter = LowPassFilter(tau=3, ts=1)
        self.steerfilter = LowPassFilter(tau=3, ts=1)

        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        twist_cmd=kwargs['twist']#current
        current_velocity=kwargs['velocity']#current
        deltatime=kwargs['delay']

        linear_velocity = abs(twist_cmd.twist.linear.x)
        angular_velocity = twist_cmd.twist.angular.z
        cte = linear_velocity - current_velocity.twist.linear.x

        steer_ = self.yawcontroller.get_steering(linear_velocity, angular_velocity, current_velocity.twist.linear.x)
        steer = self.steerfilter.filt(steer_)

        throttle_ = self.throttlepid.step(cte, deltatime)
        throttle = self.throttlefilter.filt( throttle_)

        brake_ = self.brakepid.step(cte, deltatime)
        mbrake = self.brakefilter.filt(brake_)

        if throttle > 0.0:
           if throttle > 1:
               throttle=1 #Note that throttle values passed to publish should be in the range 0 to 1
           brake = 0.0
        else:
           throttle = 0.0
           if mbrake < self.brake_deadband:
              mbrake = 0.0
           #Brake values passed to publish should be in units of torque (N*m).
           #The correct values for brake can be computed using the desired acceleration,
           #weight of the vehicle, and wheel radius.
           brake = -mbrake * (self.vehicle_mass + self.fuel_capacity * GAS_DENSITY) * self.wheel_radius

        # Return throttle, brake, steer
        return throttle, brake, steer

    def reset(self):
        self.throttlepid.reset()
        self.brakepid.reset()