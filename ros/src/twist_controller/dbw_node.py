#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

#dbw_node.py

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        args=()
        kwargs={"vehicle_mass":vehicle_mass,"fuel_capacity":fuel_capacity,"brake_deadband":brake_deadband,
                "decel_limit":decel_limit,"accel_limit":accel_limit,"wheel_radius":wheel_radius,
                "wheel_base":wheel_base,"steer_ratio":steer_ratio,"max_lat_accel":max_lat_accel,
                "max_steer_angle":max_steer_angle}

        ######################################################################################
        self.current_velocity = None
        self.current_twist_cmd = None
        self.dbw_enabled = True
        # TIME
        self.lasttime = rospy.get_time()
        ######################################################################################

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd,queue_size=1)# queue_size=10
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd,queue_size=1)#queue_size=10
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd,queue_size=1)# queue_size=10

        # TODO: Create `TwistController` object
        # self.controller = TwistController(<Arguments you wish to provide>)
        # 2) This file also imports the Controller class from twist_controller.py which will be used
        # for implementing the necessary controllers.
        self.controller = Controller(self,*args,**kwargs)
        # TODO: Subscribe to all the topics you need to
        # 1) You will need to write ROS subscribers for the /current_velocity, /twist_cmd, and
        # /vehicle/dbw_enabled topics.
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb,queue_size=1)#
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb, queue_size=1)

        self.loop()

    def current_velocity_cb(self, current_velocity):
        self.current_velocity = current_velocity

    def twist_cmd_cb(self, twist_cmd):
        self.current_twist_cmd = twist_cmd

    def dbw_enabled_cb(self, dbw_enabled):
        try:
            self.dbw_enabled = bool(dbw_enabled.data)
        except Exception:
            self.dbw_enabled = dbw_enabled


    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)

            # TIME
            currenttime = rospy.get_time()
            deltatime = currenttime - self.lasttime
            self.lasttime = currenttime
            # Since a safety driver may take control of the car during testing, you should not assume
            # that the car is always following your commands. If a safety driver does take over,
            # your PID controller will mistakenly accumulate error, so you will need to be mindful
            # of DBW status

            if self.dbw_enabled and self.current_velocity is not None and self.current_twist_cmd is not None:

                args=()
                kwargs = {"twist": self.current_twist_cmd,"velocity":self.current_velocity,"delay":deltatime}
                throttle, brake, steering = self.controller.control(self,*args,**kwargs)
                self.publish(throttle, brake, steering)
            else:
                self.controller.reset()
                self.lasttime = rospy.get_time()



            rate.sleep()

    # 3) The function used to publish throttle, brake, and steering is publish
    # Note that throttle values passed to publish should be in the range 0 to 1. Brake values
    # passed to publish should be in units of torque (N*m). The correct values for brake can
    # be computed using the desired acceleration, weight of the vehicle, and wheel radius.
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
