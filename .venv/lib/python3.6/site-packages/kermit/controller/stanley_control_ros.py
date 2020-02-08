import rospy
import numpy as np
import matplotlib.pyplot as plt
import sys
import kermit.controller.cubic_spline_planner as cubic_spline_planner
import random
from geometry_msgs.msg import Twist
from kermit.msg import path_state
from tf.transformations import euler_from_quaternion
from kermit.common import *

k = 0.5 # Control Gain

Kp = 1.0 # Proportional gain
Kd = 0.0
Ki = 0.0

dt = 0.1 # time difference (seconds)
L = 2.9 # Wheel base of vehicle (meters)

max_steer = np.radians(30.0) # max steering angle
show_animation = True

class KermitState(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0, desired_vel=1.0):
        """Instantiate the object."""
        super(KermitState, self).__init__()

        # The current state of the system
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w
        self.desired_vel = desired_vel

        # Publish value
        self.cmd = Twist()

        # Command velocity publishej
        self.cmd_pub = rospy.Publisher(topics["cmd_vel_topic"], Twist, queue_size=10)
        self.pos_sub = rospy.Subscriber(topics["path_state_topic"], path_state, self.callback)


    # Updates current state
    def callback(self, p_state):
        # Update position
        self.x = p_state.lin.x  
        self.y = p_state.lin.y
    
        self.yaw = p_state.ang.z

        # TODO
        #  self.v = p_state.lin_vel
        self.w = p_state.ang_vel


    # Send command to teensy
    def update(self, ai, di):
        di = np.clip(di, -max_steer, max_steer)

        # Linear command vel
        self.cmd.linear.x = self.v + ai * dt
        print(self.cmd.linear.x)
        self.cmd.angular.z = di

        #  print(self.yaw)




def pid_control(target, current):
    """
    Proportional control for the speed.
    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    ret = Kp * (target - current) 
    return ret


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.
    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.
    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def main(ax, ay):
    if(ax == None or ay == None):
        ax = [0.0, 100.0, 200.0, 300.0, 500.0]
        ay = [0.0, 0.0, 0.0, -20.0, 20.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    target_speed = 30.0  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    state = KermitState(x=-0.0, y=5.0, yaw=0.0, v=30.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    rate = rospy.Rate(1 / dt)

    while not rospy.is_shutdown() and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        #  print(state.cmd)
        print(state.x, state.y)
        state.cmd_pub.publish(state.cmd)

        
        time += dt
        a = 0
        b = 0
        x.append(state.x + random.uniform(a, b))
        y.append(state.y + random.uniform(a, b))
        yaw.append(state.yaw * random.uniform(a, b))
        v.append(state.v * random.uniform(a, b))
        t.append(time)
        rate.sleep()


    assert last_idx >= target_idx, "Cannot reach goal"



