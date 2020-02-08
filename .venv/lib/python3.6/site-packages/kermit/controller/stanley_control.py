import rospy
import numpy as np
import matplotlib.pyplot as plt
import sys
import kermit.controller.cubic_spline_planner as cubic_spline_planner
import random
from geometry_msgs.msg import Twist
from kermit.msg import path_state
from tf.transformations import euler_from_quaternion

k = 0.5 # Control Gain

Kp = 10.0 # Proportional gain
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

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        """Instantiate the object."""
        super(KermitState, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w

        self.cmd = Twist()

        # Input initail values To command
        self.cmd.angular.y = 0.0
        self.cmd.angular.x = 0.0
        self.cmd.angular.z = w

        self.cmd.linear.x = v * np.cos(self.yaw)
        self.cmd.linear.y = v * np.sin(self.yaw)
        self.cmd.linear.z = 0.0

        self.cmd_pub = rospy.Publisher('topic', Twist, queue_size=10)
        self.pos_sub = rospy.Subscriber('subs', path_state, self.callback)
        self.cmd_pub.publish(self.cmd)


    # Updates current state
    def callback(self, p_state):
        # Update position
        self.x = p_state.pose.position.x  
        self.y = p_state.pose.position.y
    
        # update orientation
        orientation_q = p_state.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(orientation_list)

        # TODO
        self.v = 2


    # Send command to teensy
    def update(self, acceleration, delta):
        delta = np.clip(delta, -max_steer, max_steer)
        self.cmd.linear.x = acceleration * np.cos(self.yaw)
        self.cmd.linear.y = acceleration * np.cos(self.yaw)
        
        self.cmd.angular.z = delta
        self.cmd_pub.publish(self.cmd)


class State(object):
    """
    Class representing the state of a vehicle.
    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.
        Stanley Control uses bicycle model.
        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


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
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course

    if(ax == None or ay == None):
        ax = [0.0, 100.0, 100.0, 50.0, 60.0]
        ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)


    target_speed = 30.0 / 3.6  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    state = KermitState(x=-0.0, y=5.0, yaw=np.radians(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        
        time += dt
        a = 0
        b = 0
        x.append(state.x + random.uniform(a, b))
        y.append(state.y + random.uniform(a, b))
        yaw.append(state.yaw * random.uniform(a, b))
        v.append(state.v * random.uniform(a, b))
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  # pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


def simul_main(publisher, ax = None, ay=None):
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course

    if(ax == None or ay == None):
        ax = [0.0, 100.0, 100.0, 50.0, 60.0]
        ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)


    target_speed = 30.0 / 3.6  # [m/s]

    max_simulation_time = 100.0

    # Initial state
    state = State(x=-0.0, y=5.0, yaw=np.radians(20.0), v=0.0)

    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        t_ = Twist()

        t_.linear.x = state.v * np.cos(state.yaw)
        t_.linear.y = state.v * np.sin(state.yaw)
        t_.linear.z = 0.0


        t_.angular.z = state.yaw
        t_.angular.x = 0.0
        t_.angular.y = 0.0

        publisher.publish(t_)
        
        time += dt
        a = 0
        b = 0
        x.append(state.x + random.uniform(a, b))
        y.append(state.y + random.uniform(a, b))
        yaw.append(state.yaw * random.uniform(a, b))
        v.append(state.v * random.uniform(a, b))
        t.append(time)

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  # pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


#  if __name__ == '__main__':
#      main()
