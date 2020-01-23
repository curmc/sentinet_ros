import rospy
from kermit.common import *
from tf.transformations import euler_from_quaternion

class OccupancyMap():

    def __init__(self, num_rows: int, num_cols: int, x = 0.0, y = 0.0, yaw = 0.0):
        self.num_cols = num_cols
        self.num_rows = num_rows
        self.omap = [[0.0 for i in range(len(num_cols))] for j in range(len(num_cols))]
        

        # Keep track of x and y position
        # for a sensor update
        self.x = x
        self.y = y
        self.yaw = yaw

        self.pos_sub = rospy.Subscriber(topics["path_state_topic"], path_state, self.pos_callback)
        self.pos_sub = rospy.Subscriber(topics["tof_ping_topic"], path_state, self.pos_callback)

    def pos_callback(self, p_state):
        self.x = p_state.pose.position.x
        self.y = p_state.pose.position.y
        
        orientation_q = p_state.orientation
        o_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.yaw = euler_from_quaternion(o_list)

