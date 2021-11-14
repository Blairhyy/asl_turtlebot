# import numpy as np
# from utils import wrapToPi

# # command zero velocities once we are this close to the goal
# RHO_THRES = 0.05
# ALPHA_THRES = 0.1
# DELTA_THRES = 0.1

# class PoseController:
#     """ Pose stabilization controller """
#     def __init__(self, k1, k2, k3, V_max=0.5, om_max=1):
#         self.k1 = k1
#         self.k2 = k2
#         self.k3 = k3

#         self.V_max = V_max
#         self.om_max = om_max

#     def load_goal(self, x_g, y_g, th_g):
#         """ Loads in a new goal position """
#         self.x_g = x_g
#         self.y_g = y_g
#         self.th_g = th_g

#     def compute_control(self, x, y, th, t):
#         """
#         Inputs:
#             x,y,th: Current state
#             t: Current time (you shouldn't need to use this)
#         Outputs: 
#             V, om: Control actions

#         Hints: You'll need to use the wrapToPi function. The np.sinc function
#         may also be useful, look up its documentation
#         """
#         ########## Code starts here ##########
#         rho = np.linalg.norm([self.x_g - x, self.y_g -  y])
#         alpha = wrapToPi(np.arctan2(self.y_g - y, self.x_g - x) - th)
#         delta = wrapToPi(np.arctan2(self.y_g - y, self.x_g - x) - self.th_g)
#         V = self.k1 * rho * np.cos(alpha)
#         if abs(alpha) < 0.01:
#             om = self.k2 * alpha + self.k1 * np.sinc(alpha/np.pi)* np.cos(alpha) * (alpha + self.k3 * delta)
#         else:
#             om = self.k2 * alpha + self.k1 * np.sin(alpha) / alpha * np.cos(alpha) * (alpha + self.k3 * delta)
#         ########## Code ends here ##########

#         # apply control limits
#         V = np.clip(V, -self.V_max, self.V_max)
#         om = np.clip(om, -self.om_max, self.om_max)
#         return V, om
import numpy as np
from utils import wrapToPi
import math
import rospy
from std_msgs.msg import Float32

# command zero velocities once we are this close to the goal
RHO_THRES = 0.05
ALPHA_THRES = 0.1
DELTA_THRES = 0.1

class PoseController:
    """ Pose stabilization controller """
    def __init__(self, k1, k2, k3, V_max=0.5, om_max=1):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.x_g = None
        self.y_g = None
        self.th_g = None

        self.V_max = V_max
        self.om_max = om_max
        # rospy.init_node('PoseController',anonymous=True)
        # self.rate = rospy
        
        self.alpha_pub = rospy.Publisher('/controller/alpha', Float32, queue_size=10)
        self.delta_pub = rospy.Publisher('/controller/delta', Float32, queue_size=10)
        self.rho_pub = rospy.Publisher('/controller/rho', Float32, queue_size=10)

    def load_goal(self, x_g, y_g, th_g):
        """ Loads in a new goal position """
        self.x_g = x_g
        self.y_g = y_g
        self.th_g = th_g

    def compute_control(self, x, y, th, t):
        """
        Inputs:
            x,y,th: Current state
            t: Current time (you shouldn't need to use this)
        Outputs: 
            V, om: Control actions

        Hints: You'll need to use the wrapToPi function. The np.sinc function
        may also be useful, look up its documentation
        """
        ########## Code starts here ##########
        rho = math.sqrt((self.x_g - x)**2 + (self.y_g - y)**2)
        alpha = wrapToPi(np.arctan2(self.y_g - y, self.x_g - x) - th)
        delta = wrapToPi(np.arctan2(self.y_g - y, self.x_g - x) - self.th_g)

        V = self.k1 * rho * np.cos(alpha)

        if abs(alpha) < ALPHA_THRES: 
            om = self.k2 * alpha + self.k1 * np.sinc(alpha) * np.cos(alpha) * (alpha + self.k3 * delta)
        else:
            om = self.k2 * alpha + self.k1 * np.sin(alpha) * np.cos(alpha) * (alpha + self.k3 * delta) / alpha
        
        alpha_msg = Float32()
        alpha_msg.data = alpha
        self.alpha_pub.publish(alpha_msg)

        delta_msg = Float32()
        delta_msg.data = delta
        self.delta_pub.publish(delta_msg)

        rho_msg = Float32()
        rho_msg.data = rho
        self.rho_pub.publish(rho_msg)
        ########## Code ends here ##########

        # apply control limits
        V = np.clip(V, -self.V_max, self.V_max)
        om = np.clip(om, -self.om_max, self.om_max)

        return V, om