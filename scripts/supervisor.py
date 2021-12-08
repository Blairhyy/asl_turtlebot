#!/usr/bin/env python3
# <node pkg="asl_turtlebot" type="supervisor.py" name="turtlebot_supervisor" output="screen" />
from enum import Enum
from os import close
from pickle import REDUCE
from re import S
from time import sleep
import json
from get_path_TSP import superviser_get_path
from utils.TSP import get_path
from utils.object_processing import objects_clustering
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from std_msgs.msg import Float32MultiArray, String, MultiArrayDimension, MultiArrayLayout
from utils.grids import DilatedDetOccupancyGrid2D, StochOccupancyGrid2D
from collections import deque, namedtuple
from planners.P1_astar import AStar
from numpy import sin, cos, save, load, zeros, array
import tf
WINDOW_SIZE = 5
class Mode(Enum):
    """State machine modes. Feel free to change."""
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    RESCUE = 7
    RESCUE_PLAN = 8



class SupervisorParams:

    def __init__(self, verbose=False):
        # If sim is True (i.e. using gazebo), we want to subscribe to
        # /gazebo/model_states. Otherwise, we will use a TF lookup.
        self.use_gazebo = rospy.get_param("sim")

        # How is nav_cmd being decided -- human manually setting it, or rviz
        self.rviz = rospy.get_param("rviz")

        # If using gmapping, we will have a map frame. Otherwise, it will be odom frame.
        self.mapping = rospy.get_param("map")

        # Threshold at which we consider the robot at a location
        self.pos_eps = rospy.get_param("~pos_eps", 0.1)
        self.theta_eps = rospy.get_param("~theta_eps", 0.3)

        # Time to stop at a stop sign
        self.stop_time = rospy.get_param("~stop_time", 3.)

        # Minimum distance from a stop sign to obey it
        self.stop_min_dist = rospy.get_param("~stop_min_dist", 0.5)

        # Time taken to cross an intersection
        self.crossing_time = rospy.get_param("~crossing_time", 3.)

        if verbose:
            print("SupervisorParams:")
            print("    use_gazebo = {}".format(self.use_gazebo))
            print("    rviz = {}".format(self.rviz))
            print("    mapping = {}".format(self.mapping))
            print("    pos_eps, theta_eps = {}, {}".format(self.pos_eps, self.theta_eps))
            print("    stop_time, stop_min_dist, crossing_time = {}, {}, {}".format(self.stop_time, self.stop_min_dist, self.crossing_time))


class Supervisor:

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        self.params = SupervisorParams(verbose=True)
        self.fileName = "/home/group12/catkin_ws/src/asl_turtlebot/scripts/logs.npy"

        # Starting Position for Rescue
        self.x_r = None
        self.y_r = None
        self.theta_r = None

        # Current state
        self.x = 0
        self.y = 0
        self.theta = 0

        # Goal state
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0

        # Current mode
        self.mode = Mode.IDLE
        self.prev_mode = None  # For printing purposes

        # self.occupancy = None

        ########## PUBLISHERS ##########

        # Command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.cmd_nav_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)

        # Command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Command pose for initial state
        # self.ini_loc_publisher = rospy.Publisher('/ini_state', Pose2D, queue_size=10)

        # Command pose for selected object
        self.select_obj_pos_publisher = rospy.Publisher('/select_obj_pos', Float32MultiArray, queue_size=10)
        ########## SUBSCRIBERS ##########

        # Stop sign detector
        # rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)

        # High-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)

        # If using gazebo, we have access to perfect state
        if self.params.use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        self.trans_listener = tf.TransformListener()

        # If using rviz, we can subscribe to nav goal click
        if self.params.rviz:
            rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        else:
            self.x_g, self.y_g, self.theta_g = 1.5, -4., 0.
            self.mode = Mode.NAV

        # objects buffer
        self.objects_log = deque()
        self.objects_buffer = []
        rospy.Subscriber('/detector/objects', DetectedObjectList, callback = self.detectedObjectsList_callback)
        self.comment_pub = rospy.Publisher('/commentary', String, queue_size = 10)
        self.comments = {"banana": "In 1967, Andy Warhol designed one of the most iconic album covers of all time, featuring a simple yellow banana on the sleeve of The Velvet Underground's debut record. Warhol, undeterred by his lack of experience in the music industry, had become the band's manager two years prior and even introduced the German vocalist Nico to the group.\nOur Museum collected this piece of art from out best friend google image search. It is a precious masterpiece.", \
                    "cat": "Meowwwwww~~. Gary's cute cat JioJio. This photo is taken at his age of a few month. He was adopted from Champaign Humane Soceity in 2020 and is currently in Illinois living with his Mom. Next year, he will join Stanford as well and return to Gary.", \
                    "horse": "Neigh!!!!! XuBeihong adhered to realism for his whole life. His horse sketching drafts are no less than one thousand pieces. He studied horse anatomy, so he was quite familiar with horse's skeleton, muscles, organs, actions and looks. Different from traditional horse paintings which are of horses' front and side looks, this painting adopts a 3/4 side perspective and was completed with freehand brushwork method. The horse, with a strong body and flying pony mane, is running to the viewers, which is full of visual impact.",\
                    "stop_sign": "Stanford! Oh, wait it is a stop sign, the guardian of the intersections, protecting every and each member of Stanford Community. It is placed in the museum as a awareness raiser, calling bikers and scooters to stop at stop signs on campus to make commuting on campus safer.",\
                    "hot_dog": "The true spirit of baseball games. This hot dog is hand picked from thousands of pictures of hot dogs online as Blair's, our curator's, favorite hot dog on Google.",\
                    "person": "Girl with a Pearl Earring is Vermeer's most famous painting. It is not a portrait, but a 'tronie' - a painting of an imaginary figure. Tronies depict a certain type or character; in this case a girl in exotic dress, wearing an oriental turban and an improbably large pearl in her ear. \nJohannes Vermeer was the master of light. This is shown here in the softness of the girl's face and the glimmers of light on her moist lips. And of course, the shining pearl.",\
                    "bridge": "The iconic senic view of the Golden Gate Bridge."}

    ########## SUBSCRIBER CALLBACKS ##########
    def detectedObjectsList_callback(self, msg):
        self.objects_buffer = []
        if rospy.get_param('explore'):
            objects_labels = msg.objects
            objects_msgs = msg.ob_msgs
            for ob_label, ob_msg in zip(objects_labels, objects_msgs):
                self.objects_buffer.append([ob_label, *self.localize_object(ob_msg), self.x, self.y, self.theta])
    
    def localize_object(self, ob_msg):
        theta_b = (ob_msg.thetaleft + ob_msg.thetaright)/2
        field_angle = abs(ob_msg.thetaleft - ob_msg.thetaright)
        r_b = ob_msg.distance
        x = self.x + r_b*cos(self.theta + theta_b)
        y = self.y + r_b*sin(self.theta + theta_b)
        return (x, y, ob_msg.confidence, rospy.get_rostime().secs, field_angle, r_b)



    def gazebo_callback(self, msg):
        if "turtlebot3_burger" not in msg.name:
            return

        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]
        if self.x_r is None:
            self.x_r, self.y_r, self.theta_r = self.x, self.y, self.theta

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if self.params.mapping else "/odom"
        print("Rviz command received!")

        try:
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (nav_pose_origin.pose.orientation.x,
                          nav_pose_origin.pose.orientation.y,
                          nav_pose_origin.pose.orientation.z,
                          nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        print("Received rviz goal, switch to nav")
        self.mode = Mode.NAV

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < self.params.stop_min_dist and self.mode == Mode.NAV:
            self.init_stop_sign()


    ########## STATE MACHINE ACTIONS ##########

    ########## Code starts here ##########
    # Feel free to change the code here. You may or may not find these functions
    # useful. There is no single "correct implementation".
    


    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.cmd_nav_publisher.publish(nav_g_msg)
        # self.pose_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self, x, y, theta):
        """ checks if the robot is at a pose within some threshold """

        return abs(x - self.x) < self.params.pos_eps and \
               abs(y - self.y) < self.params.pos_eps and \
               abs(theta - self.theta) < self.params.theta_eps

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return self.mode == Mode.STOP and \
               rospy.get_rostime() - self.stop_sign_start > rospy.Duration.from_sec(self.params.stop_time)

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return self.mode == Mode.CROSS and \
               rospy.get_rostime() - self.cross_start > rospy.Duration.from_sec(self.params.crossing_time)

    ########## Code ends here ##########


    ########## STATE MACHINE LOOP ##########

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not self.params.use_gazebo:
            try:
                origin_frame = "/map" if self.params.mapping else "/odom"
                translation, rotation = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x, self.y = translation[0], translation[1]
                self.theta = tf.transformations.euler_from_quaternion(rotation)[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if self.prev_mode != self.mode:
            rospy.loginfo("Current mode: %s", self.mode)
            self.prev_mode = self.mode

        ########## Code starts here ##########
        # TODO: Currently the state machine will just go to the pose without stopping
        #       at the stop sign.
        if self.mode == Mode.IDLE:
            # Send zero velocity
            self.stay_idle()

        elif self.mode == Mode.POSE:
            # Moving towards a desired pose
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        # elif self.mode == Mode.STOP:
        #     # At a stop sign
        #     self.nav_to_pose()

        # elif self.mode == Mode.CROSS:
        #     # Crossing an intersection
        #     self.nav_to_pose()

        elif self.mode == Mode.NAV:
            if self.close_to(self.x_g, self.y_g, self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.nav_to_pose()
        elif self.mode == Mode.RESCUE_PLAN:
            """Stop the robot"""
            self.stay_idle()
            """Publish the current location as the initial state"""
            self.x_g = self.x_r
            self.y_g = self.y_r
            self.theta_g = self.theta_r

            """Get coordinates from the log file"""
            objects_list = objects_clustering(self.fileName)

            """TSP planning sequence"""
            Point = namedtuple('Point', ['x', 'y', 'rob', 'name'])
            p = Point(self.x_r, self.y_r, (self.x_r, self.y_r, self.theta_r), 'start')
            objects_list.insert(0, p)
            # distance_matrix = superviser_get_path(objects_list)
            # TSP_path = get_path(distance_matrix)
            
            map_msg = rospy.client.wait_for_message("/map", OccupancyGrid)
            map_metadata_msg = rospy.client.wait_for_message("/map_metadata", MapMetaData)
            rospy.loginfo("Received Map and Map metadata")
            map_width = map_metadata_msg.width
            map_height = map_metadata_msg.height
            map_resolution = map_metadata_msg.resolution
            map_origin = (map_metadata_msg.origin.position.x, map_metadata_msg.origin.position.y)
            map_probs = map_msg.data
            occupancy = StochOccupancyGrid2D(
                map_resolution,
                map_width,
                map_height,
                map_origin[0],
                map_origin[1],
                WINDOW_SIZE,
                map_probs,
            )
            state_min = -15, -15
            state_max = 15, 15
            def snap_to_grid(x):
                return (
                    round(x[0]),
                    round(x[1]),
            )
            # Input the selection from TA
            for i, object in enumerate(objects_list):
                if i>0:
                    print("Detected object #", i, " is", object.name)
            objects_select = [objects_list[0]]
            select_index = -1
            while select_index is not None:
                select_index = input("Select the object to secure: (press enter to end selection)")
                if select_index == '':
                    break
                select_index = int(select_index)
                if select_index >0 and select_index<=len(objects_list):
                    
                    print("Object successfully selected: ", objects_list[select_index].name)
                    objects_select.append(objects_list[select_index])
                else:
                    print("Selection failure, please select between 1 and ", len(objects_list)-1)
            distance_matrix = zeros((len(objects_select), len(objects_select)))
                    


            for i, start in enumerate(objects_select):
                for j, end in enumerate(objects_select[i:], i):
                    x_init = snap_to_grid((start.rob[0], start.rob[1]))
                    x_goal = snap_to_grid((end.rob[0], end.rob[1]))
                    problem = AStar(
                        state_min,
                        state_max,
                        x_init,
                        x_goal,
                        occupancy,
                        1,
                    )
                    success = problem.solve()
                    if success:
                        distance_matrix[i, j] = len(problem.path)
                        distance_matrix[j, i] = len(problem.path)
                        # Compute Manhattan
                        # distance_matrix[i, j] = abs(x_init[0] - x_goal[0]) + abs(x_init[1]-x_goal[1])
                        # distance_matrix[j, i] = distance_matrix[i, j]
                    else:
                        pass
            self.objects_list = objects_list
            self.objects_select = objects_select
            self.TSP_path = get_path(distance_matrix)

            self.mode = Mode.RESCUE

        elif self.mode == Mode.RESCUE:
            if not self.TSP_path:
                self.mode = Mode.RESCUE_PLAN
            else:
                layout = MultiArrayLayout()
                dim1 = MultiArrayDimension()
                dim1.size = len(self.objects_select)*2
                layout.dim = [dim1]
                sel_obj_msg = Float32MultiArray()
                mydata = []
                for object_select in self.objects_select:
                    mydata.append(object_select.x)
                    mydata.append(object_select.y)
                    mydata.append(object_select.rob[0])
                    mydata.append(object_select.rob[1])
                sel_obj_msg.data = mydata
                sel_obj_msg.layout = layout
                self.select_obj_pos_publisher.publish(sel_obj_msg)

                print([self.objects_select[i].name for i in self.TSP_path])
                for idx in self.TSP_path[0:]:
                    print(self.objects_select[idx].name, self.objects_select[idx].rob)
                    xg, yg, thetag = self.objects_select[idx].rob
                    self.x_g, self.y_g, self.theta_g = xg, yg, thetag
                    while not self.close_to(self.x_g, self.y_g, self.theta_g):
                        self.nav_to_pose()
                    if self.objects_select[idx].name in self.comments:
                        self.comment_pub.publish(self.comments[self.objects_select[idx].name])
                    sleep(10)
                self.mode = Mode.RESCUE_PLAN
        else:
            raise Exception("This mode is not supported: {}".format(str(self.mode)))

        ############ Code ends here ############

    def run(self):
        rate = rospy.Rate(10) # 10 Hz

        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()
            if not rospy.get_param("explore") and self.mode != Mode.RESCUE:
                self.mode = Mode.RESCUE_PLAN
                self.objects_log = load(self.fileName)
                # save(self.fileName, self.objects_log)
            
                
                # print(len(self.objects_log))
                # rospy.loginfo("Log saved to logs.npy")
            elif self.mode != Mode.RESCUE:
                # print(self.objects_buffer, self.objects_log)
                for log in self.objects_buffer:
                    # if not self.objects_log or self.objects_log[-1][-2] + 0 < log[-1]:
                    #     self.objects_log.append(log)
                    self.objects_log.append(log)

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
