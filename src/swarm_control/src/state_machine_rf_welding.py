#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PolygonStamped, Point32, TransformStamped, PoseStamped, Twist
import tf2_ros
import tf_conversions
import time
import threading
import math

# TODO: add in python files to CMakeLists

'''
state_machine_rf_welding.py
Alex Elias

Enables/disables robots when they move in a line
Robots get disabled when they get too close to the workspace boundary or each other
Robots get enabled when they get too far from the other enabled robots

Parameters:
    robot_desired_tf_frame_names: list of tf_frames (in tf topic) of robot desired positions,
        ordered "top" to "bottom"
    robot_order: list of numbers 0, 1, ..., N-1 of robots as they appear in swarm controller
        e.g. if 3 is in position 0, this means robot_desired_tf_frame_names[3]
        is the 0th robot in the swarm controller

    min_dist: minimum distance allowed robot-to-robot and robot-to-boundary
    max_dist: maximum distance allowed robot-to-robot

    workspace_frame_vel_topic_name:
    x_top_vel_topic_name:
    x_bottom_vel_topic_name:

    keyboard_vel_cmd_topic_name:
    robot_enable_topic_name:
    swarm_desired_vel_topic_name:

    swarm_tf: swarm frame(for syncing)
    robot_tf_frame_names: actual (fused) robot position tf_frames (for syncing) in the same order as robot_desired_tf_frame_names
    tf_changer_topic_name: 

'''

MAX_TIMESTEP = 0.1 # For velocity integration
WORKSPACE_HALF_WIDTH = 1.0 # m
DECEL_TIME = 0.4
SLEEP_TIME = 0.01

class State_machine():
    def __init__(self, N_robots, min_dist, max_dist):
        self.N_robots = N_robots
        self.state = 0
        self.min_dist = min_dist
        self.max_dist = max_dist

    def transition(self, x_top, x_bottom, x_robots, v_x):
        '''
        "top" robot is robot 0
        "bottom" robot is robot N-1
        coordinate system points from "bottom" to "top" 
        positive state: some "top" robots disabled
        negative state: some "bottom" robots disabled

        Inputs:
            x_top: position of top workspace boundary
            x_bottom: position bot bottom workspace boundary
            x_robots: list of robot positions
            v_x: velocity of swarm
        
        Outputs: 
            Boolean list of robot enable status (True = Enabled)
            Index of newly enable robot
            Index of removed robot
            State
        '''

        # rospy.logwarn("x_top:" + str(x_top) + " x_bottom: " + str(x_bottom) + " x_robots: " + str(x_robots) + " v_x:" + str(v_x))

        new_robot = None
        rem_robot = None

        if self.state == 0:
            # All robots are enabled
            # Just watch for top and bottom robots hitting boundary
            if x_top - x_robots[0] < self.min_dist and v_x > 0:
                self.state = 1
                rem_robot = 0
            elif x_robots[self.N_robots-1] - x_bottom < self.min_dist and v_x < 0:
                self.state = -1
                rem_robot = self.N_robots - 1;
        
        elif self.state == self.N_robots:
            # All robots are disabled
            # Waiting for "downward" velocity
            if v_x < 0:
                self.state -= 1
                new_robot = self.N_robots - 1
        
        elif self.state == -self.N_robots:
            # All robots are disabled
            # Waiting for "upwards" velocity
            if v_x > 0:
                self.state += 1 
                new_robot = 0
        
        elif self.state > 0:
            # Some top robots are disabled

            # Robot gets disabled if too close
            if x_robots[self.state-1] - x_robots[self.state] < self.min_dist and v_x > 0:
                self.state += 1
                rem_robot = self.state - 1

            # Robot gets enabled if too far
            elif x_robots[self.state-1] - x_robots[self.state] > self.max_dist and v_x < 0:
                self.state -= 1
                new_robot = self.state
        else: # self.state < 0
            # Some bottom robots are enabled

            # Robot gets disabled if too close
            if x_robots[self.N_robots+self.state-1] - x_robots[self.N_robots+self.state] < self.min_dist and v_x < 0:
                self.state -= 1
                rem_robot = self.N_robots - (-self.state)

            # Robot gets enabled if too far
            elif x_robots[self.N_robots+self.state-1] - x_robots[self.N_robots+self.state] > self.max_dist and v_x > 0:
                self.state += 1
                new_robot = self.N_robots - 1 + self.state
        
        if self.state > 0:
            status_arr =  [False] * self.state + [True] * (self.N_robots-self.state)
        else:
            status_arr = [True] * (self.N_robots-(-self.state)) + [False] * (-self.state)

        return status_arr, new_robot, rem_robot, self.state


class State_machine_ROS_node():
    def __init__(self):
        rospy.init_node('state_machine', anonymous=False)

        # Read in parameters
        self.robot_desired_tf_frame_names = rospy.get_param('~robot_desired_tf_frame_names')
        # rospy.logwarn(str(self.robot_desired_tf_frame_names))
        self.robot_order                  = rospy.get_param('~robot_order')
        min_dist                          = rospy.get_param('~min_dist')
        max_dist                          = rospy.get_param('~max_dist')

        workspace_frame_vel_topic_name = rospy.get_param('~workspace_frame_vel_topic_name') 
        x_top_vel_topic_name           = rospy.get_param('~x_top_vel_topic_name')
        x_bottom_vel_topic_name        = rospy.get_param('~x_bottom_vel_topic_name')

        keyboard_vel_cmd_topic_name    = rospy.get_param('~keyboard_vel_cmd_topic_name')
        robot_enable_topic_name        = rospy.get_param('~robot_enable_topic_name')
        swarm_desired_vel_topic_name   = rospy.get_param('~swarm_desired_vel_topic_name')

        self.swarm_tf                  = rospy.get_param('~swarm_tf')
        self.robot_tf_frame_names      = rospy.get_param('~robot_tf_frame_names')
        tf_changer_topic_name          = rospy.get_param('~tf_changer_topic_name')

        # Initialize
        self.N_robots = len(self.robot_desired_tf_frame_names)
        self.state_machine = State_machine(self.N_robots, min_dist, max_dist)
        self.x_robots = [0.0] * self.N_robots

        self.last_timestep_requests = {}

        self.workspace_outline_seq = 0

        self.x_top = 2.0 # m
        self.x_bottom = -2.0 # m
        self.workspace_center = [0.0, 0.0, 0.0] # [m m rad]
        self.status_array = [True] * self.N_robots

        self.state_lock = threading.Lock()

        # Publish
        self.enable_pub            = rospy.Publisher(robot_enable_topic_name,        Int32,          queue_size=10)
        self.workspace_outline_pub = rospy.Publisher("workspace_outline",            PolygonStamped, queue_size=10)
        self.tf_changer_pub        = rospy.Publisher(tf_changer_topic_name,          PoseStamped,    queue_size=10)
        self.swarm_desired_vel_pub = rospy.Publisher(swarm_desired_vel_topic_name,   Twist,          queue_size=1 )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


        # Subscribe
        rospy.Subscriber(keyboard_vel_cmd_topic_name,       Twist, self.keyboard_vel_callback,        queue_size=1)
        rospy.Subscriber(workspace_frame_vel_topic_name,    Twist, self.workspace_frame_vel_callback, queue_size=1)
        rospy.Subscriber(x_top_vel_topic_name,              Twist, self.x_top_vel_callback,           queue_size=1)
        rospy.Subscriber(x_bottom_vel_topic_name,           Twist, self.x_bottom_vel_callback,        queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Timers
        rospy.Timer(rospy.Duration(0.01), self.publish_frames_callback)

        # Enable all robots when starting
        time.sleep(0.5)
        self.send_enable_status([True] * self.N_robots)

    def send_enable_status(self, status_arr):
        msg = Int32()
        for i in range(self.N_robots):
            if not status_arr[i]:
                msg.data += pow(2, self.robot_order[i])
        self.enable_pub.publish(msg)
        rospy.logwarn(str(status_arr) + " " + str(msg.data))

    def send_workspace_outline(self):
        msg = PolygonStamped()
        self.workspace_outline_seq += 1;

        msg.header.seq = self.workspace_outline_seq
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "workspace"

        msg.polygon.points = [Point32(x = self.x_top,    y =  WORKSPACE_HALF_WIDTH, z = 0.0), # top left
                              Point32(x = self.x_top,    y = -WORKSPACE_HALF_WIDTH, z = 0.0), # top right
                              Point32(x = self.x_bottom, y = -WORKSPACE_HALF_WIDTH, z = 0.0), # bot right
                              Point32(x = self.x_bottom, y =  WORKSPACE_HALF_WIDTH, z = 0.0)] # bot left

        self.workspace_outline_pub.publish(msg)

    def send_workspace_frame(self):
        msg = xyt2TF(self.workspace_center, "map", "workspace")
        self.tf_broadcaster.sendTransform(msg)

    def keyboard_vel_callback(self, data):
        with self.state_lock:
            v_x = data.linear.x; # TODO: what frame is the keyboard velocity in?

            # Find the robot x locations in the workspace frame
            for i in range(self.N_robots):
                if self.status_array[i]:
                    #t = self.tf_listener.getLatestCommonTime("workspace",self.robot_deesired_tf_frame_names[i],)
                    try:
                        trans = self.tf_buffer.lookup_transform("workspace",self.robot_desired_tf_frame_names[i], rospy.Time(0))
                        self.x_robots[i] = trans.transform.translation.x
                    except:
                        rospy.logwarn("Couldn't find transform for robot " + str(i))

            self.status_array, new_robot, rem_robot, state = self.state_machine.transition(self.x_top, self.x_bottom, self.x_robots, v_x)

            if new_robot is not None:
                rospy.logwarn("Starting to add robot " + str(new_robot) + ", new state is " + str(state))
                # 1. Send a zero command to slow down the swarm (and wait)
                t_0 = time.time()
                msg_0 = Twist()
                while(time.time() - t_0 < DECEL_TIME):
                    self.swarm_desired_vel_pub.publish(msg_0)
                    time.sleep(0.01)
                # 3. Add new robot (and wait)
                self.send_enable_status(self.status_array)
                time.sleep(SLEEP_TIME)
                # 4. Sync new robot (and wait)
                self.sync_robot(new_robot)
                time.sleep(SLEEP_TIME)
                rospy.logwarn("Done adding robot " + str(new_robot))

            if rem_robot is not None:
                rospy.logwarn("Starting to remove robot " + str(rem_robot) + ", new state is " + str(state))
                self.send_enable_status(self.status_array)
                time.sleep(SLEEP_TIME)
                rospy.logwarn("Done removing robot " + str(rem_robot))


            # Rotate velocity command to be aligned with workspace frame
            msg = Twist()
            c, s = math.cos(self.workspace_center[2]), math.sin(self.workspace_center[2])
            msg.linear.x = c * data.linear.x - s * data.linear.y
            msg.linear.y = s * data.linear.x + c * data.linear.y
            self.swarm_desired_vel_pub.publish(msg)
            # rospy.logwarn("state: " + str(state) + " enabled robots: " + str(status_array) + " new robot:" + str(new_robot))

    def sync_robot(self, n_robot):
        #if self.tf.frameExists(self.swarm_tf) and self.tf.frameExists(self.robot_tf_frame_names[n_robot]):
        #t = self.tf.getLatestCommonTime(self.robot_tf_frame_names[n_robot], self.swarm_tf)
        try:
            trans = self.tf_buffer.lookup_transform(self.swarm_tf,self.robot_tf_frame_names[n_robot],rospy.Time(0))
            p1 = PoseStamped()
            p1.header.frame_id = self.robot_desired_tf_frame_names[n_robot]
            
            p1.pose.position = trans.transform.translation
            p1.pose.orientation = trans.transform.rotation

            self.tf_changer_pub.publish(p1)
        except:
            rospy.logerr("Couldn't sync robot " + str(n_robot))

    def get_timestep(self, integrator_name):
        current_time = time.time()
        if integrator_name in self.last_timestep_requests:
            dt = current_time - self.last_timestep_requests[integrator_name]
            self.last_timestep_requests[integrator_name] = current_time
            if dt > MAX_TIMESTEP:
                dt = 0.0
            return dt
        else:
            self.last_timestep_requests[integrator_name] = current_time
            return 0.0

    def workspace_frame_vel_callback(self, data):
        dt = self.get_timestep("workspace_frame_vel")
        self.workspace_center[0] += data.linear.x * dt
        self.workspace_center[1] += data.linear.y * dt
        self.workspace_center[2] += data.angular.z * dt

    def x_top_vel_callback(self, data):
        dt = self.get_timestep("x_top_vel")
        self.x_top += data.linear.x * dt

    def x_bottom_vel_callback(self, data):
        dt = self.get_timestep("x_bottom_vel")
        self.x_bottom += data.linear.x * dt

    def publish_frames_callback(self, event):
        self.send_workspace_frame()
        self.send_workspace_outline()


def xyt2TF(xyt, header_frame_id, child_frame_id):
    '''
    Converts a list [x y theta]
    into a tf2_msgs.msg.TFMessage message
    '''
    t = TransformStamped()

    t.header.frame_id = header_frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = xyt[0]
    t.transform.translation.y = xyt[1]
    t.transform.translation.z = 0

    q = tf_conversions.transformations.quaternion_from_euler(0, 0,xyt[2])
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    return t

if __name__ == '__main__':
    node = State_machine_ROS_node()
    rospy.spin()    