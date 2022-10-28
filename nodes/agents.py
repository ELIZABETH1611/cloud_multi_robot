#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
import math
import random
import json
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import String, Float32MultiArray, Float64MultiArray
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from pathfinding import pathfinding
from reinforcenment import ReinforcementNetwork
from rooms import Check_room
from target import Target
from math import pi
import numpy
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from environment import Behaviour

class Agent(object):
    ""
    ""

    def __init__(self,agent_name,action_size,state_size,number_episode,rank, min_action_time=0.25):
        self.rank = rank

        # self.dirPath            = os.path.dirname(os.path.realpath(__file__))
        self.dirPath            = "/home/mcg/catkin_ws/src/multi_robot/save_model/environment"
        # self.dirPath            = self.dirPath.replace('multi_robot/nodes', 'multi_robot/save_model/environment_')
        self.agent_name                 = agent_name
        # Save the data laser to avoid asking ros all the time
        self.__step_cache_1               = -1
        self.step                         = 0
        self.force_update_1               = False
        self.last_heading                 = []
        self.heading_1                    = 0
        self.heading_r2_r1                = 0
        self.theta_1= 0
        self.__status                     = "follower"
        self.reward                       =0
        self.load                         =random.choice([False,False,False])
        self.step                         = 0
        self.__process                  = "driving_to_goal"
        self.number_action              = action_size
        self.max_angular_vel            = 1
        self.__min_action_time          = min_action_time
        self.__angle_actions            = np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.__min_action_time for action in range(action_size-1)])
        self.old_goal                   = (np.nan,np.nan)
        self.__forward_action           = int((action_size-2)/2.)
        self.__backward_action          = int(action_size-1)
        self.min_range                  = 0.18
        # Avoid countinf multiple crashes en the same step
        self.__crashed                  = False
        self.__crash_counter            = 0
        self.score                      = 0
        self.global_step                = 0
        self.cont                       = 0
        self.__avoid_distance           = 1
        self.__action_time              = 0

        self.diff_time                  = 0.45
        self.done                       = False
        self.finish                     = False
        self.__current_leader           = 0  # number of the rank who is the leader
        self.__ID                       = 0
        self.old_ID                     = 0
        self.room                       = Check_room()


    def call_sub(self,agent_name,action_size,state_size,number_episode,rank):
        self.unpause_proxy                = rospy.ServiceProxy('gazebo/unpause_physics', Empty)
        self.pause_proxy                  = rospy.ServiceProxy('gazebo/pause_physics', Empty)
        self.pub_cmd_vel                  = rospy.Publisher(self.agent_name+'/cmd_vel', Twist, queue_size=5)
        self.environment                  = Behaviour(agent_name,self.ID,rank)
        self.sub_odom_1                   = rospy.Subscriber(self.agent_name+'/odom', Odometry, self.get_Odometry_1)
        self.robot_position_i           = Pose()
        self.robot_position_y           = 0
        self.robot_position_x           = 0
        self.pub_scan                     = rospy.Publisher(self.agent_name+'/scan_out', Float64MultiArray, queue_size=24)
        self.__pathfinding                = pathfinding(self.laser_angles/360.*2*np.pi)
        self.learning                     = ReinforcementNetwork(state_size,action_size,number_episode,self.load)
        self.start_time                   = time.time()
        self.learning.get_Pa()
        self.state_initial                = None


    @property
    def ID(self):
        return self.__ID

    @ID.setter
    def ID(self,value):
        self.__ID = value
        self.environment.ID=value
        self.environment.target_position.ID=value

    @property
    def current_leader(self):
        return self.__current_leader

    @current_leader.setter
    def current_leader(self,value):
        self.__current_leader = value
        if value==self.rank:
            self.status="lead"
        else:
            self.status="follower"

    @property
    def get_initial_status(self):
        self.state_initial,_ = np.array(self.state)
        return self.state_initial

    @property
    def status(self):
        return self.__status

    @status.setter
    def status(self,value):
        self.__status = value

    def get_Odometry_1(self, odom):
        '''
        Position and orientation to the robot
        '''
        self.robot_position_i = odom.pose.pose.position
        self.robot_position_x =self.robot_position_i.x
        self.robot_position_y =self.robot_position_i.y
        robot_1_angle = odom.pose.pose.orientation
        angles_robot_list_1 = [robot_1_angle.x, robot_1_angle.y, robot_1_angle.z, robot_1_angle.w]
        _, _, yaw = euler_from_quaternion(angles_robot_list_1)
        self.theta_1 = yaw
        self.environment.goal_angle = math.atan2(self.environment.goal_y - self.robot_position_y, self.environment.goal_x - self.robot_position_x)
        self.heading_1 = self.environment.goal_angle - yaw

        if self.heading_1 > pi:
            self.heading_1 -= 2 * pi
        elif self.heading_1 < -pi:
            self.heading_1 += 2 * pi
        self.last_heading.append(self.heading_1)
        # print("last heading "+str(self.agent_name)+": ", self.heading_1)


    @property
    def scan_data(self):
        '''
        Get data of the laser
        '''
        if self.__step_cache_1 == self.step and not self.force_update_1:
            scan_data=self.__scan_data_cache_1
        else:
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message(self.agent_name+'/scan', LaserScan, timeout=5)
                except:
                    pass
            scan = data
            scan_data = []
            for i in range(len(scan.ranges)):
                if scan.ranges[i] == float("Inf"):
                    scan_data.append(3.5)
                elif np.isnan(scan.ranges[i]):
                    scan_data.append(0)
                else:
                    scan_data.append(scan.ranges[i])
            if np.any(np.isnan(np.array(scan_data))):
                raise Exception("it's nan sensor")

            self.__step_cache_1     = self.step
            self.__scan_data_cache_1 = scan_data
            self.force_update_1      = False
            scan_data2 = Float64MultiArray()
            scan_data12=np.array(scan_data)
            scan_data2.data = scan_data12
            self.pub_scan.publish(scan_data2)

        return scan_data

    def perform_action(self,action):
        '''
        Calculates the time needed to excecute the actions
        and executes them
        '''
        self.diff_time = time.time()-self.__action_time
        self.__action_time = time.time()
        max_angular_vel = 1.5
        if action != self.__backward_action:
            ang_vel = ((self.number_action - 1)/2 - action) * max_angular_vel * 0.5

        vel_cmd = Twist()


        if action == self.__backward_action:
            vel_cmd.linear.x = -0.15
            vel_cmd.angular.z = 0

        elif action == self.__forward_action:
            vel_cmd.linear.x = 0.15
            vel_cmd.angular.z = 0
            self.stand=False

        else:
            vel_cmd.linear.x = 0.12
            vel_cmd.angular.z = ang_vel

            self.vel_cmd=vel_cmd
        self.pub_cmd_vel.publish(vel_cmd)


    def reset(self):
        """
          Reset the robot
        """
        self.__process      = "driving_to_goal"
        self.__free_counter = 0
        self.__coll_count   = 0


    @property
    def position(self):
        return (self.robot_position_x, self.robot_position_y)

    @property
    def process(self):
        return self.__process

    @process.setter
    def process(self,value):
        self.__process = value

    @property
    def angle_action(self):
        # return np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.diff_time for action in range(self.number_action-1)])
        return np.array([((self.number_action-4-action)* self.max_angular_vel*0.5)*self.diff_time for action in range(self.number_action-1)])

    @property
    def laser_angles(self):
        """
          Returns the angles of the laser
        """
        scan_data = self.scan_data
        angles = 360./(len(scan_data)-1)*np.arange(len(scan_data))
        angles[angles>180] = angles[angles>180]-360
        return angles

    def rotate_to_angle(self):
        """
          Rotate to a given angle
        """
        eta=0
        # aa=self.__angle_actions+eta*self.__angle_actions
        aa=self.angle_action+eta*self.angle_action
        diff_angles = self.heading_1-self.__desired_angle
        if diff_angles<0:
            mask   = ((diff_angles-aa)<0)
        elif diff_angles>=0:
            mask   = (diff_angles-aa)>=0
        if abs(diff_angles) <np.deg2rad(10):
            helper = np.argmin(abs(diff_angles-aa[mask]))
            mask2=np.ones(len(aa),dtype=bool)
        else:
            mask2 =( np.arange(len(aa))==1 )| (np.arange(len(aa))==3)
            helper = np.argmin(abs(diff_angles-aa[mask&mask2]))
        action = np.arange(len(aa))[mask&mask2][helper]
        # print("change_angle",action,self.heading-self.__desired_angle)
        return action, (action==2)

    @property
    def free_dist_goal(self):
        """
        Calculates the free distance to the goal, using laser information
        """
        scan_data = np.array(self.scan_data)
        sortkey = np.argsort(abs(np.rad2deg(self.heading_1)-self.laser_angles))[0:3]

        return np.min(scan_data[sortkey])

    @property
    def status_regions(self):
        scan_data =self.scan_data
        regions = {
        'right':  min(scan_data[16:20]),
        'sright':  max(scan_data[15:19]),
        'fright': min(scan_data[18:22]),
        'front':  min(min(scan_data[21:25]), min(scan_data[0:4])),
        'fleft':  min(scan_data[2:6]),
        'left':   min(scan_data[5:9]),
        'sleft':   max(scan_data[5:9]),
        'backl':  scan_data[11],
        'backr':  scan_data[13],
        'back':  scan_data[12],
        'stop': min(min(scan_data[21:25]),min(scan_data[0:3])) }
        return regions

    def change_process(self, finished):
        """
          Change the optimal process to reach the goal
        """
        # print("i am in change_process")

        free_dist = self.free_dist_goal
        goal_dist = self.environment.get_current_Distance(self.robot_position_x,self.robot_position_y)
        if finished:
            self.__process = "follow_path"
        elif self.__process=="follow_path":
            # Only drive to goal if the distance to the goal is okay and not blocked
            if (free_dist>goal_dist):
                self.__desired_angle = self.__find_good_angle()
                self.__process="driving_to_goal"
        elif self.__process=="driving_to_goal":
            if self.status_regions["front"]<=self.__avoid_distance/1.5:
                self.__process="follow_path"
        elif self.__process=="collision":
            if self.__coll_count >= 15:
                self.__process="follow_path"


    def __find_good_angle(self):
            '''
            Look for an obstacle free angle
            '''
            scan         = np.array(self.scan_data)
            laser_angles = np.array(self.laser_angles)
            mask         = scan>self.__avoid_distance
            indices      = np.arange(len(laser_angles))
            ii_g         = np.argsort(abs((np.deg2rad(laser_angles[mask])-self.heading_1)))[0]
            idx          = indices[mask][ii_g]
            towards_goal = np.deg2rad(laser_angles[idx])

            res = towards_goal
            if idx+1>=len(scan):
                nidx = -1
            else:
                nidx = idx+1

            if scan[idx-1]>scan[nidx]:
                res = res+np.deg2rad(laser_angles[idx]-laser_angles[idx-1])
            else:
                res = res+np.deg2rad(laser_angles[idx]-laser_angles[nidx])
            res=self.environment.fix_angle(res-self.heading_1)

            return res


    def evolve(self):
        """
          Make one step with the robot
        """
        scan = np.array(self.scan_data)
        # Update the map, probably not necessary every step
        if (self.step % 1 == 0) and (self.step!=0):
            self.__pathfinding.update_map(self.position,self.environment.target_position.position,self.heading_1,scan)
        # Construct a path
        if (self.old_goal != self.environment.target_position.position) or (self.step % 5 == 0) and (self.__process == "follow_path"):
            self.__pathfinding.construct_path(self.position,self.environment.target_position.position)

        # Finish the actual __process
        finished = False
        if self.__process=="collision":
            laser_angles = np.array(self.laser_angles)
            #laser_angles expressed in degrees
            if abs(laser_angles[np.argmin(scan)])>90:
                action = self.__forward_action
            else:
                action = self.__backward_action
            self.__coll_count +=1
        elif self.__process == "follow_path":
            self.__desired_angle = self.__pathfinding.follow_path(self.position,self.environment.target_position.position)
            action,_  = self.rotate_to_angle()
        elif self.__process== "driving_to_goal":
            self.__desired_angle = 0
            action,_  = self.rotate_to_angle()
        elif self.__process == "change_angle":
            action,finished = self.rotate_to_angle()

        # For debugging
        self.__pathfinding.monitor(self.position,self.environment.target_position.position)
        # Change the process
        self.change_process(finished)
        self.old_goal = self.environment.target_position.position
        self.action = action
        return self.action


    @property
    def state(self):
        '''
        Get state of the robot goal_angle heading,  scan_data,current_distance
        '''
        current_distance= self.environment.get_current_Distance(self.robot_position_x,self.robot_position_y)
        heading = self.heading_1
        scan_data = self.scan_data
        if ((self.min_range >= min(scan_data) > 0) and (not self.__crashed)) \
            or ((self.min_range >= min(scan_data) > 0) and (self.__crash_counter>5))  :
            print("collision position "+str(self.agent_name)+": ", min(scan_data),self.min_range )
            sys.stdout.flush()

            self.done = True
            self.__crashed = True
            self.__crash_counter = 0
            self.__process = "collision"
            self.__coll_count = 0
        elif (self.min_range >= min(scan_data) > 0) and (self.__crashed):
             self.__crash_counter += 1
        elif (self.min_range < min(scan_data)):
             self.__crashed = False
        wall_dist = min(self.scan_data)
        obstacle_angle = np.argmin(self.scan_data)

        goal_heading_initial= self.last_heading[0]

        return scan_data + [heading, current_distance,wall_dist,goal_heading_initial], self.done

    def next_values(self):
        '''
        Call reward function and return next_state, reward,done
        '''
        # done                = self.done
        state, done         = self.state
        # print("next_values ",done)
        state               = np.array(state)
        self.reward         = self.environment.set_reward(state, done, self.action,self.step)
        next_state          = np.array(state)
        next_state[0:24]    = next_state[0:24]/3.5
        # next_state[24]      = (next_state[24]+np.pi)/(2*np.pi)
        next_state[24]      = next_state[24]
        next_state[25]      = next_state[25]/self.environment._goal_distance_initial
        next_state[26]      = next_state[26]/3.5
        next_state[27]      = next_state[27]
        # next_state[27]      = (next_state[27]+np.pi)/(2*np.pi)

        self.next_state     = next_state
        self.done           = done
        return self.next_state

    def get_action_value(self):
        # state, _         = np.array(self.state)
        self.action, self.evolve_rule = self.learning.get_action(np.array(self.state_initial))

    def save_data(self,rank,step,e):
        if not os.path.exists(self.dirPath +"_value_agent_"+str(rank)+'.txt'):
            with open(self.dirPath+"_value_agent_"+str(rank)+'.txt', 'a') as outfile:
                outfile.write("step".rjust(8," ")+ "   "+"episode".rjust(8," ")\
                + "   "+"a".rjust(1," ")+"   "+"   "+"reward".rjust(10," ")\
                +"   "+"score".rjust(10," ")+"   "+"robot_x".rjust(10," ")\
                +"   "+"robot_y".rjust(10," ")+"  "+"goal_x".rjust(10," ")\
                +"   "+"goal_y".rjust(10," ") +"   " +"e_r".rjust(1," ")\
                +"   "+"q_value".rjust(8," ")+"   "+"b_time".rjust(10," ")\
                +"   " +"win".rjust(4," ")+"   " +"fail".rjust(4," ")\
                +"   " +"Pa".rjust(10," ")\
                +"   "+"t_h".rjust(2," ")+"   "+"t_m".rjust(2," ")\
                +"   "+"t_s".rjust(2," ")+"   "+"state".rjust(150," ")\
                +"   "+"next_state".rjust(200," ")+"\n")
        m, s = divmod(int(time.time() - self.start_time), 60)
        h, m = divmod(m, 60)
        with open(self.dirPath +"_value_agent_"+str(rank)+'.txt', 'a') as outfile:
            outfile.write("{:8d}".format(step)+"   "+"{:8d}".format(e)\
            +"   "+str(self.action)+"   "+"   "+"{: .3e}".format(self.reward)\
            +"   "+"{: .3e}".format(self.score)+"   "+"{: .3e}".format(self.robot_position_x)\
            +"   "+"{: .3e}".format(self.robot_position_y)+"   "+"{: .3e}".format(self.environment.goal_x) \
            +"   "+"{: .3e}".format(self.environment.goal_y) +"   " +str(int(self.evolve_rule))\
            +"   "+"{: .3e}".format(np.max(self.learning.q_value))+"   "+"{: .3e}".format(self.environment.best_time)\
            +"   " +str(int(self.environment.get_goalbox))+"   " +str(int(self.done))\
            +"   "+"{: .3e}".format(self.learning.Pa)\
            +"   "+"{:8d}".format(h)+"   "+"{:02d}".format(m) \
            +"   "+"   "+"{:02d}".format(s) +"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.state_initial))\
            +"   "+"   "+' '.join(map(lambda x: "{: .6f}".format(x), self.next_state))+"\n")


    def append_memory(self):
        self.learning.append_D(self.state_initial, self.action, self.reward, self.next_state, self.done)
        self.learning.append_EPS(self.state_initial, self.action, self.reward, self.next_state, self.done)
        self.state_initial  = self.next_state
        self.score += self.reward

    def work_out_best_state(self):
        if self.environment.get_goalbox == True:
            rospy.loginfo("goal achieved by  %s",self.agent_name)
            sys.stdout.flush()

            if self.environment.best_time > 0.85:
                self.learning.best_state()
            else:
                self.learning.winning_state()
            self.keep_or_reset()
        else:
            pass

    def keep_or_reset(self):
        if self.learning.Pa < self.learning.normal_process:
            # Calculate the distance to the target from the agent's last position
            self.environment.get_goalbox = False
            self.environment.goal_x, self.environment.goal_y = self.environment.target_position.getPosition(True, delete=True)
            self.last_heading = []
            self.environment.get_Distance_goal(self.robot_position_x,self.robot_position_y)

        else:
            # Returns to the agent's origin position and calculates the distance to the target
            self.environment.get_goalbox = False
            self.reset()
            self.environment.reset_gazebo()
            # self.environment.reset(self.robot_position_x, self.robot_position_y)

            self.last_heading=[]

            self.environment.goal_x, self.environment.goal_y = self.environment.target_position.getPosition(True, delete=True)
            self.environment.get_Distance_goal(self.robot_position_x,self.robot_position_y)


    def update_variables(self):
        self.score += self.reward
        self.state = self.next_state
        self.global_step += 1

    def Done(self,done,finish):
        self.learning.update_target_network()
        if finish:
            finish = False
            out= "break"
        if self.evolve_rule:
            self.process="collision"
            self.cont+=1
            self.learning.increase_fact()
            self.last_heading=[]
            self.environment.reset(self.robot_position_x,self.robot_position_y)
            if self.cont > 20:
                self.cont=0
                out= "break"
        else:
            out= "break"
        return out


    def time_to_update(self,q_model_theta,target_model_theta):
        # self.global_step += 1
        # if self.global_step % self.learning.target_update == 0:
        self.learning.update_target_network(q_model_theta,target_model_theta)

    def time_out(self,step):
        if step >= 700:
            rospy.loginfo("Time out!! of %s", self.agent_name )
            # self.done = True
            return True
        else:
            return False

    def check_room(self):
        self.old_ID = self.ID
        self.ID=self.room.check_room(self.robot_position_x,self.robot_position_y)
