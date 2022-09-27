#!/usr/bin/env python

"""
=========================================================================================================
This file contains class AutonomousAgent containing methods for implementation
of BezierTurn and BezierOverTake trajectory planning. Finite State Machine is described for 
behavior planning.  

Platform: Windows

Author:: Aditya Jagani
Last updated: 09/24/2022
=========================================================================================================
"""

import pygame
import carla
import math
import numpy as np

from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.lane_change import BezierTurn, BezierOverTake
from agents.tools.misc import transform_to_frame


class AutonomousAgent(Agent):
    """
    BasicAgent implements a basic agent that navigates scenes to reach a given
    target destination. This agent respects traffic lights and other vehicles.
    """

    def __init__(self, ego):
        """
        :param ego: ego to apply to local planner logic onto
        """
        super(AutonomousAgent, self).__init__(ego.player)
        self._world_obj = ego

        self._THW = 2
        self._target_speed = None

        # Local plannar
        self._local_planner = LocalPlanner(ego.player)
        self.update_parameters()
        
        # Global plannar
        self._proximity_threshold = 10.0  # meter   # Distance between waypoints
        self._state = AgentState.NAVIGATING
        self._hop_resolution = 0.2
        self._path_seperation_hop = 2
        self._path_seperation_threshold = 0.5
        self._grp = None  # global route planar
        
        # Behavior planning
        self._hazard_detected = False
        self._blocked_time = None
        self._perform_lane_change = False
        self._front_r = []
        self._left_front_r = []
        self._left_back_r = []
        
        # Turns positions
        self.right_positions = None
        self.left_positions = None

        # Turn flags
        self.right_turn = False
        self.left_turn = False
        self.temp_flag = True
        self.left_positions = None

    def update_parameters(self):
        self._THW = 2
        self._target_speed = 30

        CONTROLLER_TYPE = 'PID' # options: MPC, PID, STANLEY
        args_lateral_dict = {'K_P': 1.0, 'K_I': 0.4, 'K_D': 0.01, 'control_type': CONTROLLER_TYPE}
        args_longitudinal_dict = {'K_P': 0.3, 'K_I': 0.2, 'K_D': 0.002}
        self._local_planner.init_controller(opt_dict={'target_speed': self._target_speed,
                                                      'lateral_control_dict': args_lateral_dict,
                                                      'longitudinal_control_dict': args_longitudinal_dict})


    
    def set_destination(self, location):
        """
        This method creates a list of waypoints from agent's position to destination location
        based on the route returned by the global router
        """
        start_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        end_waypoint = self._map.get_waypoint(carla.Location(location[0], location[1], location[2]))

        route_trace = self._trace_route(start_waypoint, end_waypoint)   
        assert route_trace

        self._local_planner.set_global_plan(route_trace)
        
    
    def _trace_route(self, start_waypoint, end_waypoint):
        """
        This method sets up a global router and returns the optimal route
        from start_waypoint to end_waypoint
        """
        # Setting up global router
        if self._grp is None:
            dao = GlobalRoutePlannerDAO(self._vehicle.get_world().get_map(), self._hop_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Obtain route plan
        route = self._grp.trace_route(start_waypoint.transform.location, end_waypoint.transform.location)

        self.turn_positions_getter(route, RoadOption.RIGHT)
        self.turn_positions_getter(route, RoadOption.LEFT)

        return route

    def turn_positions_getter(self, route, state):
        """
        Returns list of all Left and right turns waypoints
        """
        count_flag = False
        temp_list=[]
        list_of_turn_waypoints=[]
        for i,j in route:
            if j==state:
                count_flag=True
                temp_list.append(i)
                continue

            if count_flag:
                start_waypoint = temp_list[0]
                end_waypoint = temp_list[-1]
                list_of_turn_waypoints.append((start_waypoint,end_waypoint))
                temp_list=[]
                count_flag=False

        if state == RoadOption.RIGHT:
            self.right_positions = list_of_turn_waypoints
        
        else:
            self.left_positions = list_of_turn_waypoints


    def _get_speed(self):
        """
        :private: get feedback velocity of ego 
        """
        v = self._vehicle.get_velocity()
        ego_speed = math.sqrt(v.x**2 + v.y**2)
        return ego_speed

    
    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """
        ## Update Environment ##
        # Check all the radars
        try:
            if self._state==AgentState.EMERGENCY_BRAKE:
                pass
            pass
        except:
            pass
        if self._world_obj.front_radar.detected:
            if abs(self._world_obj.front_radar.rel_pos[1]) < 1:
                self._front_r = [pygame.time.get_ticks(), self._world_obj.front_radar.rel_pos, 
                                                        self._world_obj.front_radar.rel_vel]
            self._world_obj.front_radar.detected = False        

        if self._world_obj.left_front_radar.detected:
            if self._world_obj.left_front_radar.rel_pos[1] < -1:
                self._left_front_r =[pygame.time.get_ticks(), self._world_obj.left_front_radar.rel_pos, 
                                                            self._world_obj.left_front_radar.rel_vel]
            self._world_obj.left_front_radar.detected = False
        if self._world_obj.left_back_radar.detected:
            if self._world_obj.left_back_radar.rel_pos[1] < -1:
                self._left_back_r = [pygame.time.get_ticks(), self._world_obj.left_back_radar.rel_pos, 
                                                            self._world_obj.left_back_radar.rel_vel]
            self._world_obj.left_back_radar.detected = False
        # Remove radar data if not detected again in 0.5 second
        if self._front_r and (pygame.time.get_ticks() - self._front_r[0] > 5000):
            self._front_r = []
        if self._left_front_r and (pygame.time.get_ticks() - self._left_front_r[0] > 5000):
            self._left_front_r = []
        if self._left_back_r and (pygame.time.get_ticks() - self._left_back_r[0] > 5000):
            self._left_back_r = []
        
        # Detect vehicles in front
        self._hazard_detected = False
        if self._front_r and (self._front_r[1][0] < 20.0):
            self._hazard_detected = True
        
        # update hazard existing time
        if self._hazard_detected:
            if self._blocked_time is None:
                self._blocked_time = pygame.time.get_ticks()
                hazard_time = 0
            else:
                hazard_time = pygame.time.get_ticks() - self._blocked_time
        else:
            self._blocked_time = None

        # Get a safe_distance
        safe_distance = self._THW * self._get_speed()
        
        try:
            i=self.right_positions[0][0]
            j=self.right_positions[0][1]
            loc_start = i.transform.location
            loc_start_yaw = i.transform.rotation.yaw 
            loc = loc_start
            loc_end_yaw = j.transform.rotation.yaw
            loc_end = j.transform.location
            if (abs(loc.x-self._vehicle.get_location().x)+\
                abs(loc.y-self._vehicle.get_location().y)+\
                abs(loc.z-self._vehicle.get_location().z))<=10:
                self.right_turn=True
                self.temp_flag = False

        except:
            pass

        try:
            i=self.left_positions[0][0]
            j=self.left_positions[0][1]
            loc2_start = i.transform.location
            loc2_start_yaw = i.transform.rotation.yaw  
            loc2 = loc2_start
            loc2_end = j.transform.location
            loc2_end_yaw = j.transform.rotation.yaw 
            if (abs(loc2.x-self._vehicle.get_location().x)+\
                abs(loc2.y-self._vehicle.get_location().y)+\
                abs(loc2.z-self._vehicle.get_location().z))<=10:
                self.left_turn=True
                self.temp_flag = False

        except:
            pass


        # Finite State Machine
        # 1, Navigating
        if self._state == AgentState.NAVIGATING:
            if self._hazard_detected:
                self._state = AgentState.BLOCKED_BY_VEHICLE

        # 2, Blocked by Vehicle
        elif self._state == AgentState.BLOCKED_BY_VEHICLE:
            if not self._hazard_detected:
                self._state = AgentState.NAVIGATING
            # The vehicle is driving at a certain speed
            # There is enough space
            else:
                if hazard_time > 5000 and \
                    190 > self._vehicle.get_location().x > 10 and \
                    10 > self._vehicle.get_location().y > 7:
                    self._state = AgentState.PREPARE_LANE_CHANGING

        # 4, Prepare Lane Change
        elif self._state == AgentState.PREPARE_LANE_CHANGING:
            if  not (self._front_r and self._front_r[1][0] < safe_distance) and \
                not (self._left_front_r and self._left_front_r[1][0] < safe_distance) and \
                not (self._left_back_r and self._left_back_r[1][0] > -10):
                    self._state = AgentState.LANE_CHANGING
                    self._perform_lane_change = True

        # 5, Lane Change
        elif self._state == AgentState.LANE_CHANGING:
            if abs(self._vehicle.get_velocity().y) < 0.5 and \
               self._vehicle.get_location().y < 7.0:
                self._state = AgentState.NAVIGATING

        
        # 6, Emergency Brake
        emergency_distance = safe_distance *3/5
        emergency_front_speed = 1.0
        if self._front_r and (self._front_r[1][0] < emergency_distance or 
                                self._front_r[2][0] < emergency_front_speed):
            self._state = AgentState.EMERGENCY_BRAKE


        # Local Planner Behavior according to states
        if self._state == AgentState.NAVIGATING or self._state == AgentState.LANE_CHANGING:
            control = self._local_planner.run_step(debug=debug)

        elif self._state == AgentState.PREPARE_LANE_CHANGING:
            if self._left_front_r and self._left_front_r[1][0] < safe_distance or \
               self._front_r and self._front_r[1][0] < safe_distance:
                control = self._local_planner.empty_control(debug=debug)
            else:
                control = self._local_planner.run_step(debug=debug)

        elif self._state == AgentState.BLOCKED_BY_VEHICLE:
            # ACC
            front_dis = self._front_r[1][0]
            front_vel = self._front_r[2][0]
            ego_speed = self._get_speed()
            desired_speed = front_vel - (ego_speed-front_vel)/front_dis
            if ego_speed > 1:
                desired_speed += 2*(front_dis/ego_speed - self._THW)
            control = self._local_planner.run_step(debug=debug, target_speed=desired_speed*3.6)

        elif self._state == AgentState.EMERGENCY_BRAKE:
            control = self._local_planner.brake()
            if self._front_r:
                if self._front_r[1][0] >= emergency_distance and \
                    self._front_r[2][0] > emergency_front_speed:
                    self._state = AgentState.NAVIGATING

        elif self._state == AgentState.BLOCKED_RED_LIGHT:
            control = self._local_planner.empty_control(debug=debug)

        # When performing a lane change
        if self._perform_lane_change:
            # Record original destination
            destination = self._local_planner.get_global_destination()
            # Get lane change start location
            ref_location = self._world_obj.player.get_location()
            ref_yaw = self._world_obj.player.get_transform().rotation.yaw

            if self._local_planner.waypoint_buffer:
                waypoint = self._local_planner.waypoint_buffer[-1][0]
                ref_location = waypoint.transform.location
            
            wait_dist = 0.0  # need some time to plan
            ref = [ref_location.x + wait_dist, ref_location.y, ref_yaw]

            # Replace current plan with a lane change plan
            overtake = BezierOverTake(self._world_obj)
            overtake_plan = overtake.get_waypoints(ref)
            self._local_planner.set_local_plan(overtake_plan)

            # replan globally with new vehicle position after lane changing
            new_start = self._map.get_waypoint(overtake_plan[-1][0].transform.location)
            route_trace = self._trace_route(new_start, destination)
            assert route_trace
            self._local_planner.add_global_plan(route_trace)

            self._perform_lane_change = False
            print("overtake")
        
        if self.right_turn or self.left_turn:
            # Record original destination
            destination = self._local_planner.get_global_destination()
            # Get lane change start location
            ref_location = self._world_obj.player.get_location()
            ref_yaw = self._world_obj.player.get_transform().rotation.yaw

            if self._local_planner.waypoint_buffer:
                waypoint = self._local_planner.waypoint_buffer[-1][0]
                ref_location = waypoint.transform.location
            
            if self.right_turn:

                ref1 = [loc_start.x , loc_start.y, loc_start_yaw]
                ref2 = [loc_end.x, loc_end.y, loc_end_yaw]
                turner = BezierTurn(self._world_obj, True)
                turn_plan = turner.get_waypoints(ref1,ref2)
                self.right_turn = False
                print('Right Turn')

            elif self.left_turn:
                ref1 = [loc2_start.x, loc2_start.y, loc2_start_yaw]
                ref2 = [loc2_end.x, loc2_end.y, loc2_end_yaw] 
                turner = BezierTurn(self._world_obj,False)
                turn_plan = turner.get_waypoints(ref1,ref2)
                self.left_turn = False      
                print('Left turn')         
        

            self._local_planner.set_local_plan(turn_plan)
            # replan globally with new vehicle position after lane changing
            new_start = self._map.get_waypoint(turn_plan[-1][0].transform.location)
            route_trace = self._trace_route(new_start, destination)
            assert route_trace
            self._local_planner.add_global_plan(route_trace)


        return control

    def done(self):
        """
        Check whether the agent has reached its destination.
        :return bool
        """
        return self._local_planner.done()
