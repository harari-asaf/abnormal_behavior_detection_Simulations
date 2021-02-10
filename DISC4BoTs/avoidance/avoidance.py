import sys, os
sys.path.append(os.path.abspath(os.path.join('..', 'DISC4BoTs')))
from PythonClient import airsim
from scenario import Scenario

import numpy as np
from math import *
import random
import time


"""Compute resultant normalized angle

Args:
    goal (list): desired point
    pos (list): current position

Returns:
    angle (float): desired angle according to pos and goal
"""
def get_vec_dist_angle (goal, pos):
    vec = np.array(goal) - np.array(pos)
    angle = atan2(vec[1],vec[0])
    if angle > pi:
        angle -= 2*pi
    elif angle < -pi:
        angle += 2*pi
    return angle


class Avoidance(Scenario):
    """
		A class representing the Avoidance scenario.
        Drones are moving from one axis towards another parallel axis. 
        In their way there are simulated obstacles, from which they are supposed to avoid.

		Attributes:
            - Scenario Attributes
            client (airsim.MultirotorClient): the client which is connected to airsim
            drones (list): list of simulated drones
            initial_positions (list): initial positions of all drones
            malicious (str): the malicious drones which represents the anomaly
            anomaly_type (str): the type of malicious behavior (none/shift/random/path)
            time_factor (int): the simulation speed is multiplied by the time factor

            pos (dict): the current position of all drones
            started anomaly (bool): True when the anomaly has started
            yaw (dict): the current yaw of all drones
            goal (list): the goal axis
            base (list): the beginning of the drones
            time (int): time until the scenario ends (in seconds)
            obstacles (list): the scenario's obstacles
            start (float): the beginning time of the scenario

            - Constants
            step: the maximum step a drone can move in a single action. set to 0.5 meters
            coll_thres: the distance for the drones to keep from obstacles. set to 1.5 meters

		Methods:
            goto_initial_locations():
                Determine and set the initial position for all drones
            moveUAV(client, pos, yaw, drone):
                Move drone to a certian position with a certain yaw
            get_next_vec(goal, pos, yaw, coll_thres):
                Calculate the next position and yaw for drone according to the goal and the obstacles  
            get_obstacles():
                Set the obstacles for the scenario 
            remove_mal_drones(mal_drone):
                Remove the malicious drone from current drones when anomaly starts
            check_distance(pos, goal):
                Check and returns the distance between two given points
            detect_collision(pos, yaw, coll_thres):
                Return True if by the next step of the drone, it will collide with a obstacle
            has_ended():
                Check if the time of the scenario has passed
            get_collision():
                Check which drones are colliding with each other, if at all

            next_action():
                The action for all drones to make - moving towards the goal while avoiding the obstacles
            next_action_anomaly():
                The malicious action for the malicous drone to make (shift/random/path)
    """
    
    def __init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor=1):
        Scenario.__init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor)

        self.started_anomaly = False
        self.pos = {}
        # the new initial positions will be later determined according to the goal & base axis
        self.initial_positions = []

        self.yaw = {}
        for drone in _drones:
            self.yaw[drone] = 0
        
        # The maximum step a drone can move in a single action
        self.step = 0.5
        # The distance to keep from obstacles
        self.coll_thres = 1.5
        
        # The first item in goal represents the axis (0 = x, 1 = y), the second item represents the index
        # (for example [1, 60] would represent that the goal is 60 on the y axis)
        self.goal = [0, 0]
        self.goal[0] = np.random.randint(0, 2)
        self.goal[1] = np.random.randint(-30, 30)

        # base represents the initial axis of the drones
        # its axis is the same as the goal, and its distance from the goal is 80-120 meters
        base_range = random.choice([(-120, -80),(80, 120)])
        self.base = self.goal[1] + random.randint(*base_range)

        # The time of the scenario is the minimum time needed to get from the base to the goal
        # (theoretically going in straight line, without obstacles on the way)
        self.time = abs(self.base - self.goal[1]) / (2 * _time_factor)
       
        # Set the drones in their initial locations
        self.goto_initial_locations()
        # Get the scenario's obstacles
        self.obstacles = self.get_obstacles()
        # Get the beginning time of the scenario
        self.start = time.perf_counter()


    """
    Determine and set the initial positions for the drones
    The initial positions are on the base axis
    (If the base is x = 120, all initial positions' x value would be 120)
    """
    def goto_initial_locations(self):
        if self.initial_positions == []:
            sim_drones_num = 5

            # Choose a unique coordinate for each drone
            # All drones have share one of their coordinates (the base value)
            unique_cors = random.sample(range(-30, 30), sim_drones_num)

            # Create the initial positions for each drone by the base and their unique coordinate
            for i in range(sim_drones_num):
                pos = [0, 0, -2]
                pos[self.goal[0]] = self.base
                pos[1 - self.goal[0]] = unique_cors[i]
                self.initial_positions.append(pos)

        # Set all drones' position to their initial position
        for i, drone in enumerate(self.drones):
            self.pos[drone] = list(self.initial_positions[i])
            self.client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(self.pos[drone][0], self.pos[drone][1], self.pos[drone][2])), True, drone) 


    """ Move drone to certian position with certain yaw

        Args:
            client (airsim.MultirotorClient) - current client connection
            pos(list) - desired position for the drone
            yaw (float) - desired yaw for the drone
            drone (str) - drone to move

        Returns:
            None
    """
    def moveUAV(self, client, pos, yaw, drone):
        client.moveToPositionAsync(pos[0], pos[1], pos[2], 2, yaw_mode=airsim.YawMode(True, yaw), vehicle_name=drone)
        

    """ Calculates the next position and yaw for drone according to the goal and to the obstacles   

        Args:
            goal (list) - the drone's current goal
            pos(list) -the drone's current position
            yaw (float) - drone's current yaw
            coll thres (int) - distance to keep from the obstalces (constant)

        Returns:
            pos (list) - the next position (point) of the drone
            yaw (float) - the next yaw of the drone
    """
    def get_next_vec(self, goal, pos, yaw, coll_thres):
        limit_yaw = 5
        # Compute angle to goal
        t_angle = get_vec_dist_angle(goal, pos[:-1])

        # If collision occurs, avoid the obstacles by changing the yaw
        if self.detect_collision(pos, yaw, coll_thres):
            yaw -= radians(limit_yaw)
        else:
            yaw += min(t_angle - yaw, radians(limit_yaw))

        # Calculating next position
        pos[0] = pos[0] + self.step*cos(yaw)
        pos[1] = pos[1] + self.step*sin(yaw)
        return pos, yaw


    """ Randomly sets the obstacles for the scenario
        Obstacles are created specificaly between the base and the goal

        Args:
           None

        Returns:
            obs (list): list of points which represent places where there are obstacles
    """
    def get_obstacles(self):
        # obstacles are set from base to goal, and from -40 to 40 on the other axis
        # Drones' initial position on this axis is from -30 to 30
        goal_rng = 40
        end_loop = False
        i = 0
        # list of obstacles
        obs = []
        
        # The first obstacle is set at the minimum values of both axis
        obs.append([0, 0])
        dist = 0
        obs[0][self.goal[0]] = min(self.base, self.goal[1]) + dist
        obs[0][1 - self.goal[0]] = (-goal_rng) + dist
        
        # The current base which is not on the goal axis (from -40 to 40)
        sec_base = obs[0][1 - self.goal[0]]

        while not end_loop:
            # Each obstacle is determined relatively to the previous one
            # Random distance on the goal's axis from the next obstacle
            dist = np.random.uniform(-5, 7)
            new_obs = [0, 0]
            # Set the new obstacle value at the goal axis as the previous value + random dist
            new_obs[self.goal[0]] = obs[i][self.goal[0]] + dist
            # Set the new obstacle value at the none goal axis as the previous value + another random dist
            new_obs[1 - self.goal[0]] = sec_base + (np.random.uniform(-4, 5))

            # Loop ends when we reached the end of the range on the none goal axis
            end_loop = new_obs[1 - self.goal[0]] >= goal_rng

            # If we reached the maximum on the goal axis, reset to the minimum
            if new_obs[self.goal[0]] >= max(self.base, self.goal[1]):
                new_obs[self.goal[0]] = min(self.base, self.goal[1]) + dist
                new_obs[1 - self.goal[0]] =  obs[i][1 - self.goal[0]] + dist
                sec_base = new_obs[1 - self.goal[0]]

            obs.append(new_obs)
            i += 1
        
        return obs

    """ Remove the malicious drone from the list of drones when anomaly starts
        so it won't keep behaving regulary

        Args:
           mal_drones (str) - the malicious drone

        Returns:
            obs (list): list of points which represent places where there are obstacles
    """
    def remove_mal_drones(self, mal_drone):
        self.drones = [item for item in self.drones if item not in mal_drone]


    """ Check the distance between two points

        Args:
           pos (list) - current drone's position
           goal (list) - current drone's goal

        Returns:
            dist (float): distance between pos and goal
    """
    def check_distance(self, pos, goal):
        if len(pos) > 2:
            pos = [pos[0], pos[1]]
        vec = np.array(goal) - np.array(pos)
        dist = sqrt(vec[0]**2 + vec[1]**2)
        return dist
    

    """ Check if in the next drone's action, it will collide with an obstacle

        Args:
           pos (list) - current drone's position
           yaw (float) - current drone's yaw
           coll_thres(float) - distance to keep from obstacles (constant)

        Returns:
            True if a collision is about to happen, otherwise False
    """
    def detect_collision(self, pos, yaw, coll_thres):
        new_pos = [pos[0] + self.step*cos(yaw), pos[1] + self.step*sin(yaw)]
        for obstacle in self.obstacles:
            # if the distance between the next position and an obstacle is smaller than coll_thres
            if self.check_distance(new_pos, obstacle) < coll_thres:
                return True
        return False


    """ Drones' next action - moving towards the target while avoiding obstacles
    """
    def next_action(self):
        for drone in self.drones:
            # Set the goal according to the goal axis
            drone_goal = [0, 0]
            drone_goal[self.goal[0]] = self.goal[1]
            drone_goal[1 - self.goal[0]] = self.pos[drone][1 - self.goal[0]]

            # Calculate the movement to next position
            self.pos[drone], self.yaw[drone] = self.get_next_vec(drone_goal, self.pos[drone], self.yaw[drone], self.coll_thres)
            self.moveUAV(self.client, self.pos[drone], self.yaw[drone], drone)


    """
    Return true if more than self.time passed, which means the scenario's time is up
    """
    def has_ended(self):
        return (time.perf_counter() - self.start > self.time)


    """ Check which drones are colliding with each other, if at all

        Args:
           None

        Returns:
            col_drones (list): list of drones which are currently colliding
    """
    def get_collision(self):
        col_drones = []
        for drone in self.drones:
            # if has_collided is True - the drone is in collision 
            if self.client.simGetCollisionInfo(drone).has_collided:
                col_drones.append(drone)
        return col_drones


    """
    The malicious drone's next action
    There are three types of malicious behaviours: (determined in the config file)
    Shift - the malicious drone will shift along a random axis (straight line)
    Random - the malicious drone will move to random point which is constantly changing
    Path - the malicous drone will follow a random path (move from point to point in straight lines)
    """
    def next_action_anomaly(self):
        # Random anomaly was chosen
        if self.anomaly_type == "random":
            range_num = 400
            pos = [0, 0]
            # Select a random point in range of (-400, 400)
            pos[0] = np.random.uniform(-range_num, range_num)
            pos[1] = np.random.uniform(-range_num, range_num)
            # Move towards this point
            self.client.moveToPositionAsync(pos[0], pos[1], -2, 2, vehicle_name = self.malicious)

        else: 
            if not self.started_anomaly:
                self.started_anomaly = True

                # Path anomaly was chosen
                if self.anomaly_type == "path":
                    num_of_points = 40
                    range_num = 25
                    path = []
                    # Start the path from the current's malicious position
                    path.append(self.client.simGetVehiclePose(self.malicious).position) 

                    # Add random points to the path in range of (-25, 25)
                    for i in range(1, num_of_points):
                        add_x = np.random.uniform(-range_num, range_num)
                        add_y = np.random.uniform(-range_num, range_num)
                        path.append(airsim.Vector3r(path[i - 1].x_val + add_x, path[i - 1].y_val + add_y, -2))
                    
                    # Start moving on this path (from point to point)
                    self.client.moveOnPathAsync(path, 2, vehicle_name = self.malicious)

                else:
                    # Shift anomaly was chosen (default)
                    mal_goal = [0, 0, -2]
                    # Choose a very far point and move towards it
                    mal_goal_range = random.choice([(-2000,-1000),(1000,2000)])
                    mal_goal[0] = np.random.randint(*mal_goal_range)
                    mal_goal_range = random.choice([(-2000,-1000),(1000,2000)])
                    mal_goal[1] = np.random.randint(*mal_goal_range)

                    self.client.moveToPositionAsync(mal_goal[0], mal_goal[1], mal_goal[2], 2, vehicle_name = self.malicious)