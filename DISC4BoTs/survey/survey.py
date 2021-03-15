from rectangle import Rectangle

import sys, os
sys.path.append(os.path.abspath(os.path.join('..', 'DISC4BoTs')))
from PythonClient import airsim
from scenario import Scenario

import time
import random
import numpy as np


"""Transforming the created path to an airsim path which can be
    used by the moveOnPathAsync function later
Args:
    old_path (list): the created path - list of points
Returns:
    new_path (list): the new path, with the points as an airsim Vector3r with 3 coordinates
"""
def to_airsim_path(old_path: list):
    new_path = []
    for pos in old_path:
        new_path.append(airsim.Vector3r(pos[0], pos[1], -20))
    return new_path


class Survey(Scenario):
    """
    A class representing the scenario of surveying a rectangle
    Each drone has its own random area to survey, therefore a different path.
    Initial positions, scenario time and route are random.
    Attributes:
        - Scenario Attributes
        client (airsim.MultirotorClient): the client which is connected to airsim
        drones (list): list of simulated drones
        initial_positions (list): initial positions of all drones
        malicious (str): the malicious drones which represents the anomaly
        anomaly_type (str): the type of malicious behavior (none/shift/random/path)
        time_factor (int): the simulation speed is multiplied by the time factor
        paths (list): list of all drones' paths
        time (float): time until the scenario ends (in seconds)
        started (bool): True when the scenario has started
        started anomaly (bool): True when the anomaly has started
        start (float): the beginning time of the scenario
    Methods:
        goto_initial_locations(locs):
            Set the initial position for all drones according to their initial point on their path
        has_ended():
            Check if the time of the scenario has passed
        get_collision():
            Check which drones are colliding with each other, if at all
        next_action()
            The action for all drones to make - moving on their surveying path
        next_action_anomaly()
            The malicious action for the malicous drone to make (shift/random/path)
    """
    def __init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor=1):
        Scenario.__init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor)
        
        num_of_sim_drones = 5
       
        # Create a rectangle of size 200 x 200 and split randomly to all drones
        initial_rectangle = Rectangle((-25, -25), (25, 25))
        rectangles = initial_rectangle.divide(num_of_sim_drones)
        self.paths = [to_airsim_path(r.get_path()) for r in rectangles]

        # Set the time of the scenario to be the time it will take to survey the average rectangle 
        # (2 is the velocity of the drones)
        distance_list = [r.get_distance() for r in rectangles]
        avg_distance = sum(distance_list) / len(distance_list)
        self.time = avg_distance / (2 * _time_factor)
        
        # The initial position each drone is the initial point in its path
        self.goto_initial_locations([path[0] for path in self.paths])

        self.started = False
        self.started_anomaly = False
        # Get the beginning time of the scenario
        self.start = time.perf_counter()



    """
    Set the initial positions for the drones according to the given initial positions
    Args:
        locs - the drones' initial locations
    
    Returns: 
        None
    """
    def goto_initial_locations(self, locs):
        for i in range(len(self.drones)):
            self.client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(locs[i].x_val, locs[i].y_val, locs[i].z_val), airsim.to_quaternion(0, 0, 0)), True, self.drones[i]) 


    """
     Drones' next action - moving in the scanning path
     This function is executed once.
    """
    def next_action(self):
        if not self.started:
            self.started = True
            for i in range(len(self.drones)):
                self.client.moveOnPathAsync(self.paths[i], 2, vehicle_name=self.drones[i])
    

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
    Return true if more than self.time passed, which means the scenario's time is up
    """
    def has_ended(self):
        return (time.perf_counter() - self.start > self.time)
    

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