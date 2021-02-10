import sys, os
sys.path.append(os.path.abspath(os.path.join('..', 'DISC4BoTs')))

from PythonClient import airsim
from scenario import Scenario

import numpy as np
import time
import random


class Simple (Scenario):
    """
    A class representing the scenario of moving in a certain structure from (0,0) to (0, 100)
    All five drones moving along the y axis together
    Initial positions, scenario time and route are predetermined.

    Attributes:
        - Scenario Attributes
        client (airsim.MultirotorClient): the client which is connected to airsim
        drones (list): list of simulated drones
        initial_positions (list): initial positions of all drones
        malicious (str): the malicious drones which represents the anomaly
        anomaly_type (str): the type of malicious behavior (none/shift/random/path)
        time_factor (int): the simulation speed is multiplied by the time factor

        monitoring time (int): time until the scenario ends (in seconds)
        X (int) - the x value of the goal
        Z (int) - the z value of the goal
        started (bool): True when the scenario has started
        started anomaly (bool): True when the anomaly has started
        start (float): the beginning time of the scenario

        - Constants
        Y (int) - the y value of the goal. set to 100 
        V (int) - the drones' speed. set to 2 m/s

    Methods:
        goto_initial_locations():
            Set the initial position for all drones
        has_ended():
            Check if the time of the scenario has passed
        get_collision():
            Check which drones are colliding with each other, if at all
        next_action()
            The action for all drones to make - moving from one point to another
        next_action_anomaly()
            The malicious action for the malicous drone to make (shift/random/path)
    """
    def __init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor=1):
        Scenario.__init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor)

        self.Y = 100  # meter
        self.V = 2  # Ms
        self.monitoring_time = (self.Y / self.V) / _time_factor

        self.X = np.random.randint((0),(10))
        self.Z = np.random.randint((-4),(-2))

        # Set the drones to be in their initial locations
        self.goto_initial_locations()

        self.started = False
        self.started_anomaly = False
        # Get the beginning time of the scenario
        self.start = time.perf_counter()

  
    """
    Set the initial positions for the drones according to the initial positions attribute
    """
    def goto_initial_locations(self):
        for i, drone in enumerate(self.drones):
            pos = self.initial_positions[i]
            self.client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(pos[0], pos[1], pos[2]), airsim.to_quaternion(0, 0, 0)), True, drone)


    """
     Drones' next action - moving towards the goal point
     This function is executed once.
    """
    def next_action(self):
        if not self.started:
            self.started = True
            [self.client.moveToPositionAsync(self.X, self.Y, self.Z, self.V, vehicle_name = drone) for drone in self.drones]
    

    """
    Return true if more than self.time passed, which means the scenario's time is up
    """
    def has_ended(self):
        return (time.perf_counter() - self.start > self.monitoring_time)


    """ Check which drones are colliding with each other, if at all

        Args:
           None

        Returns:
            col_drones (list): list of drones which are currently colliding
    """
    def get_collision(self):
        col_drones = []
        for drone in self.drones:
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