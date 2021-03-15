# Main program for simple class

from simple import Simple

import sys, os
sys.path.append(os.path.abspath(os.path.join('..', 'DISC4BoTs')))
from PythonClient import airsim
from scenario import Scenario
from RepeatedTimer import RepeatedTimer
from data_testing import test_data

import time
import pandas as pd
import random
import numpy as np
from configparser import ConfigParser


"""Gets the drones' current state and writes it to the data dictionary

Args:
    data (dict): The dictionary containing the data
    drones (list): list of drones
    client (airsim.MultirotorClient): the monitor client used to get the drones' live data
    values (dict): a dictionary of scenario values which are created and constantly
    updated in the main program. These values are monitored and written.

Returns:
    None
"""
def monitor(data, drones, client, values):
    is_anomaly = values["is_anomaly"] # indicates whether the anomaly has started or not
    start = values["start"] # the time of the scenario's beginning
    update_step = values["update_step"] # represents the number of times next_action() has been called
    i = values["i"] #  currect iteration index
    malicious = values["malicious"] # the malicious drone

    # Start monitoring the data 
    # Each line in the data represents a current state of a single drone
    for drone_index, drone in enumerate(drones):
        # Get drone information from the simulator
        gps = client.getGpsData(vehicle_name=drone)
        imu = client.getImuData(vehicle_name=drone)
        state = client.getMultirotorState(vehicle_name=drone)
        # The label value is 1 when the drone is acting maliciously
        label = 1 if is_anomaly and drones[drone_index] in malicious else 0
        # The collision value is 1 when the drone is colliding with another drone
        collision = 1 if drones[drone_index] in values["collision"] else 0

        # Write all the values to the data dictionary
        data_vals = [drones[drone_index],
                        update_step, i,
                        time.perf_counter() - start,
                        gps.time_stamp,
                        gps.gnss.geo_point.latitude,
                        gps.gnss.geo_point.longitude,
                        gps.gnss.geo_point.altitude,
                        state.kinematics_estimated.angular_acceleration.x_val,
                        state.kinematics_estimated.angular_acceleration.y_val,
                        state.kinematics_estimated.angular_acceleration.z_val,
                        imu.angular_velocity.x_val,
                        imu.angular_velocity.y_val,
                        imu.angular_velocity.z_val,
                        imu.linear_acceleration.x_val,
                        imu.linear_acceleration.y_val,
                        imu.linear_acceleration.z_val,
                        gps.gnss.velocity.x_val,
                        state.kinematics_estimated.linear_velocity.y_val,
                        state.kinematics_estimated.linear_velocity.z_val,
                        state.kinematics_estimated.orientation.x_val,
                        state.kinematics_estimated.orientation.y_val,
                        state.kinematics_estimated.orientation.z_val,
                        state.kinematics_estimated.position.x_val,
                        state.kinematics_estimated.position.y_val,
                        state.kinematics_estimated.position.z_val,
                        collision, label]
        [data[key].append(val) for key, val in zip(data.keys(), data_vals)]


def main():
  #Read config.ini file
  config_object = ConfigParser()
  init_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.ini')
  config_object.read(init_file)

  # The file in which the data will be written into
  output_file_name = config_object.get("SETTINGS", "output_file_name")
  # Number of different scenarios to run 
  num_of_iter = int(config_object.get("SETTINGS", "num_of_iter"))
  # Simulation time factor which is set in the settings.json file of AirSim
  sim_time_factor = int(config_object.get("SETTINGS", "sim_time_factor"))
  # The malicious behaviour type 
  anomaly_type = config_object.get("SETTINGS", "anomaly_type")
  # If True, after getting the data it will get tested and updated by 'test.py' script
  active_data_test = config_object.get("SETTINGS", "active_data_test")

  # Time until next_action() and has_ended() is called again 
  update_cycle_time = 0.1 / sim_time_factor

  # headers for csv file
  data_keys = ['drone', 'update_step', 'iter', 'script_time', 'sim_time',
              'latitude', 'longitude', 'altitude', 'angular_acceleration_x',
              'angular_acceleration_y', 'angular_acceleration_z',
              'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z',
              'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z',
              'linear_velocity_x', 'linear_velocity_y', 'linear_velocity_z',
                'orientation_x', 'orientation_y', 'orientation_z',
              'position_x', 'position_y', 'position_z', 'collision', 'label']

  data = {data: [] for data in data_keys}

  drones = ["Drone1", "Drone2", "Drone3", "Drone4", "Drone5"] 

  # Connect to the regular client
  client = airsim.MultirotorClient()
  client.confirmConnection()

  # Connect to the Monitor's client which is used by the monitor function
  client_monitor = airsim.MultirotorClient()
  client_monitor.confirmConnection()
  
  [client_monitor.enableApiControl(True, drone) for drone in drones]
  [client_monitor.armDisarm(True, drone) for drone in drones]

  # Dictionary of values that describe the current iteration state
  # It is used by the monitor function
  values = {"i": 0, "update_step": 0, "start": 0, "is_anomaly": False, "malicious": None,  "collision": None}

  # Start the scenarios - each iteration is a new scenario
  for i in range(num_of_iter):

    [client.enableApiControl(True, drone) for drone in drones]
    [client.armDisarm(True, drone) for drone in drones]

    print(f'Iteration: {i}')
    values["i"] = i

    # On the first iteration, the drones need to take off first
    if i == 0:
      [client.takeoffAsync(vehicle_name=drone) for drone in drones]
      time.sleep(6)

    # Randomly choose a malicious drone
    malicious = random.sample(drones, 1)[0]
    values["malicious"] = malicious
    # After time_till_anoamly passes, the malicious drone starts acting maliciously
    time_till_anomaly = np.random.uniform(10, 15) / sim_time_factor

    print(f'starting new scenario with malicious: {malicious}')

    # Drones' initial positions which are also set in the settings.json file of AirSim 
    initial_positions = ([[2, 0, -2], [4, 0, -2], [6, 0, -2], [8, 0, -2], [10, 0, -2]])

    # Optional : use trace line to see the routes of the drones
    [client.simSetTraceLine([0.0], 20.0, drone) for drone in drones]

    # Creating the avoidance scenario class
    scenario = Simple(client, drones, initial_positions, malicious, anomaly_type, sim_time_factor)

    # Get the beginning time for later measures
    start = time.perf_counter()
    values["start"] = start

    values["update_step"] = 0

    time_from_beginning = 0

    # When has_ended is True, the scenario will end
    has_ended = False
    # When is_anomaly is True, the malicious behaviour will start
    is_anomaly = False
    values["is_anomaly"] = False

    time.sleep(0.5)

    # Create the RepeatedTimer class with the monitor function, which monitors the scenario data
    # By creating the object, the function starts running and repeats itself every 0.05 seconds
    rt = RepeatedTimer(0.05, monitor, data, drones, client_monitor, values)            

    while not has_ended:
      record_time = time.perf_counter()
      # Call the next action of the drones, which gets them to recalculate their movement
      scenario.next_action()
      # Check if the scenario has ended
      has_ended = scenario.has_ended()
      # Check if a collision has occured
      values["collision"] = scenario.get_collision()

      # If enough time has passed, the anomaly begins
      if time_from_beginning > time_till_anomaly and anomaly_type != "none" and not is_anomaly:
        print("starts anomaly")
        is_anomaly = True
        values["is_anomaly"] = True
        # Optional - set the malicious drone's path to have another color
        client.simSetTraceLine([1.0], 20.0, malicious)
      
      # If the anomaly has started, call the malicious behavior function
      if is_anomaly:
        scenario.next_action_anomaly()

      # calculating the time that passed since the beginning of the scenario
      current = time.perf_counter()
      time_from_beginning = current - start

      values["update_step"] += 1

      # If not enough time has passed until the next action should happen, sleep
      try: time.sleep(update_cycle_time - (current - record_time))
      except: print("passing")

    # When the scenario ends, the RepeatedTimer stops recording and the client resets
    rt.stop() 
    client.reset()

  # Disconnect from the client
  [client.armDisarm(False, drone) for drone in drones]
  [client.enableApiControl(False, drone) for drone in drones]

  print('finished mission')

  # Write the data to the output file
  data_df = pd.DataFrame.from_dict(data)
  data_df.to_csv(output_file_name, index=False)

  if active_data_test == "True":
    test_data(output_file_name)

if __name__ == '__main__':
    main()





