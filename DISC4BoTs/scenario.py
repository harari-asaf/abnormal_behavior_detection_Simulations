from abc import ABC, abstractmethod
import time

class Scenario(ABC):
	"""
		An abstract class used to represent a scenario (a drone's movement pattern)

		Attributes:
			client (airsim.MultirotorClient): the client which is connected to airsim
			drones (list): list of simulated drones
			initial_positions (list): initial positions of all drones
			malicious (str): the malicious drones which represents the anomaly
			anomaly_type (str): the type of malicious behavior (none/shift/random/path)
			time_factor (int): the simulation speed is multiplied by the time factor

		Methods:
			next_action() (abstract method)
				The action for all drones to make
			next_action_anomaly() (abstract method)
				The malicious action for the malicous drone to make
    """

	def __init__(self, _client, _drones, _initial_positions, _malicious, _anomaly_type, _time_factor):
		self.client = _client
		self.drones = _drones
		self.initial_positions = _initial_positions
		self.malicious = _malicious
		self.anomaly_type = _anomaly_type
		self.time_factor = _time_factor


	@abstractmethod
	def next_action(self):
		pass

	@abstractmethod
	def next_action_anomaly(self):
		pass
