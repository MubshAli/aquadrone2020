import rospy
import numpy
from data_structutes import Vector, Submarine, Gate, Pole, Wall, CompetitionMap, VisionData


class Localization:
	def __init__(self, localization_topic='localization', vision_topic='vision', pressure_sensor_topic='pressure', gyro_topic='gyro'):
		self.localization_publisher = rospy.Publisher(localization_topic, CompetitionMap, queue_size=1)
		rospy.Subscriber(vision_topic, VisionData, process_vision_data, queue_size=10)
		rospy.Subscriber(pressure_sensor_topic, PressureData, process_pressure_data, queue_size=10)
		rospy.Subscriber(gyro_topic, GyroData, process_gyro_data, queue_size=10)

		sub = Submarine(Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0), Vector(0, 0, 0))
		sub.set_uncertainties(Vector(0, 0, None), Vector(None, None, 0), None, None)  # only x, y and yaw are known by definition
		self.competition_map = CompetitionMap(sub)

	def process_vision_data(self, vision_data):
		pass

	def process_pressure_data(self, pressure_data):
		pass

	def process_gyro_data(self, gyro_data):
		pass