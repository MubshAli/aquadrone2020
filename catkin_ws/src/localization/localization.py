import rospy
import numpy


class Vector:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z


	def magnitude(self):
		return np.linalg.norm([x_i for x_i in [self.x, self.y, self.z] if x_i is not None])

	
	def mean(*vectors):
		x = sum(v.x for v in vectors) / len(vectors)
		y = sum(v.y for v in vectors) / len(vectors)
		z = sum(v.z for v in vectors) / len(vectors)
		return Vector(x, y, z)


	def std_mean(*vectors):
		"""
		:@return: the component-wise standard deviation of the mean of a set of vectors with the given standard deviations
		"""
		x = (sum(v.x ** 2 for v in vectors) ** 0.5) / len(vectors)
		y = (sum(v.y ** 2 for v in vectors) ** 0.5) / len(vectors)
		z = (sum(v.z ** 2 for v in vectors) ** 0.5) / len(vectors)
		return Vector(x, y, z)


class Submarine:
	def __init__(self, position, direction, velocity, angular_velocity):
		self.position = position
		self.direction = direction
		self.velocity = velocity
		self.angular_velocity = angular_velocity
		self.position_std = None
		self.direction_std = None
		self.velocity_std = None
		self.angular_velocity_std = None


	def set_uncertainties(self, position, direction, velocity, angular_velocity):
		self.position_std = position
		self.direction_std = direction
		self.velocity_std = velocity
		self.angular_velocity_std = angular_velocity


class Gate:
	def __init__(self, top_corners, bottom_corners):
		self.top_corners = top_corners
		self.bottom_corners = bottom_corners
		self.position = Vector.mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])


	def set_uncertainties(self, top_corners, bottom_corners):
		self.top_corners_std = top_corners
		self.bottom_corners_std = bottom_corners
		self.position_std = Vector.std_mean(top_corners[0], top_corners[1], bottom_corners[0], bottom_corners[1])


class Pole:
	def __init__(self, x, y):
		self.postition = Vector(x, y, None)
		self.position_std = None


	def set_uncertainties(self, x, y):
		self.position_std = Vector(x, y, None)


class Wall:
	def __init__(self, m, b):
		# y = mx + b
		self.m = m
		self.b = b
		self.m_std = None
		self.b_std = None


	def set_uncertainties(self, m, b):
		self.m_std = None
		self.b_std = None


class CompetitionMap:
	def __init__(self, sub):
		self.sub = sub
		self.gate = None
		self.pole = None
		self.walls = []


	def set_gate(self, gate):
		self.gate = gate

	def set_pole(self, pole):
		self.pole = pole


	def add_wall(self, wall):
		self.walls.append(wall)


class VisionData:
	def __init__(self, labels, data):
		self.data = zip(labels, data)  # list of tuples of class type, object for all the objects that were found


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