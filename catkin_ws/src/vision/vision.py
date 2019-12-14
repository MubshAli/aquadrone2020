import rospy


class Vector:
	def __init__(self, x, y, z):
		self.x = x
		self.y = y
		self.z = z


	def magnitude(self):
		return np.linalg.norm([x_i for x_i in [self.x, self.y, self.z] if x_i is not None])

	
	def mean(*vectors):
		return sum(vectors) / len(vectors)


	def std_mean(*vectors):
		"""
		:@return: the component-wise standard deviation of the mean of a set of vectors with the given standard deviations
		"""
		return sum(v ** 2 for v in vectors) ** 0.5 / len(vectors)


	def __add__(self, vector):
		return Vector(self.x + other.x, self.y + other.y, self.z + other.z)


	def __sub__(self, vector):
		return Vector(self.x - other.x, self.y - other.y, self.z - other.z)


	def __mul__(self, scalar):
		return Vector(self.x * scalar, self.y * scalar, self.z * scalar)


	def __truediv__(sefl, scalar):
		return Vector(self.x / scalar, self.y / scalar, self.z / scalar)


	def __pow__(self, exponent):
		return Vector(self.x ** exponent, self.y ** exponent, self.z ** exponent)  # exponentiation is component-wise


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


	def set_uncertainties(x, y):
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


class DirectionalMarker:
	def __init__(self, raw_points):
		"""
		:param raw_points: The 3 points that make up the directional marker in the order [near, middle, far] where the marker is pointing from near to far
		"""
		self.raw_points = raw_points
		self.position = postition[1]
		self.direction = raw_points[2] - raw_points[1]
		self.postition_std = None
		self.direction_std = None


	def set_uncertainties(self, raw_points):
		self.postition_std = raw_points[1]
		self.direction_std = Vector.std_mean(raw_points[2], raw_points[1])


class Vision:
	def __init__(self, vision_output_topic='vision', zed_camera_topic='/zed/zed_node/right_raw/image_raw_color'):
		rospy.Subscriber(zed_camera_topic, ZEDData, process_zed_frame, queue_size=1)  # always look at the most recent frame
		self.output_publisher = rospy.Publisher(vision_output_topic, VisionData, queue_size=10)


	def process_zed_frame(self, zed_data):
		pass


	def output_data(self, labels, data):
		output_publisher.publish(VisionData(labels, data))