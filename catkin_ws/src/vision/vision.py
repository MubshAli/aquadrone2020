import rospy
from data_structures import Vector, Gate, Pole, Wall, DirectionalMarker, VisionData


class Vision:
	def __init__(self, vision_output_topic='vision', zed_camera_topic='/zed/zed_node/right_raw/image_raw_color'):
		rospy.Subscriber(zed_camera_topic, ZEDData, process_zed_frame, queue_size=1)  # always look at the most recent frame
		self.output_publisher = rospy.Publisher(vision_output_topic, VisionData, queue_size=10)


	def process_zed_frame(self, zed_data):
		pass


	def output_data(self, labels, data):
		output_publisher.publish(VisionData(labels, data))