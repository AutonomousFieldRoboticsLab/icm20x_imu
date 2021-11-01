#!/usr/bin/env python3

# Author: Alex Johnson adapted from Adafruit circuitpython icm20x example code
#	and from afrl/driftNode code
# For use with the ICM20948


import time
import board
import adafruit_icm20x
import rospy
from sensor_msgs.msg import Imu

class imuNode:

	def __init__(self):
		self.i2c = board.I2C()
		self.icm = adafruit_icm20x.ICM20948(self.i2c)
		self.pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
		rospy.init_node("imu_publisher")

		self.seq = 0
		self.prefix = "imu_"

		self.imu_rate = int( rospy.get_param("imu_rate") ) # Hz
		self.debug = int( rospy.get_param("debug") ) #debug flag, prints info
		if self.debug:
			rospy.loginfo("DEBUG ENABLED")
			rospy.Subscriber("/imu/data", Imu, self.debug_callback)

		self.icm.accelerometer_data_rate = self.imu_rate
		self.icm.gyro_data_rate = self.imu_rate
		# magnetometer can only be 100, 50, 20, or 10 Hz
		# 100 is default
		# self.icm.magnetometer_data_rate = adafruit_icm20x.MagDataRate.RATE_100HZ


		self.rate = rospy.Rate(self.imu_rate)

		while not rospy.is_shutdown():
			self.publish_data()
			self.seq +=1

			self.rate.sleep()


	def publish_data(self):
		self.msg = Imu()

		# header vals
		self.msg.header.seq = self.seq
		self.msg.header.frame_id = self.prefix
		time = rospy.get_rostime()
		self.msg.header.stamp.secs = time.secs
		self.msg.header.stamp.nsecs = time.nsecs

		# data
		self.msg.linear_acceleration.x = self.icm.acceleration[0]
		self.msg.linear_acceleration.y = self.icm.acceleration[1]
		self.msg.linear_acceleration.z = self.icm.acceleration[2]
		self.msg.angular_velocity.x = self.icm.gyro[0]
		self.msg.angular_velocity.y = self.icm.gyro[1]
		self.msg.angular_velocity.z = self.icm.gyro[2]
		self.msg.orientation.x = self.icm.magnetic[0]
		self.msg.orientation.y = self.icm.magnetic[1]
		self.msg.orientation.z = self.icm.magnetic[2]

		self.pub.publish(self.msg)


	def debug_callback(self, msg):
		if self.debug:
			rospy.loginfo("DEBUG TRUE")
			rospy.loginfo(msg)
			rospy.loginfo( "Rate: {0} \nSeq: {1}".format(self.imu_rate, self.seq) )


if __name__ == "__main__":
	imuNode()
