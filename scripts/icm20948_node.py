#!/usr/bin/env python3

# Author: Bharat Joshi
# For use with the ICM20948


import board
import busio
import adafruit_icm20x
import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


class imuNode:
    def __init__(self):

        self.freq = 1000000  # 1 MHz
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=self.freq)
        self.icm = adafruit_icm20x.ICM20948(self.i2c)

        self.imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
        self.mag_pub = rospy.Publisher(
            "/imu/mag", MagneticField, queue_size=10)

        rospy.init_node("icm20948_node")

        self.seq = 0

        self.imu_rate = rospy.get_param("~imu_rate")

        self.event_time = 1.0 / float(self.imu_rate)  # type: ignore

        self.icm.accelerometer_data_rate_divisor = 0
        self.icm.accelerometer_data_rate_divisor = 0
        self.icm.magnetometer_data_rate = adafruit_icm20x.MagDataRate.RATE_100HZ  # type: ignore

        self.imu_publish_timer = rospy.Timer(
            rospy.Duration.from_sec(self.event_time), self.publish_imu_data)
        self.mag_publish_timer = rospy.Timer(
            rospy.Duration.from_sec(0.05), self.publish_mag_data)

    def publish_imu_data(self, timer_event):
        imu_msg = Imu()

        # header vals
        imu_msg.header.seq = self.seq
        imu_msg.header.frame_id = "imu"

        # data
        imu_start_time = rospy.Time.now()
        accel_data = self.icm.acceleration
        gyro_data = self.icm.gyro
        imu_stop_time = rospy.Time.now()

        # TODO(bjoshi): Just averaging the imu times to publish
        # Since we do not get the actual time from device
        stamp = imu_start_time + \
            rospy.Duration.from_sec(
                (imu_stop_time - imu_start_time).to_sec() * 0.5)
        imu_msg.header.stamp = stamp

        imu_msg.linear_acceleration.x = accel_data[0]
        imu_msg.linear_acceleration.y = accel_data[1]
        imu_msg.linear_acceleration.z = accel_data[2]
        imu_msg.angular_velocity.x = gyro_data[0]
        imu_msg.angular_velocity.y = gyro_data[1]
        imu_msg.angular_velocity.z = gyro_data[2]

        self.imu_pub.publish(imu_msg)

        self.seq += 1

    def publish_mag_data(self, timer_event):

        mag_msg = MagneticField()
        mag_msg.header.seq = self.seq
        mag_msg.header.frame_id = "imu"

        # TODO(bjoshi): Just averaging the imu times to publish
        # Since we do not get the actual time from device
        mag_start_time = rospy.Time.now()
        mag_data = self.icm.magnetic
        mag_stop_time = rospy.Time().now()
        stamp = mag_start_time + \
            rospy.Duration.from_sec(
                (mag_stop_time - mag_start_time).to_sec() * 0.5)

        mag_msg.header.stamp = stamp
        mag_msg.magnetic_field.x = mag_data[0]
        mag_msg.magnetic_field.y = mag_data[1]
        mag_msg.magnetic_field.z = mag_data[2]

        self.mag_pub.publish(mag_msg)


if __name__ == "__main__":
    imuNode()
    rospy.spin()
