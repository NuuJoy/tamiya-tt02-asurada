#!/usr/bin/env python

'''
rosrun ubloxm8n ubloxm8n_cps.py queue_size=1 rate=10
'''

import sys
import time
import numpy

import rospy
from std_msgs.msg import Vector3

import smbus

class magneticSensor():
    def __init__(self,sysInput):
        self.queue_size = 1
        self.rate       = 10
        for eachInput in sysInput:
            if '=' in eachInput:
                name, val = eachInput.split('=')
                if name == 'queue_size':
                    self.queue_size = int(val)
                elif name == 'rate':
                    self.rate = int(val)

        # Setup I2C compass
        self.bus = smbus.SMBus(1)
        self.bus.write_byte_data(0x1E, 0x00, 0x60)
        self.bus.write_byte_data(0x1E, 0x02, 0x00)

        self.pub_magxyz = rospy.Publisher('/ubloxm8n/magxyz', Vector3, queue_size=self.queue_size)

        time.sleep(1.0)

    def dataGetAndPublish(self):
        # HMC5883 compass
        def magneticCalculate(data1,data2):
            mag = data1 * 256 + data2
            if mag > 32767 :
                mag -= 65536
            return mag
        data = self.bus.read_i2c_block_data(0x1E, 0x03, 6)
        ublox_xMag = magneticCalculate(data[0],data[1])
        ublox_yMag = magneticCalculate(data[2],data[3])
        ublox_zMag = magneticCalculate(data[4],data[5])

        self.pub_magxyz.publish(Vector3(x=ublox_xMag,y=ublox_yMag,z=ublox_zMag))

if __name__ == '__main__':
    try:
        HMC5883 = magneticSensor(sys.argv)

        rospy.init_node('ubloxm8n_cps', anonymous=True)
        rate = rospy.Rate(HMC5883.rate)

        while not rospy.is_shutdown():
            HMC5883.dataGetAndPublish()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
