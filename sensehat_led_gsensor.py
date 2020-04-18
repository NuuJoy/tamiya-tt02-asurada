#!/usr/bin/env python

import rospy
import sys
import math
import sense_hat
import std_msgs.msg
from raspi_sensehat.msg import sensehat_imu

class raspiLEDDisplay():
    def __init__(self):
        self.sense = sense_hat.SenseHat()
        self.sense.clear()
        self.sense.set_rotation(270)

        self.xy_sen = 1.0
        self.z_sen  = 1.0
        self.size   = 0.175
        self.hue    = [1.0,0.0,0.0]
        self.led_PosMap = [[-1.0,-1.0  ],[-0.714,-1.0  ],[-0.429,-1.0  ],[-0.143,-1.0  ],[0.143,-1.0  ],[0.429,-1.0  ],[0.714,-1.0  ],[1.0,-1.0  ],
                           [-1.0,-0.714],[-0.714,-0.714],[-0.429,-0.714],[-0.143,-0.714],[0.143,-0.714],[0.429,-0.714],[0.714,-0.714],[1.0,-0.714],
                           [-1.0,-0.429],[-0.714,-0.429],[-0.429,-0.429],[-0.143,-0.429],[0.143,-0.429],[0.429,-0.429],[0.714,-0.429],[1.0,-0.429],
                           [-1.0,-0.143],[-0.714,-0.143],[-0.429,-0.143],[-0.143,-0.143],[0.143,-0.143],[0.429,-0.143],[0.714,-0.143],[1.0,-0.143],
                           [-1.0, 0.143],[-0.714, 0.143],[-0.429, 0.143],[-0.143, 0.143],[0.143, 0.143],[0.429, 0.143],[0.714, 0.143],[1.0, 0.143],
                           [-1.0, 0.429],[-0.714, 0.429],[-0.429, 0.429],[-0.143, 0.429],[0.143, 0.429],[0.429, 0.429],[0.714, 0.429],[1.0, 0.429],
                           [-1.0, 0.714],[-0.714, 0.714],[-0.429, 0.714],[-0.143, 0.714],[0.143, 0.714],[0.429, 0.714],[0.714, 0.714],[1.0, 0.714],
                           [-1.0, 1.0  ],[-0.714, 1.0  ],[-0.429, 1.0  ],[-0.143, 1.0  ],[0.143, 1.0  ],[0.429, 1.0  ],[0.714, 1.0  ],[1.0, 1.0  ]]
        
        self.acc_x = [0,0,0,0,0,0,0,0,0,0]
        self.acc_y = [0,0,0,0,0,0,0,0,0,0]
        self.acc_z = [0,0,0,0,0,0,0,0,0,0]

    def disp_update(self):

        def spotProfile(spotSize,distance):
            if distance < spotSize:
                return 255
            elif distance > spotSize*2:
                return 0
            else:
                return int(255*(2.0*spotSize-distance)/spotSize)
    
        led_reMap = []
        x = sum(self.acc_x)/len(self.acc_x)
        y = sum(self.acc_y)/len(self.acc_y)
        z = sum(self.acc_z)/len(self.acc_z)
        for i,eachPos in enumerate(self.led_PosMap): 
            # Calculate Euclidean distance
            distance = math.sqrt((eachPos[0]-self.xy_sen*y)**2+(eachPos[1]-self.xy_sen*x)**2)
            led_reMap.append([int(self.hue[0]*spotProfile(self.size*(1.0+self.z_sen*abs(1.0-z)), distance)),
                            int(self.hue[1]*spotProfile(self.size*(1.0+self.z_sen*abs(1.0-z)), distance)),
                            int(self.hue[2]*spotProfile(self.size*(1.0+self.z_sen*abs(1.0-z)), distance))])
        self.sense.set_pixels(led_reMap)

    def imu_update(self,imuStatus):
        self.acc_x = self.acc_x[1:] + [imuStatus.accelerometer_x]
        self.acc_y = self.acc_y[1:] + [imuStatus.accelerometer_y]
        self.acc_z = self.acc_z[1:] + [imuStatus.accelerometer_z]

    def mode_update(self,modeStatus):
        if modeStatus.data == 1.0:
            self.hue = [1.0,0.0,0.0]
        elif modeStatus.data == 0.5:
            self.hue = [0.0,1.0,0.0]
        elif modeStatus.data == 0.2:
            self.hue = [0.0,0.0,1.0]

def led_display_core(raspi_led):

    rospy.init_node('sensehat_led_node', anonymous=True)
    rospy.Subscriber('/sensehat/imu', sensehat_imu, raspi_led.imu_update, queue_size=1)
    rospy.Subscriber('/motor_pwm_control/mode_spdlmt', std_msgs.msg.Float64, raspi_led.mode_update, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        raspi_led.disp_update()
        rate.sleep()

if __name__ == '__main__':
    try:
        raspi_led = raspiLEDDisplay()
        led_display_core(raspi_led)

    except rospy.ROSInterruptException:
        sense.clear()