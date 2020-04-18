#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
import Adafruit_PCA9685
from pca9685.msg import pwm_status
from hayato_kazami.msg import hayato_to_pwm

'''
## ---- format
rosrun pca9685 pwmcontrol.py steer_pin=[12, drive_pin=13 

# Default for this project
# steer, ch=0 ,min=270,neu=310..320..330,max=370
# drive, ch=4 ,min=100,neu=310..320..330,max=512
# picamx,ch=15,min=220,neu=310..320..330,max=420
# picamy,ch=11,min=220,neu=310..320..330,max=420
'''

class pwmDriver():
    def __init__(self,sysInput):
        #Init parameter with default value
        self.steer_pin  = [ 0, 270, 320, 370]
        self.drive_pin  = [ 4, 140, 320, 500]
        self.picamx_pin = [15, 220, 320, 420]
        self.picamy_pin = [11, 220, 320, 420]

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(50) # set frequency 50 Hz
        
        self.pub_status = rospy.Publisher('/pca9685/pwm_status', pwm_status, queue_size=1)

        #Override setting from sysInput
        for eachInput in sysInput:
            if '=' in eachInput:
                name, val = eachInput.split('=')
                if name == 'steer_pin':
                    self.steer_pin   = [int(eachval) for eachval in val.split(',')]
                elif name == 'drive_pin':
                    self.drive_pin   = [int(eachval) for eachval in val.split(',')]
                elif name == 'picamx_pin':
                    self.picamx_pin  = [int(eachval) for eachval in val.split(',')]
                elif name == 'picamy_pin':
                    self.picamy_pin  = [int(eachval) for eachval in val.split(',')]
        
        print('pwmDrive start ---- steer_pin: channel {:2}, min {:3}, neutral {:3}, max {:3}'.format(*self.steer_pin))
        print('                    drive_pin: channel {:2}, min {:3}, neutral {:3}, max {:3}'.format(*self.drive_pin))
        print('                   picamx_pin: channel {:2}, min {:3}, neutral {:3}, max {:3}'.format(*self.picamx_pin))
        print('                   picamy_pin: channel {:2}, min {:3}, neutral {:3}, max {:3}'.format(*self.picamy_pin))

    def setPWM(self,hayatoRequest):
        def setPinValue(pinval,minval,neuval,maxval,setval):
            if setval == 0:
                self.pwm.set_pwm(pinval, 0, neuval)
            elif setval > 0:
                self.pwm.set_pwm(pinval, 0, neuval + int(setval*(maxval-neuval)))
            elif setval < 0:
                self.pwm.set_pwm(pinval, 0, neuval + int(setval*(neuval-minval)))
        
        setPinValue(self.steer_pin[0],self.steer_pin[1],self.steer_pin[2],self.steer_pin[3],-1.0*hayatoRequest.set_steer)        
        setPinValue(self.drive_pin[0],self.drive_pin[1],self.drive_pin[2],self.drive_pin[3],-1.0*hayatoRequest.set_drive)
        setPinValue(self.picamx_pin[0],self.picamx_pin[1],self.picamx_pin[2],self.picamx_pin[3],hayatoRequest.set_picamx)
        setPinValue(self.picamy_pin[0],self.picamy_pin[1],self.picamy_pin[2],self.picamy_pin[3],hayatoRequest.set_picamy)

if __name__ == '__main__':
    try:
        rospy.init_node('motor_pwm_control', anonymous=True)
        
        pwmControl = pwmDriver(sys.argv)
        rospy.Subscriber('/hayato/pwm_output', hayato_to_pwm, pwmControl.setPWM, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        for i in range(12):
            pwm.set_pwm(i, 0, 0)

