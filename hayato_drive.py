#!/usr/bin/env python

import sys
import rospy
import std_msgs.msg
from ds4_driver.msg import Feedback, Status
from pca9685.msg import pwm_status
from hayato_kazami.msg import hayato_to_pwm


class hayatoDrive():
    def __init__(self,sysInput):
        chassis_curSteer = None
        chassis_curDrive = None
        picam_curX       = None
        picam_curY       = None

        self.pub_pwmSet = rospy.Publisher('/hayato/pwm_output', hayato_to_pwm, queue_size=1)
    
    def pwm_response(self,pwmStatus):
        self.chassis_curSteer = pwmStatus.current_steer
        self.chassis_curDrive = pwmStatus.current_drive
        self.picam_curX       = pwmStatus.current_picamx
        self.picam_curY       = pwmStatus.current_picamy

    def ds4_callback(self,ds4Status):
        # Steering control
        setSteer = ds4Status.axis_right_x
        # Throttle control
        #setDrive = ds4Status.axis_right_y
        if ds4Status.axis_l2 > 0:
            setDrive = ds4Status.axis_l2
        else:
            setDrive = -1.0*ds4Status.axis_r2
        # piCamera control
        setPiCamX = ds4Status.axis_left_x
        setPiCamY = ds4Status.axis_left_y

        self.pub_pwmSet.publish(hayato_to_pwm(setSteer,setDrive,setPiCamX,setPiCamY))

if __name__ == '__main__':
    try:
        rospy.init_node('hayato_drive', anonymous=True)

        hayato_driver = hayatoDrive(sys.argv)
        rospy.Subscriber('/status', Status, hayato_driver.ds4_callback, queue_size=1)
        rospy.Subscriber('/pca9685/pwm_status', pwm_status, hayato_driver.pwm_response, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
