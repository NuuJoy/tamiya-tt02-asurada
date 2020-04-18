#!/usr/bin/env python

'''
---- Format
rosrun raspi_sensehat sensehat_sensor_acq.py [rate_hz] [queue_size] [sensor]
---- [sensor] list
accxyz accrpy gyrxyz gyrrpy magxyz magcps envall joystk
---- Example
rosrun raspi_sensehat sensehat_sensor_acq.py 100 10 joystk accxyz magcps
'''

import sys
import rospy
import sense_hat

from raspi_sensehat.msg import sensehat_env
from raspi_sensehat.msg import sensehat_imu
from std_msgs.msg import String

def sensor_acquisition(sysInput):

    rate_hz,queue_size = float(sysInput[1]),float(sysInput[2])

    sense = sense_hat.SenseHat()

    # Enable sensor
    cps_enable = True if ('magxyz' in sysInput) or ('magcps' in sysInput) else False
    gyr_enable = True if ('gyrxyz' in sysInput) or ('gyrrpy' in sysInput) else False
    acc_enable = True if ('accxyz' in sysInput) or ('accrpy' in sysInput) else False
    imu_enable = True if (cps_enable or gyr_enable or acc_enable) else False
    env_enable = True if ('envall' in sysInput) else False
    joy_enable = True if ('joystk' in sysInput) else False

    noneOut = {'x':None,'y':None,'z':None,'roll':None,'pitch':None,'yaw':None}

    sense.set_imu_config(cps_enable,gyr_enable,acc_enable)


    pub_env = rospy.Publisher('/sensehat/env', sensehat_env, queue_size=queue_size) if env_enable else None
    pub_imu = rospy.Publisher('/sensehat/imu', sensehat_imu, queue_size=queue_size) if imu_enable else None
    pub_joy = rospy.Publisher('/sensehat/joystick', String, queue_size=queue_size) if joy_enable else None

    rospy.init_node('raspi_sense_hat_sensor', anonymous=True)
    rate = rospy.Rate(rate_hz)

    def inv_z(vector3):
        return {'x':vector3['x'],'y':vector3['y'],'z':1.0*vector3['z']}

    while not rospy.is_shutdown():
        
        accxyz = inv_z(sense.get_accelerometer_raw()) if 'accxyz' in sysInput else noneOut
        accrpy = sense.get_orientation_degrees() if 'accrpy' in sysInput else noneOut
        gyrxyz = inv_z(sense.get_gyroscope_raw()) if 'gyrxyz' in sysInput else noneOut
        gyrrpy = sense.get_gyroscope() if 'gyrrpy' in sysInput else noneOut
        magxyz = inv_z(sense.get_compass_raw()) if 'magxyz' in sysInput else noneOut
        magcps = sense.get_compass() if 'magcps' in sysInput else None

        # IMU sensor
        if imu_enable:
            pub_imu.publish(sensehat_imu(accxyz['x'], accxyz['y'], accxyz['z'],
                                         accrpy['roll'], accrpy['pitch'], accrpy['yaw'],
                                         gyrxyz['x'], gyrxyz['y'], gyrxyz['z'],
                                         gyrrpy['roll'], gyrrpy['pitch'], gyrrpy['yaw'],
                                         magxyz['x'], magxyz['y'], magxyz['z'],magcps))

        # Environment sensor
        if env_enable:
            pub_env.publish(sensehat_env(sense.get_humidity(),
                                         sense.get_temperature_from_humidity(),
                                         sense.get_temperature_from_pressure(),
                                         sense.get_pressure()))

        # Joystick event
        if joy_enable:
            getEvents = sense.stick.get_events()
            for eachEvent in getEvents:
                pub_joy.publish(eachEvent.direction+'_'+eachEvent.action)

        rate.sleep()
        
if __name__ == '__main__':
    try:
        sensor_acquisition(sys.argv)
    except rospy.ROSInterruptException:
        pass



