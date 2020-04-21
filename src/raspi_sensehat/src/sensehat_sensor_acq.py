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
import std_msgs.msg

def sensor_acquisition(sysInput):

    rate_hz,queue_size = float(sysInput[1]),float(sysInput[2])

    sense = sense_hat.SenseHat()

    # Enable sensor
    acc_enable = True if ('accxyz' in sysInput) or ('accrpy' in sysInput) else False
    gyr_enable = True if ('gyrxyz' in sysInput) or ('gyrrpy' in sysInput) else False
    cps_enable = True if ('magxyz' in sysInput) or ('magcps' in sysInput) else False
    env_enable = True if ('envall' in sysInput) else False
    joy_enable = True if ('joystk' in sysInput) else False

    sense.set_imu_config(cps_enable,gyr_enable,acc_enable)

    if ('accxyz' in sysInput):
        pub_accxyz = rospy.Publisher('/sensehat/accxyz', std_msgs.msg.Vector3, queue_size=queue_size) if acc_enable else None
    if ('accrpy' in sysInput):
        pub_accrpy = rospy.Publisher('/sensehat/accrpy', std_msgs.msg.Vector3, queue_size=queue_size) if acc_enable else None
    if ('gyrxyz' in sysInput):
        pub_gyrxyz = rospy.Publisher('/sensehat/gyrxyz', std_msgs.msg.Vector3, queue_size=queue_size) if gyr_enable else None
    if ('gyrrpy' in sysInput):
        pub_gyrrpy = rospy.Publisher('/sensehat/gyrrpy', std_msgs.msg.Vector3, queue_size=queue_size) if gyr_enable else None
    if ('magxyz' in sysInput):
        pub_magxyz = rospy.Publisher('/sensehat/magxyz', std_msgs.msg.Vector3, queue_size=queue_size) if cps_enable else None
    if ('magcps' in sysInput):
        pub_magcps = rospy.Publisher('/sensehat/magcps', std_msgs.msg.Float64, queue_size=queue_size) if cps_enable else None
    if ('envall' in sysInput):
        pub_envhmd = rospy.Publisher('/sensehat/envhmd', std_msgs.msg.Float64, queue_size=queue_size) if env_enable else None
        pub_envtph = rospy.Publisher('/sensehat/envtph', std_msgs.msg.Float64, queue_size=queue_size) if env_enable else None
        pub_envtpp = rospy.Publisher('/sensehat/envtpp', std_msgs.msg.Float64, queue_size=queue_size) if env_enable else None
        pub_envprs = rospy.Publisher('/sensehat/envprs', std_msgs.msg.Float64, queue_size=queue_size) if env_enable else None
    if ('joystk' in sysInput):
        pub_joystk = rospy.Publisher('/sensehat/joystick', std_msgs.msg.String, queue_size=queue_size) if joy_enable else None

    rospy.init_node('raspi_sense_hat_sensor', anonymous=True)
    rate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():

        # IMU sensor
        if acc_enable:
            accxyz = sense.get_accelerometer_raw()
            pub_accxyz.publish(std_msgs.msg.Vector3(x=accxyz['x'],y=accxyz['y'],z=accxyz['z']))
            accrpy = sense.get_orientation_degrees()
            pub_accrpy.publish(std_msgs.msg.Vector3(x=accrpy['roll'],y=accrpy['pitch'],z=accrpy['yaw']))
        if gyr_enable:
            gyrxyz = sense.get_gyroscope_raw()
            pub_gyrxyz.publish(std_msgs.msg.Vector3(x=gyrxyz['x'],y=gyrxyz['y'],z=gyrxyz['z']))
            gyrrpy = sense.get_gyroscope()
            pub_gyrrpy.publish(std_msgs.msg.Vector3(x=gyrrpy['roll'],y=gyrrpy['pitch'],z=gyrrpy['yaw']))
        if cps_enable:
            magxyz = sense.get_compass_raw()
            pub_magxyz.publish(std_msgs.msg.Vector3(x=magxyz['x'],y=magxyz['y'],z=magxyz['z']))
            magcps = sense.get_compass()
            pub_magcps.publish(std_msgs.msg.Float64(magcps))

        # Environment sensor
        if env_enable:
            pub_envhmd.publish(std_msgs.msg.Float64(sense.get_humidity()))
            pub_envtph.publish(std_msgs.msg.Float64(sense.get_temperature_from_humidity()))
            pub_envtpp.publish(std_msgs.msg.Float64(sense.get_temperature_from_pressure()))
            pub_envprs.publish(std_msgs.msg.Float64(sense.get_pressure()))

        # Joystick event
        if joy_enable:
            getEvents = sense.stick.get_events()
            for eachEvent in getEvents:
                pub_joystk.publish(eachEvent.direction+'_'+eachEvent.action)

        rate.sleep()
        
if __name__ == '__main__':
    try:
        sensor_acquisition(sys.argv)
    except rospy.ROSInterruptException:
        pass
