#!/usr/bin/env python

import sys
import rospy
import RPi.GPIO
import time
import std_msgs.msg
from std_msgs.msg import Float64

class echoFeedback():
    def __init__(self,trig_pin,echo_pin,timeout,pubobj):
        # Pin setup
        self.trig_pin   = trig_pin
        self.echo_pin   = echo_pin
        self.timeout    = timeout
        self.pubobj     = pubobj

        self.temperature_tph = 30.0
        self.temperature_tpp = 30.0
        self.humidity_pcnt   = 70.0
        
        # Initial RPi.GPIO
        RPi.GPIO.setmode(RPi.GPIO.BOARD)
        RPi.GPIO.setwarnings(False)
        
    def __enter__(self):
        RPi.GPIO.setup(self.trig_pin, RPi.GPIO.OUT, initial=RPi.GPIO.LOW)
        RPi.GPIO.setup(self.echo_pin, RPi.GPIO.IN,  pull_up_down=RPi.GPIO.PUD_DOWN)
        return self
    
    def __exit__(self ,type, value, traceback):
        RPi.GPIO.remove_event_detect(self.echo_pin)
        RPi.GPIO.cleanup([self.trig_pin,self.echo_pin])

    def distanceMeasure(self):
        StartTime = None
        StopTime  = None
        RPi.GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        RPi.GPIO.output(self.trig_pin, False)
        
        loopTime = time.time()
        while time.time()-loopTime < self.timeout:
            if RPi.GPIO.input(self.echo_pin):
                StartTime = time.time()
                break
        
        loopTime = time.time()
        while time.time()-loopTime < self.timeout:
            if not(RPi.GPIO.input(self.echo_pin)):
                StopTime = time.time()
                break
        
        if StartTime and StopTime:
            TimeElapsed = StopTime - StartTime
            soundSpeed  = (0.00085*(273.0+(self.temperature_tph+self.temperature_tpc)/2.0)-0.232677)*self.humidity_pcnt +\
                           0.5930736*(273.0+(self.temperature_tph+self.temperature_tpc)/2.0)+169.262723
            distance_mm = 1000.0*(TimeElapsed/2.0)*soundSpeed

            print('distancemeasure: ',distance_mm)

            self.pubobj.publish(distance_mm)
        
class ultrasonicEnvUpdate():
    def __init__(self,sensor):
        self.sensor = sensor

    def envhmd_update(self,envhmd):
        for eachSensor in self.sensor:
            eachSensor.humidity = envhmd.data

    def envtph_update(self,envtph):
        for eachSensor in self.sensor:
            eachSensor.temperature_tph = envtph.data

    def envtpp_update(self,envtpp):
        for eachSensor in self.sensor:
            eachSensor.temperature_tpp = envtpp.data

    def attr_update(self,attrName,attrVal):
        for eachSensor in self.sensor:
            setattr(eachSensor,attrName,attrVal)

if __name__ == '__main__':

    #Init parameter with default value
    rate_hz       = 10
    queue_size    = 1
    frnt_trig_pin = 38
    frnt_echo_pin = 40
    rear_trig_pin = 35
    rear_echo_pin = 37
    timeout       = 0.01

    #Override sensor object setting from sysInput
    for eachInput in sys.argv:
        if '=' in eachInput:
            name, val = eachInput.split('=')
            if name == 'rate_hz':
                rate_hz = int(val)
            elif name == 'queue_size':
                queue_size = int(val)
            elif name == 'timeout':
                rear_echo_pin = float(val)
            elif name == 'frnt_trig_pin':
                frnt_trig_pin = int(val)
            elif name == 'frnt_echo_pin':
                frnt_echo_pin = int(val)
            elif name == 'rear_trig_pin':
                rear_trig_pin = int(val)
            elif name == 'rear_echo_pin':
                rear_echo_pin = int(val)
                    
    print('UltraSonicSensor start ---- rate_hz      : {}'.format(rate_hz))
    print('                            queue_size   : {}'.format(queue_size))
    print('                            timeout      : {}'.format(timeout))
    print('                            frnt_trig_pin: {}'.format(frnt_trig_pin))
    print('                            frnt_echo_pin: {}'.format(frnt_echo_pin))
    print('                            rear_trig_pin: {}'.format(rear_trig_pin))
    print('                            rear_echo_pin: {}'.format(rear_echo_pin))

    rospy.init_node('ultrasonic_node', anonymous=True)
    pub_frnt = rospy.Publisher('/ultrasonic/front', std_msgs.msg.Float64, queue_size=queue_size)
    pub_rear = rospy.Publisher('/ultrasonic/rear',  std_msgs.msg.Float64, queue_size=queue_size)

    try:
        with echoFeedback(frnt_trig_pin,frnt_echo_pin,timeout,pub_frnt) as frnt_sensor,\
             echoFeedback(rear_trig_pin,rear_echo_pin,timeout,pub_rear) as rear_sensor:

            envUpdater = ultrasonicEnvUpdate([frnt_sensor,rear_sensor])
            envUpdater.attr_update('timeout',timeout)

            rospy.Subscriber('/sensehat/envhmd', Float64, envUpdater.envhmd_update, queue_size=1)
            rospy.Subscriber('/sensehat/envtph', Float64, envUpdater.envtph_update, queue_size=1)
            rospy.Subscriber('/sensehat/envtpp', Float64, envUpdater.envtpp_update, queue_size=1)
            
            rate = rospy.Rate(rate_hz)
            while not rospy.is_shutdown():
                frnt_sensor.distanceMeasure()
                rear_sensor.distanceMeasure()
                rate.sleep()

    except rospy.ROSInterruptException:
        pass
