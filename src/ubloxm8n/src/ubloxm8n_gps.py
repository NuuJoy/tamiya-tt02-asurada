#!/usr/bin/env python

'''
rosrun ubloxm8n ubloxm8n_gps.py GGA GSA GLL RMC rate=5
'''

import sys
import rospy
import serial
import pynmea.nmea
from ubloxm8n.msg import gps_gga
from ubloxm8n.msg import gps_gsa
from ubloxm8n.msg import gps_gll
from ubloxm8n.msg import gps_rmc


class serialGPSModule():
    def __init__(self,sysInput):
        #Init parameter with default value
        self.serialport = '/dev/ttyUSB0'
        self.baudrate   = 9600
        self.timeout    = 1.5
        self.queue_size = 1
        self.filter     = {}
        self.rate       = 1
        
        #Override setting from sysInput
        for eachInput in sysInput:
            if eachInput == 'GGA':
                self.filter.update({'GGA':{'timestamp','latitude','longitude','antenna_altitude','horizontal_dil','num_sats'}})
            elif eachInput == 'GSA':
                self.filter.update({'GSA':{'pdop','hdop','vdop'}})
            elif eachInput == 'GLL':
                self.filter.update({'GLL':{'timestamp','lat','lon'}})
            elif eachInput == 'RMC':
                self.filter.update({'RMC':{'datestamp','timestamp','lat','lon','spd_over_grnd'}})
            elif '=' in eachInput:
                name, val = eachInput.split('=')
                if name == 'serialport':
                    self.serialport = val
                elif name == 'baudrate':
                    self.baudrate   = int(val)
                elif name == 'timeout':
                    self.timeout    = float(val)
                elif name == 'queue_size':
                    self.queue_size = int(val)
                elif name == 'rate':
                    self.rate = int(val)
        
        print('Reading GPS signal ---- serialport = {}'.format(self.serialport))
        print('                        baudrate   = {}'.format(self.baudrate))
        print('                        timeout    = {}'.format(self.timeout))
        print('                        queue_size = {}'.format(self.queue_size))
        print('                        filter     = {}'.format(','.join([val for val in self.filter])))
        print('                        rate       = {}'.format(self.rate))
        
        self.ser = serial.Serial(self.serialport,baudrate=self.baudrate,timeout=self.timeout)
        if self.rate == 1:
            self.ser.write(b'\xb5b\x06\x08\x06\x00\xe8\x03\x01\x00\x00\x00\x007')  #change data rate to 1 hz
        elif self.rate == 5:
            self.ser.write(b'\xb5b\x06\x08\x06\x00\xc8\x00\x01\x00\x00\x00\xddh')  #change data rate to 5 hz
        elif self.rate == 10:
            self.ser.write(b'\xb5b\x06\x08\x06\x00d\x00\x01\x00\x00\x00y\x10')     #change data rate to 10 hz

        if 'GGA' in self.filter:
            self.GGA_obj = pynmea.nmea.GPGGA()
            self.pub_gga = rospy.Publisher('/ubloxm8n/gps_gga', gps_gga, queue_size=self.queue_size)
        if 'GSA' in self.filter:
            self.GSA_obj = pynmea.nmea.GPGSA()
            self.pub_gsa = rospy.Publisher('/ubloxm8n/gps_gsa', gps_gsa, queue_size=self.queue_size)
        if 'GLL' in self.filter:
            self.GLL_obj = pynmea.nmea.GPGLL()
            self.pub_gll = rospy.Publisher('/ubloxm8n/gps_gll', gps_gll, queue_size=self.queue_size)
        if 'RMC' in self.filter:
            self.RMC_obj = pynmea.nmea.GPRMC()
            self.pub_rmc = rospy.Publisher('/ubloxm8n/gps_rmc', gps_rmc, queue_size=self.queue_size)
        
        rospy.init_node('ubloxm8n_gps', anonymous=True)
        
    def __enter__(self):
        if not(self.ser.is_open):
            self.ser.open()
        return self
            
    def __exit__(self,typee,value,traceback):
        if self.ser.is_open:
            self.ser.close()
    
    def read(self):
        return self.ser.readline()

    def getData(self,dataType,rawsentence):
        output = {}
        output.update({'dataType':dataType})

        nmeaReadObj = getattr(self,dataType+'_obj')
        nmeaReadObj.parse(rawsentence.decode())
        for eachAttr in nmeaReadObj.__dict__:
            nmeaReadObj_attr = getattr(nmeaReadObj,eachAttr)
            output.update({eachAttr:nmeaReadObj_attr})

        return output

def gps_data_acquisition(gps):
    rawsentence = gps.read()
    dataType = rawsentence[3:6].decode()

    if dataType in gps.filter:
        gpsRead = gps.getData(dataType,rawsentence)
        if dataType == 'GGA':
            pass
            gps.pub_gga.publish(gps_gga(str(gpsRead['timestamp']),
                                        float(gpsRead['latitude']),
                                        float(gpsRead['longitude']),
                                        float(gpsRead['antenna_altitude']),
                                        float(gpsRead['horizontal_dil']),
                                        int(gpsRead['num_sats'])))
        elif dataType == 'GSA':
            gps.pub_gsa.publish(gps_gsa(float(gpsRead['pdop']),
                                        float(gpsRead['hdop']),
                                        float(gpsRead['vdop'].split('*')[0])))
        elif dataType == 'GLL':
            gps.pub_gll.publish(gps_gll(str(gpsRead['timestamp']),
                                        float(gpsRead['lat']),
                                        float(gpsRead['lon'])))
        elif dataType == 'RMC':
            gps.pub_rmc.publish(gps_rmc(str(gpsRead['timestamp']),
                                        str(gpsRead['datestamp']),
                                        float(gpsRead['lat']),
                                        float(gpsRead['lon']),
                                        float(gpsRead['spd_over_grnd'])))

if __name__ == '__main__':
    try:
        with serialGPSModule(sys.argv) as gps:
            gps.read() #readout for bit alignment
            while not(rospy.is_shutdown()):
                try:
                    gps_data_acquisition(gps)
                except KeyboardInterrupt:
                    break
                except:
                    pass
    except rospy.ROSInterruptException:
        pass
