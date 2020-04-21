#!/usr/bin/env python

import rospy
import std_msgs.msg
from ds4_driver.msg import Feedback, Status
from pca9685.msg import pwm_status
from ubloxm8n.msg import gps_gga
from ubloxm8n.msg import gps_rmc

import sys
import time
import numpy

import vispy.scene
import vispy.app
import vispy.visuals.transforms


canvas = vispy.scene.SceneCanvas(keys='interactive')
canvas.size = 1200, 600
canvas.show()

grid = canvas.central_widget.add_grid()

# -------- actuator monitor
# build subplot
chassis_subplot = grid.add_view(row=0, col=0, row_span=1, col_span=1)
chassis_subplot.border_color = (0.5, 0.5, 0.5, 1)
chassis_subplot.camera = vispy.scene.PanZoomCamera(rect=(0,-1.1,49,2.2),interactive=False)
vispy.scene.visuals.GridLines(parent=chassis_subplot.scene)
# add data line plot
driveCtrl_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[1,0,0], antialias=False, method='gl')
drivePcnt_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[0,1,0], antialias=False, method='gl')
steerCtrl_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[0,1,1], antialias=False, method='gl')
steerPcnt_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[1,0,1], antialias=False, method='gl')
chassis_subplot.add(driveCtrl_line)
chassis_subplot.add(drivePcnt_line)
chassis_subplot.add(steerCtrl_line)
chassis_subplot.add(steerPcnt_line)

# -------- camera control monitor
# build subplot
camcontrol_subplot = grid.add_view(row=0, col=1, row_span=1, col_span=1)
camcontrol_subplot.border_color = (0.5, 0.5, 0.5, 1)
camcontrol_subplot.camera = vispy.scene.PanZoomCamera(rect=(0,-1.1,49,2.2),interactive=False)
vispy.scene.visuals.GridLines(parent=camcontrol_subplot.scene)
# add data line plot
panPcnt_line   = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[0,1,1], antialias=False, method='gl')
tiltPcnt_line  = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[1,1,0], antialias=False, method='gl')
camcontrol_subplot.add(panPcnt_line)
camcontrol_subplot.add(tiltPcnt_line)

# -------- imu accelerometer monitor
# build subplot
accelerometer_subplot = grid.add_view(row=1, col=0, row_span=1, col_span=1)
accelerometer_subplot.border_color = (0.5, 0.5, 0.5, 1)
accelerometer_subplot.camera = vispy.scene.PanZoomCamera(rect=(0,-1.1,49,2.2),interactive=False)
vispy.scene.visuals.GridLines(parent=accelerometer_subplot.scene)
# add data line plot
acclx_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                     color=[1,0,0], antialias=False, method='gl')
accly_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                      color=[0,1,0], antialias=False, method='gl')
acclz_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                      color=[0,0,1], antialias=False, method='gl')
accelerometer_subplot.add(acclx_line)
accelerometer_subplot.add(accly_line)
accelerometer_subplot.add(acclz_line)

# -------- imu gyroscope monitor
# build subplot
gyroscope_subplot = grid.add_view(row=1, col=1, row_span=1, col_span=1)
gyroscope_subplot.border_color = (0.5, 0.5, 0.5, 1)
gyroscope_subplot.camera = vispy.scene.PanZoomCamera(rect=(0,-1.1,49,2.2),interactive=False)
vispy.scene.visuals.GridLines(parent=gyroscope_subplot.scene)
# add data line plot
gyrox_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                     color=[0,1,1], antialias=False, method='gl')
gyroy_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                     color=[1,0,1], antialias=False, method='gl')
gyroz_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                     color=[1,1,0], antialias=False, method='gl')
gyroscope_subplot.add(gyrox_line)
gyroscope_subplot.add(gyroy_line)
gyroscope_subplot.add(gyroz_line)

# -------- ultrasonic monitor
# build subplot
ultrasonic_subplot = grid.add_view(row=2, col=0, row_span=1, col_span=1)
ultrasonic_subplot.border_color = (0.5, 0.5, 0.5, 1)
ultrasonic_subplot.camera = vispy.scene.PanZoomCamera(rect=(0,-100,49,1200),interactive=False)
vispy.scene.visuals.GridLines(parent=ultrasonic_subplot.scene)
# add data line plot
frntultra_line = vispy.scene.visuals.Line(pos=numpy.array([[time,randVal] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[1,1,0], antialias=False, method='gl')
rearultra_line = vispy.scene.visuals.Line(pos=numpy.array([[time,randVal] for time,randVal in zip(range(50),numpy.zeros(51))]),
                                          color=[1,0,1], antialias=False, method='gl')
ultrasonic_subplot.add(frntultra_line)
ultrasonic_subplot.add(rearultra_line)

# -------- magnetometer monitor
# build subplot
magnetometer_subplot = grid.add_view(row=2, col=1, row_span=1, col_span=1)
magnetometer_subplot.border_color = (0.5, 0.5, 0.5, 1)
vispy.scene.visuals.GridLines(parent=magnetometer_subplot.scene)
magnetometer_subplot.camera = 'turntable'
magnetometer_subplot.camera.interactive = False
magnetometer_subplot.camera.center = [0,0,0]
magnetometer_subplot.camera.distance = 3.0
# add data line plot
sensemagnetraw  = [ 1.0, 1.0, 1.0]
ubloxmagnetraw  = [-1.0,-1.0,-1.0]
sensemagnetplot = numpy.array([[0,0,0],[sensemagnetraw[0],sensemagnetraw[1],0],sensemagnetraw,[0,0,0]])
ubloxmagnetplot = numpy.array([[0,0,0],[ubloxmagnetraw[0],ubloxmagnetraw[1],0],ubloxmagnetraw,[0,0,0]])
sensemagnet_line = vispy.scene.visuals.Line(pos=sensemagnetplot,
                                            color=[1,0,0], antialias=True, method='gl')
ubloxmagnet_line = vispy.scene.visuals.Line(pos=ubloxmagnetplot,
                                            color=[0,1,0], antialias=True, method='gl')
magnetometer_subplot.add(sensemagnet_line)
magnetometer_subplot.add(ubloxmagnet_line)

# -------- map monitor
# build subplot
map_subplot = grid.add_view(row=0, col=2, row_span=3, col_span=3)
map_subplot.border_color = (0.5, 0.5, 0.5, 1)
vispy.scene.visuals.GridLines(parent=map_subplot.scene)
map_subplot.camera = 'turntable'
map_subplot.camera.center = [0,0,0]
map_subplot.camera.distance = 10.0
# add data line plot
path_line = vispy.scene.visuals.Line(pos=numpy.array([[0,0,0]]),
                                     color=[0.75,0.75,0.75], antialias=True, method='gl')
pstn_scat = vispy.scene.visuals.Markers(pos=numpy.array([[0,0,0]]), 
                                        edge_color=numpy.array([[0,0,0]]), 
                                        face_color=numpy.array([[0,0,0]]), 
                                        size=numpy.array([[0]]))
map_subplot.add(path_line)
map_subplot.add(pstn_scat)

# -------- current state monitor
# build subplot
curstate_subplot = grid.add_view(row=2, col=2, row_span=1, col_span=1)
curstate_subplot.border_color = (0.5, 0.5, 0.5, 1)
vispy.scene.visuals.GridLines(parent=curstate_subplot.scene)
curstate_subplot.camera.interactive = False
curstate_subplot.bgcolor = 'black'
curstate_subplot.camera  = 'turntable'
curstate_subplot.camera.center   = [0,0,0]
curstate_subplot.camera.distance = 20.0
# add data line plot
roverstate_box1 = vispy.scene.visuals.Box(width=3, height=1, depth=5,
                                         color=[1.0,0.5,0.0],
                                         edge_color="black",
                                         parent=curstate_subplot.scene)
roverstate_box2 = vispy.scene.visuals.Box(width=3, height=1, depth=1,
                                         color=[1.0,0.0,0.0],
                                         edge_color="black",
                                         parent=curstate_subplot.scene)
roverstate_box3 = vispy.scene.visuals.Box(width=2.5, height=0.5, depth=3,
                                         color=[0.8,0.8,0.8],
                                         edge_color="black",
                                         parent=curstate_subplot.scene)
curstate_subplot.add(roverstate_box1)
curstate_subplot.add(roverstate_box2)
curstate_subplot.add(roverstate_box3)
roverstate_box1.transform = vispy.visuals.transforms.MatrixTransform()
roverstate_box2.transform = vispy.visuals.transforms.MatrixTransform()
roverstate_box3.transform = vispy.visuals.transforms.MatrixTransform()
roverstate_box1.transform.translate([0,-0.5,0])
roverstate_box2.transform.translate([0,2.5,0])
roverstate_box3.transform.translate([0,-0.5,0.75])
class roverStateObj():
    """ rover rotating class, make it easier to transform group of box """
    def __init__(self,box1,box2,box3):
        self.box1, self.box2, self.box3 = box1, box2, box3
        self.lastRoll, self.lastPitch, self.lastYaw = 0,0,0
    def reRotation(self,roll,pitch,yaw):
        for tripleBox in [self.box1,self.box2,self.box3]:
            tripleBox.transform.rotate(-self.lastYaw,  [0,0,1]) # unrotate yaw
            tripleBox.transform.rotate(-self.lastPitch,[1,0,0]) # unrotate pitch
            tripleBox.transform.rotate(-self.lastRoll, [0,1,0]) # unrotate roll
            tripleBox.transform.rotate(roll, [0,1,0]) # rerotate roll
            tripleBox.transform.rotate(pitch,[1,0,0]) # rerotate pitch
            tripleBox.transform.rotate(roll, [0,0,1]) # rerotate yaw
        self.lastRoll  = roll
        self.lastPitch = pitch
        self.lastYaw   = yaw
roverstate = roverStateObj(roverstate_box1,roverstate_box2,roverstate_box3)
roverstate.reRotation(360*(numpy.random.random()-0.5),360*(numpy.random.random()-0.5),360*(numpy.random.random()-0.5))

class graphDataUpdateModule():
    """ vispy graph updater, this object will do both data-update and graphical-update """
    def __init__(self,chassis_subplot,driveCtrl_line,drivePcnt_line,steerCtrl_line,steerPcnt_line,\
                      camcontrol_subplot,panPcnt_line,tiltPcnt_line,\
                      accelerometer_subplot,acclx_line,accly_line,acclz_line,\
                      gyroscope_subplot,gyrox_line,gyroy_line,gyroz_line,\
                      ultrasonic_subplot,frntultra_line,rearultra_line,\
                      magnetometer_subplot,sensemagnet_line,ubloxmagnet_line,\
                      map_subplot,path_line,pstn_scat,\
                      curstate_subplot,roverstate):
        # -------- Init self data store
        self.plotDataNum     = 50
        self.positionDataNum = 600
        self.pathDataNum     = 36000
        # controller
        self.driveCtrl_data,self.steerCtrl_data = self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        # chassis
        self.drivePcnt_data,self.steerPcnt_data = self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        self.panPcnt_data,self.tiltPcnt_data    = self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        # sensehat
        self.acclx_data,self.accly_data,self.acclz_data = self.plotDataNum*[0.0],self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        self.gyrox_data,self.gyroy_data,self.gyroz_data = self.plotDataNum*[0.0],self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        self.mgntx_data,self.mgnty_data,self.mgntz_data = self.plotDataNum*[0.0],self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        self.roll_data,self.pitch_data ,self.yaw_data   = 0.0,0.0,0.0
        #ultrasonic
        self.frntultra_data,self.rearultra_data = self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        #ublox
        self.posx_data,self.posy_data,self.posz_data,self.velo_data = [],[],[],[]
        self.cmpsx_data,self.cmpsy_data,self.cmpsz_data             = self.plotDataNum*[0.0],self.plotDataNum*[0.0],self.plotDataNum*[0.0]
        #heading
        self.heading = None
        self.pathVal = None
        self.pstnVal = None
        self.faceclr = None

        # -------- Init data-plot handle
        self.driveCtrl_line   = driveCtrl_line
        self.steerCtrl_line   = steerCtrl_line
        self.drivePcnt_line   = drivePcnt_line
        self.steerPcnt_line   = steerPcnt_line
        self.panPcnt_line     = panPcnt_line
        self.tiltPcnt_line    = tiltPcnt_line
        self.acclx_line       = acclx_line
        self.accly_line       = accly_line
        self.acclz_line       = acclz_line
        self.gyrox_line       = gyrox_line
        self.gyroy_line       = gyroy_line
        self.gyroz_line       = gyroz_line
        self.sensemagnet_line = sensemagnet_line
        self.ubloxmagnet_line = ubloxmagnet_line
        self.roverstate       = roverstate
        self.frntultra_line   = frntultra_line
        self.rearultra_line   = rearultra_line
        self.path_line        = path_line
        self.pstn_scat        = pstn_scat

        # -------- Init subplot handle
        self.map_subplot      = map_subplot

    def controllerDataUpdate(self,ds4Status):
        if ds4Status.axis_l2 > 0:
            setDrive = ds4Status.axis_l2
        else:
            setDrive = -1.0*ds4Status.axis_r2
        setSteer = ds4Status.axis_right_x

        # update self data store
        self.driveCtrl_data = self.driveCtrl_data[-49:]+[setDrive]
        self.steerCtrl_data = self.steerCtrl_data[-49:]+[setSteer]

        # update vispy graph
        driveCtrl_line.set_data(numpy.transpose([range(50),self.driveCtrl_data],[1,0]))
        steerCtrl_line.set_data(numpy.transpose([range(50),self.steerCtrl_data],[1,0]))

    def chassisDataUpdate(self,pwmStatus):
        readDrivePcnt_data = pwmStatus.current_drive
        readSteerPcnt_data = pwmStatus.current_steer
        readPanPcnt_data   = pwmStatus.current_picamx
        readTiltPcnt_data  = pwmStatus.current_picamy

        # update self data store
        self.drivePcnt_data = self.drivePcnt_data[-49:]+[readDrivePcnt_data]
        self.steerPcnt_data = self.steerPcnt_data[-49:]+[readSteerPcnt_data]
        self.panPcnt_data   = self.panPcnt_data[-49:]+[readPanPcnt_data]
        self.tiltPcnt_data  = self.tiltPcnt_data[-49:]+[readTiltPcnt_data]

        # update vispy graph
        self.drivePcnt_line.set_data(numpy.transpose([range(50),self.drivePcnt_data],[1,0]))
        self.steerPcnt_line.set_data(numpy.transpose([range(50),self.steerPcnt_data],[1,0]))
        self.panPcnt_line.set_data(numpy.transpose([range(50),self.panPcnt_data],[1,0]))
        self.tiltPcnt_line.set_data(numpy.transpose([range(50),self.tiltPcnt_data],[1,0]))

    def sensehatAccXYZUpdate(self,sensehat_accxyz):
        # update self data store (accelrometer)
        self.acclx_data = self.acclx_data[-49:]+[sensehat_accxyz.x]
        self.accly_data = self.accly_data[-49:]+[sensehat_accxyz.y]
        self.acclz_data = self.acclz_data[-49:]+[sensehat_accxyz.z]
        # update vispy graph (accelrometer)
        self.acclx_line.set_data(numpy.transpose([range(50),self.acclx_data],[1,0]))
        self.accly_line.set_data(numpy.transpose([range(50),self.accly_data],[1,0]))
        self.acclz_line.set_data(numpy.transpose([range(50),self.acclz_data],[1,0]))

    def sensehatAccRPYUpdate(self,sensehat_accrpy):
        # update self data store (self-state)
        self.roll_data  = sensehat_accrpy.x
        self.pitch_data = sensehat_accrpy.y
        self.yaw_data   = sensehat_accrpy.z
        # update vispy graph (self-state)
        self.roverstate.reRotation(self.roll_data,self.pitch_data,self.yaw_data)

    def sensehatGyrXYZUpdate(self,sensehat_gyrxyz):
        # update self data store (gyroscope)
        self.gyrox_data = self.gyrox_data[-49:]+[sensehat_gyrxyz.x]
        self.gyroy_data = self.gyroy_data[-49:]+[sensehat_gyrxyz.y]
        self.gyroz_data = self.gyroz_data[-49:]+[sensehat_gyrxyz.z]
        # update vispy graph (gyroscope)
        self.gyrox_line.set_data(numpy.transpose([range(50),self.gyrox_data],[1,0]))
        self.gyroy_line.set_data(numpy.transpose([range(50),self.gyroy_data],[1,0]))
        self.gyroz_line.set_data(numpy.transpose([range(50),self.gyroz_data],[1,0]))

    def sensehatMagXYZUpdate(self,sensehat_magxyz):
        # update self data store (sensehat magnetometer)
        self.mgntx_data = sensehat_magxyz.x
        self.mgnty_data = sensehat_magxyz.y
        self.mgntz_data = sensehat_magxyz.z
        # update vispy graph (sensehat magnetometer)
        self.sensemagnet_line.set_data(numpy.array([[0,0,0],[self.mgntx_data,self.mgnty_data,0],[self.mgntx_data,self.mgnty_data,self.mgntz_data],[0,0,0]]))

    def ultrasonicFrontDataUpdate(self,ultrasonicFront):
        # update self data store
        self.frntultra_data = self.frntultra_data[-49:]+[ultrasonicFront.data]
        # update vispy graph
        frntultra_line.set_data(numpy.transpose([range(50),self.frntultra_data],[1,0]))

    def ultrasonicRearDataUpdate(self,ultrasonicRear):
        # update self data store
        self.rearultra_data = self.rearultra_data[-49:]+[ultrasonicRear.data]
        # update vispy graph
        rearultra_line.set_data(numpy.transpose([range(50),self.rearultra_data],[1,0]))

    def ubloxGPSPositionDataUpdate(self,ublox_gpsgga):
        # update self data store
        self.posx_data = self.posx_data + [ublox_gpsgga.longitude]
        self.posy_data = self.posy_data + [ublox_gpsgga.latitude]
        self.posz_data = self.posz_data + [ublox_gpsgga.antenna_altitude]
        # update path
        self.pathVal = numpy.transpose([self.posx_data[-self.pathDataNum:],
                                        self.posy_data[-self.pathDataNum:],
                                        self.posz_data[-self.pathDataNum:]],[1,0])
        self.pstnVal = numpy.transpose([self.posx_data[-self.positionDataNum:],
                                        self.posy_data[-self.positionDataNum:],
                                        self.posz_data[-self.positionDataNum:]],[1,0])
        # update vispy graph
        self.path_line.set_data(pos=self.pathVal)
        # update map view
        self.map_subplot.camera.center   = numpy.mean(self.pathVal,axis=0)
        self.map_subplot.camera.distance = 2.0*numpy.max(self.pathVal)-numpy.min(self.pathVal)
        # update scatter graph
        dataIndx = min([len(self.pstnVal),len(self.faceclr)])
        self.pstn_scat.set_data(pos=self.pstnVal[:dataIndx],edge_color=None,face_color=self.faceclr[:dataIndx],size=10)

    def ubloxGPSVelocityDataUpdate(self,ublox_gpsrmc):
        # update self data store
        self.velo_data = self.velo_data + [ublox_gpsrmc.spd_over_grnd]
        # update scatter color
        fclrVal = [val/50.0 if val <= 50.0 else 1.0 for val in self.velo_data[-self.positionDataNum:]]
        self.faceclr = numpy.array([[val,0.0,1-val] for val in fclrVal])
        # update scatter graph
        dataIndx = min([len(self.pstnVal),len(self.faceclr)])
        self.pstn_scat.set_data(pos=self.pstnVal[:dataIndx],edge_color=None,face_color=self.faceclr[:dataIndx],size=10)

    def ubloxCompassDataUpdate(self,ublox_magxyz):
        # update self data store (ublox magnetometer)
        self.cmpsx_data = ublox_magxyz.x
        self.cmpsy_data = ublox_magxyz.y
        self.cmpsz_data = ublox_magxyz.z
        # update vispy graph (ublox magnetometer)
        self.ubloxmagnet_line.set_data(numpy.array([[0,0,0],[self.cmpsx_data,self.cmpsy_data,0],[self.cmpsx_data,self.cmpsy_data,self.cmpsz_data],[0,0,0]]))

graph = graphDataUpdateModule(chassis_subplot,driveCtrl_line,drivePcnt_line,steerCtrl_line,steerPcnt_line,\
                              camcontrol_subplot,panPcnt_line,tiltPcnt_line,\
                              accelerometer_subplot,acclx_line,accly_line,acclz_line,\
                              gyroscope_subplot,gyrox_line,gyroy_line,gyroz_line,\
                              ultrasonic_subplot,frntultra_line,rearultra_line,\
                              magnetometer_subplot,sensemagnet_line,ubloxmagnet_line,\
                              map_subplot,path_line,pstn_scat,\
                              curstate_subplot,roverstate)

if __name__ == '__main__' and sys.flags.interactive == 0:
    rospy.init_node('vispy_graph', anonymous=True)

    rospy.Subscriber('/status', Status, graph.controllerDataUpdate, queue_size=1)
    rospy.Subscriber('/pca9685/pwm_status', pwm_status, graph.chassisDataUpdate, queue_size=1)

    rospy.Subscriber('/sensehat/accxyz', std_msgs.msg.Vector3, graph.sensehatAccXYZUpdate, queue_size=1)
    rospy.Subscriber('/sensehat/accrpy', std_msgs.msg.Vector3, graph.sensehatAccRPYUpdate, queue_size=1)
    rospy.Subscriber('/sensehat/gyrxyz', std_msgs.msg.Vector3, graph.sensehatGyrXYZUpdate, queue_size=1)
    rospy.Subscriber('/sensehat/magxyz', std_msgs.msg.Vector3, graph.sensehatMagXYZUpdate, queue_size=1)

    rospy.Subscriber('/ultrasonic/front', std_msgs.msg.Float64, graph.ultrasonicFrontDataUpdate, queue_size=1)
    rospy.Subscriber('/ultrasonic/rear',  std_msgs.msg.Float64, graph.ultrasonicRearDataUpdate, queue_size=1)

    rospy.Subscriber('/ubloxm8n/gps_gga', gps_gga, graph.ubloxGPSPositionDataUpdate, queue_size=1)
    rospy.Subscriber('/ubloxm8n/gps_rmc', gps_rmc, graph.ubloxGPSVelocityDataUpdate, queue_size=1)
    rospy.Subscriber('/ubloxm8n/magxyz',  std_msgs.msg.Vector3, graph.ubloxCompassDataUpdate, queue_size=1)
    
    try:
        vispy.app.run()
    except rospy.ROSInterruptException:
        pass
