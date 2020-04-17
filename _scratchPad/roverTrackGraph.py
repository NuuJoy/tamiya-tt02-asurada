import sys
import numpy
import vispy.scene
import vispy.app
import vispy.visuals.transforms

"""
┌ ┐ └ ┘ ├ ┤ ┬ ┴ ┼ ─ │ ┄ ┆
┌────────────┬────────────┬────────────────────────────────────┐
│  chassis   │ camcontrol │                                    │
├────────────┼────────────┤                                    │
│  IMU,Gyro  │  IMU,accl  │                   Map area         │
├────────────┼────────────┼────────────┐                       │
│ ultrasonic │  2 magnet  │  curState  │                       │
└────────────┴────────────┴────────────┴───────────────────────┘
"""

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
driveCtrl_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                          color=[1,0,0], antialias=False, method='gl')
drivePcnt_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                          color=[0,1,0], antialias=False, method='gl')
steerCtrl_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                          color=[0,1,1], antialias=False, method='gl')
steerPcnt_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
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
panPcnt_line   = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                          color=[0,1,1], antialias=False, method='gl')
tiltPcnt_line  = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
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
acclx_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                     color=[1,0,0], antialias=False, method='gl')
accly_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                      color=[0,1,0], antialias=False, method='gl')
acclz_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
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
gyrox_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                     color=[0,1,1], antialias=False, method='gl')
gyroy_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
                                     color=[1,0,1], antialias=False, method='gl')
gyroz_line = vispy.scene.visuals.Line(pos=numpy.array([[time,2.0*(randVal-0.5)] for time,randVal in zip(range(50),numpy.random.random(51))]),
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
frntultra_line = vispy.scene.visuals.Line(pos=numpy.array([[time,randVal] for time,randVal in zip(range(50),1000.0*numpy.random.random(51))]),
                                          color=[1,1,0], antialias=False, method='gl')
rearultra_line = vispy.scene.visuals.Line(pos=numpy.array([[time,randVal] for time,randVal in zip(range(50),1000.0*numpy.random.random(51))]),
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
sensemagnetraw  = [2.0*(0.5-numpy.random.random()),2.0*(0.5-numpy.random.random()),2.0*(0.5-numpy.random.random())]
ubloxmagnetraw  = [2.0*(0.5-numpy.random.random()),2.0*(0.5-numpy.random.random()),2.0*(0.5-numpy.random.random())]
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
path_line = vispy.scene.visuals.Line(pos=numpy.random.normal(scale=(1,1,0.1),size=(100,3)),
                                     color=[0.75,0.75,0.75], antialias=True, method='gl')
pstn_scat = vispy.scene.visuals.Markers(pos=numpy.random.normal(scale=(1,1,0.1),size=(100,3)), 
                                        edge_color=numpy.random.random(size=(100,3)), 
                                        face_color=numpy.random.random(size=(100,3)), 
                                        size=15.0*numpy.random.random(size=100))
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
    def __init__(self,chassis_subplot,driveCtrl_line,drivePcnt_line,steerCtrl_line,steerPcnt_line,\
                      camcontrol_subplot,panPcnt_line,tiltPcnt_line,\
                      accelerometer_subplot,acclx_line,accly_line,acclz_line,\
                      gyroscope_subplot,gyrox_line,gyroy_line,gyroz_line,\
                      ultrasonic_subplot,frntultra_line,rearultra_line,\
                      magnetometer_subplot,sensemagnet_line,ubloxmagnet_line,\
                      map_subplot,path_line,pstn_scat,\
                      curstate_subplot,roverstate):
        # Init all self data store
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

    def controllerDataUpdate(self,newData):
        self.driveCtrl_data = None
        self.steerCtrl_data = None

    def chassisDataUpdate(self,newData):
        self.drivePcnt_data = None
        self.steerPcnt_data = None
        self.panPcnt_data   = None
        self.tiltPcnt_data  = None

    def sensehatDataUpdate(self,newData):
        self.acclx_data = None
        self.accly_data = None
        self.acclz_data = None
        self.gyrox_data = None
        self.gyroy_data = None
        self.gyroz_data = None
        self.mgntx_data = None
        self.mgnty_data = None
        self.mgntz_data = None
        self.roll_data  = None
        self.pitch_data = None
        self.yaw_data   = None

    def ultrasonicDataUpdate(self,newData):
        self.frntultra_data = None
        self.rearultra_data = None

    def ubloxGPSDataUpdate(self,newData):
        self.posx_data = None
        self.posy_data = None
        self.posz_data = None
        self.velo_data = None

    def ubloxCompassDataUpdate(self,newData):
        self.cmpsx_data = None
        self.cmpsy_data = None
        self.cmpsz_data = None

#timer = vispy.app.Timer()                  <<<<
#timer.connect(graphHandle.graphUpdate) #   <<<< or update on ros subscribe????
#timer.start(0.1)                           <<<<

if __name__ == '__main__' and sys.flags.interactive == 0:
    vispy.app.run()
