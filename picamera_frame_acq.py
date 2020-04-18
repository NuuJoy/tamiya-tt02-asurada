#!/usr/bin/env python

import sys
import rospy
import picamera
import io
import sensor_msgs.msg
import std_msgs.msg

'''
## ---- format
rosrun raspi_picamera picamera_frame_acq.py sensor_mode=1 framerate=15 resolution=[820,466] resize=[640,360] queue_size=5
rosrun raspi_picamera picamera_frame_acq.py sensor_mode=5 resolution=[820,466] framerate=40 resize=None queue_size=1 rotation=90
## ---- picamera senser_mode (from picameraV2 official document) (this script use 5 as default)
#   Resolution  Aspect Ratio      Framerates     Video   Image    FoV     Binning
1   1920x1080       16:9      1/10 <= fps <= 30    x            Partial     None
2   3280x2464        4:3      1/10 <= fps <= 15    x       x      Full      None
3   3280x2464        4:3      1/10 <= fps <= 15    x       x      Full      None
4   1640x1232        4:3      1/10 <= fps <= 40    x              Full      2x2
5   1640x922        16:9      1/10 <= fps <= 40    x              Full      2x2
6   1280x720        16:9         40 < fps <= 90    x            Partial     2x2
7   640x480          4:3         40 < fps <= 90    x            Partial     2x2
## ---- picamera use resolution from sensor_mode if not specified
## ---- picamera set framerate to 30 fps if not specified
## ---- picamera will not apply hardware resizing if not specified
## ---- ros queue_size will set to 1 as default
'''

def camera_acquisition(camera,sysInput):
    #Init parameter with default value
    camera.sensor_mode = 5
    camera.resolution  = [1640,922]
    camera.framerate   = 30
    camera.rotation    = 180
    resize     = None
    queue_size = 1
    #Override setting from sysInput
    for eachInput in sysInput:
        if '=' in eachInput:
            name, val = eachInput.split('=')
            if name == 'sensor_mode':
                camera.sensor_mode = int(val)
            elif (name == 'resolution') and (val != 'None'):
                camera.resolution  = [int(v) for v in val.strip('[]').split(',')]
            elif name == 'framerate':
                camera.framerate   = int(val)
            elif name == 'rotation':
                camera.rotation    = int(val)
            elif (name == 'resize') and (val != 'None'):
                resize     = [int(v) for v in val.strip('[]').split(',')]
            elif name == 'queue_size':
                queue_size = int(val)

    print('Start capturing ---- sensormode: {}'.format(camera.sensor_mode))
    print('                     resolution: {}'.format(camera.resolution))
    print('                     framerate : {}'.format(camera.framerate))
    print('                     resize    : {}'.format(resize))
    print('                     queue_size: {}'.format(queue_size))

    #streaming-data setting
    stream  = io.BytesIO()
    imgfrmt = 'jpeg'
    header  = std_msgs.msg.Header()
    pub_frm = rospy.Publisher('/picamera/frame', sensor_msgs.msg.CompressedImage, queue_size=queue_size)
    rospy.init_node('raspi_picamera_frame', anonymous=True)

    rate = rospy.Rate(camera.framerate)

    for _ in camera.capture_continuous(stream, format=imgfrmt, resize=resize, use_video_port=True):
        if rospy.is_shutdown(): break
        
        header.stamp = rospy.Time.now()
        pub_frm.publish(header,imgfrmt,stream.getvalue())
        stream.seek(0)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        with picamera.PiCamera() as camera:
            camera_acquisition(camera,sys.argv)
    except rospy.ROSInterruptException:
        camera.close()
