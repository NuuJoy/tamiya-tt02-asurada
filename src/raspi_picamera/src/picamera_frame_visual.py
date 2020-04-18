#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy
import sensor_msgs.msg

'''
## ---- format
rosrun raspi_picamera picamera_frame_visual.py fullscreen resize=screen figurename=FrontCamera
'''

class cmprsdImage_visualize():
    def __init__(self,sysInput):
        #Init parameter with default value
        self.figurename = 'raspi_camera_display'
        self.fullscreen = False
        self.abortFlag  = False
        self.resize     = None
        #Override setting from sysInput
        for eachInput in sysInput:
            if '=' in eachInput:
                name, val = eachInput.split('=')
                if name == 'figurename':
                    self.figurename = val
                elif name == 'resize':
                    if val == 'screen':
                        import subprocess
                        screenResolution = [int(val) for val in subprocess.check_output(['fbset','-s']).decode().split('\n')[2].split()[1:3]]
                        self.resize      = tuple(screenResolution)
                    else:
                        self.resize = tuple([int(v) for v in val.strip('[]').split(',')])
            elif eachInput == 'fullscreen':
                self.fullscreen = True

        print('Live video start ---- figurename: {}'.format(self.figurename))
        print('                      fullscreen: {}'.format(self.fullscreen))
        print('                      resize    : {}'.format(self.resize))

        #Initialize cv2 window
        cv2.namedWindow(self.figurename, cv2.WINDOW_NORMAL)
        if self.fullscreen:
            cv2.setWindowProperty(self.figurename,cv2.WND_PROP_FULLSCREEN,cv2.WINDOW_FULLSCREEN)

    def plot(self,imgMessage):
        
        nparr  = numpy.frombuffer(imgMessage.data, numpy.uint8)
        decimg = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        if self.resize:
            cv2.imshow(self.figurename, cv2.resize(decimg,self.resize))
        else:
            cv2.imshow(self.figurename, decimg)

        if cv2.waitKey(1) != 255:
            self.abortFlag = True

if __name__ == '__main__':
    try:
        imgvisual = cmprsdImage_visualize(sys.argv)
        
        rospy.init_node('raspi_picamera_visual', anonymous=True)
        rospy.Subscriber('/picamera/frame', sensor_msgs.msg.CompressedImage, imgvisual.plot, queue_size=1)
        
        while not(imgvisual.abortFlag) and not(rospy.core.is_shutdown()):
            rospy.rostime.wallsleep(0.5)

    except rospy.ROSInterruptException:
        pass
    
    finally:
        cv2.destroyAllWindows()
