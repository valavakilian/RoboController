import picamera
import picamera.array
import numpy as np
import cv2
import pickle

rList = []
phiList = []
class MotionAnalyser(picamera.array.PiMotionAnalysis):
    def analyse(self, a):
        #return np.mean(a['x']), np.mean(a['y']))
        
        # Calculate the motion vector polar lengths
        r = np.sqrt(
            np.square(a['x'].astype(np.float)) +
            np.square(a['y'].astype(np.float))
            ).clip(0, 255).astype(np.uint8)

        # Calculate the motion vector polar angles
        # arctan values are + or - pi (radians)
        # OpenCV hue values are 0-179 (degrees)
        phi = ((np.arctan2(
              a['y'].astype(np.float),
              a['x'].astype(np.float)) +
              np.pi) * 180/(2*np.pi)
              ).clip(0, 179).astype(np.uint8)
        rList.append(r)
        phiList.append(phi)
        
        #print(np.mean(r), np.mean(phi))
        print(np.mean(a['x'].astype(np.float)), np.mean(a['y'].astype(np.float)))

        # Make an array for a fixed colour saturation
        sat = np.empty_like(r)
        sat[:] = 255 # 100% saturation

        # Assemble the HSV image array
        hsv = cv2.merge((phi,sat,r))

        # Change to the native OpenCV array (BGR)
        bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)

        # Expand the image - 16x zoom
        big=cv2.resize(bgr,None,fx=16,fy=16)
        # Show the image in the window
        cv2.imshow( 'PiMotionAnalysis', big )
        key = cv2.waitKey(100)


with picamera.PiCamera() as camera:
    camera.resolution = (640, 480)
    with MotionAnalyser(camera) as output:
        cv2.namedWindow( 'PiMotionAnalysis' )
        camera.start_recording('/dev/null', 'h264', motion_output=output)
        camera.wait_recording(60) # continue recording for 20 seconds
        camera.stop_recording()
        cv2.destroyWindow('PiMotionAnalysis')
        
        with open('rList', 'wb') as fp:
            pickle.dump(rList, fp)
    
        with open('phiList', 'wb') as fp:
            pickle.dump(phiList, fp)


        
#followAngle(898)
    