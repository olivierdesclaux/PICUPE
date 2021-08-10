from threading import Thread, Timer
from enum import Enum
import cv2 as cv
import pyk4a as k4a
import numpy as np
# Local modules
from utils import stop

class VideoStream:
    def __init__(self, videocapture, flag=""):
        # initialize the file video stream
        self.capture = videocapture
        self.capture.set(cv.CAP_PROP_BUFFERSIZE, 2)
        if "W" in flag:
            self.capture.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
            self.capture.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        # ESSENTIAL to maintain stream FPS for FLIR camera
        self.capture.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.grabbed, self.frame = self.capture.read() 

        # Used to indicate if the thread should be stopped or not
        self.stopped = False
        # Thread used to constantly grab new frames
        Thread(target=self._update, args=(), daemon=True).start()

    def _update(self):
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            else:
                # Grab next frame
                self.grabbed, self.frame = self.capture.read()
                # Ends thread if read() fails
                if not self.grabbed:
                    self.stopped = True                        
    
    def read(self):
        return self.frame

    def stop(self):
        # Stops thread
        self.stopped = True
        self.capture.release()


# Specific VideoStream class for Kinect that returns depth + rgb using pyK4a
class KinectVideoStream:
    def __init__(self):
            # Open Kinect with pyK4a for more options
        self.kinect = k4a.PyK4A(k4a.Config(
                    color_resolution=k4a.ColorResolution.RES_720P,
                    depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                    camera_fps=k4a.FPS.FPS_30,
                    synchronized_images_only=True,
                ))

        self.kinect.start()
        frame  = self.kinect.get_capture()
        self.color = frame.color[:, :, :3]
        self.depth = self._convertDepth(frame.depth)
        # used to indicate if the thread should be stopped or not
        self.stopped = False
        # Thread used to constantly grab new frames
        Thread(target=self.update, args=(), daemon=True).start()

    def update(self):
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            else:
                # Grab next frame
                try:
                    frame  = self.kinect.get_capture()
                    self.color = frame.color
                    self.depth = self._convertDepth(frame.depth)
                except:
                # Ends thread if get_capture() fails
                    self.stopped = True                        
    def read(self):
        return [self.color, self.depth]

    def readRGB(self):
        return self.color

    def readDepth(self):
        return self.depth

    def stop(self):
        # Stops thread
        self.stopped = True
        self.kinect.stop()
    
    def _convertDepth(self, frame, maxValue=4000):
        # Clip numpy array with maxvalue for better contrast
        clipFrame = frame.clip(0, maxValue)
        # Convert 8U values to standard grayscale + normalize values to maximise contrast
        return cv.normalize(clipFrame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U) 

    def colorize(self, frame, colormap=cv.COLORMAP_HSV):
        # Apply colormap to visualize data
        return cv.applyColorMap(frame, colormap)
    
class StreamType(Enum):
    rgb = 0
    ir = 1
    depth = 2

def openStream(targetCamera = None, targetHeight = None, flag = ""):
    # Directly opens specified camera indexes
    if targetCamera is not None:
        # CAP_DSHOW is essential for FLIR framerate
        cap = cv.VideoCapture(targetCamera, cv.CAP_DSHOW)
        if cap.isOpened():
            print("Found camera number", targetCamera)
            return VideoStream(cap, flag)
        else:
            raise Exception("Unable to open camera number " + str(targetCamera))

    # Searches for cameras using height
    elif targetHeight is not None:
        # Starts at first camera
        cameraIndex = 0
        while True:
            # CAP_DSHOW is essential for FLIR framerate
            cap = cv.VideoCapture(cameraIndex, cv.CAP_DSHOW)
            cameraIndex += 1

            if cap.isOpened():
                frameHeight = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
                if frameHeight == targetHeight:
                    print("Found camera with height", targetHeight, "px")
                    return VideoStream(cap, flag)
                else:
                    # Otherwise, releases capture and moves on to next camera
                    cap.release()
            else: 
            # If we reach the end of available cameras
                raise Exception("Unable to open camera with height " + str(targetHeight) + " px")
    # If no camera parameter specificed
    else:
        raise Exception("No camera specified.")

class FPS:
    def __init__(self):
        self.frames = 0 # Counts frames within a second
        self.fps = 0 # Displays FPS for previous second
        self._timer = None # Triggers update every second
        self._isRunning = False # Used to stop Timer
        self.start()

    def _run(self):
        self._isRunning = False
        self.start()
        self._takeFPS()

    def _takeFPS(self):
        self.fps = self.frames
        self.frames = 0

    def start(self):
        if not self._isRunning:
            self._timer = Timer(1, self._run)
            self._timer.start()
            self._isRunning = True

    def stop(self):
        self._isRunning = False
        self._timer.cancel()

