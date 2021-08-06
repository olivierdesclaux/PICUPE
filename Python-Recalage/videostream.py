from threading import Thread, Timer
import cv2 as cv
import pyk4a as k4a

class VideoStream:
    def __init__(self, videocapture):
        # initialize the file video stream
        self.stream = videocapture
        self.stream.set(cv.CAP_PROP_BUFFERSIZE, 2)
        # ESSENTIAL to maintain stream FPS for FLIR camera
        self.stream.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.grabbed, self.frame = self.stream.read()
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
                self.grabbed, self.frame = self.stream.read()
                cv.waitKey(1)
                # Ends thread if read() fails
                if not self.grabbed:
                    self.stopped = True                        
    
    def read(self):
        return self.frame

    def stop(self):
        # Stops thread
        self.stopped = True
        self.stream.release()


# Specific VideoStream class for Kinect that returns depth + rgb using pyK4a
class KinectVideoStream:
    def __init__(self):
            # Open Kinect with pyK4a for more options
        self.kinect = k4a.PyK4A(k4a.Config(
                    color_format=k4a.ImageFormat.COLOR_MJPG,
                    color_resolution=k4a.ColorResolution.RES_720P,
                    depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                    camera_fps=k4a.FPS.FPS_30,
                    synchronized_images_only=True,
                ))
        self.kinect.start()
        self.frame  = self.kinect.get_capture()
    
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
                    self.frame  = self.kinect.get_capture()
                except:
                # Ends thread if get_capture() fails
                    self.stopped = True                        
    
    def read(self):
        return self.frame

    def stop(self):
        # Stops thread
        self.stopped = True
        self.kinect.stop()

def openStream(targetHeights = [], targetCameras = []):
    # Function to open a set of streams for a set of targetCameras/targetheights
    # Directly opens specified camera indexes
    if len(targetCameras) != 0:
        streams = [None] * len(targetCameras)
        for (streamIndex, cameraIndex) in enumerate(targetCameras):
            cap = cv.VideoCapture(cameraIndex, cv.CAP_DSHOW)
            if cap.isOpened():
                print("Found", cameraIndex)
                streams[streamIndex] = VideoStream(cap)
        return streams

    # Searches for cameras using height
    elif len(targetHeights) != 0:
        cameraIndex = 0 
        # Array of None streams equal to number of cameras looked for
        streams = [None] * len(targetHeights)
        while True:
            # CAP_DSHOW is essential for FLIR framerate
            cap = cv.VideoCapture(cameraIndex, cv.CAP_DSHOW)

            # If we reach the end of available cameras
            if not cap.isOpened():
                break
            else: 
                cameraIndex += 1
                frameHeight = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
                try:
                    # Uses frame height to identify FLIR T1020 vs Kinect, maybe should be changed
                    pos = targetHeights.index(frameHeight)
                    print("Found", targetHeights[pos])
                    streams[pos] = VideoStream(cap)
                except:
                    # Otherwise, releases capture and moves on to next camera
                    cap.release()
        
        return streams

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

