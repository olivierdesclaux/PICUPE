from threading import Thread, Timer
from enum import Enum
import cv2 as cv
import pyk4a as k4a

class VideoStream:
    """Returns frames from an OpenCV VideoCapture

    Opens a new thread with _update as target to maximise frames taken 
    per second and reduce lag on main thread.
    Also performs special operations on _capture to ensure proper codec
    for FLIR camera.

    Attributes
    ----------
    _capture : OpenCV VideoCapture
        OpenCV object for opening a camera
        Provides continuous access to camera and new images
    frame : OpenCV image
        Latest frame taken from camera
    stopped : bool
        Indicator of frame-capturing thread status

    Parameters
    ----------
    _capture : see attributes above.
    flag : string
        Used to signal "Webcam" status
        Only used for webcam testing as resolution was too low

    Methods
    -------
    read()
        Returns latest frame taken in thread
    stop()
        Stops frame-capturing thread and releases _capture
    """
    def __init__(self, videocapture, flag=""):
        # initialize the file video stream
        self._capture = videocapture
        self._capture.set(cv.CAP_PROP_BUFFERSIZE, 2)
        # TEMP : for webcam only, set dimensions to maximise resolution
        if "W" in flag:
            self._capture.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
            self._capture.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        # Forces a different codec for capture
        # ESSENTIAL to maintain stream FPS for FLIR camera
        self._capture.set(
            cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        _, self.frame = self._capture.read()

        # Used to indicate if the thread should be stopped or not
        self.stopped = False
        # Thread used to constantly grab new frames
        Thread(target=self._update, args=(), daemon=True).start()

    def _update(self):
        """Fetches next frame from capture object

        Returns
        -------
        None.
        """
        while True:
            # If the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            else:
                # Grab next frame
                grabbed, self.frame = self._capture.read()
                # Ends thread if read() fails
                if not grabbed:
                    self.stopped = True                        
    
    def read(self):
        """Returns latest frame taken in thread
        """
        return self.frame

    def stop(self):
        """Stops frame-capturing thread and closes camera
        """
        # Stops thread
        self.stopped = True
        # Closes video capture
        self._capture.release()


# Specific VideoStream class for Kinect that returns depth + rgb using pyK4a
class KinectVideoStream:
    """Returns frames from a Kinect camera using pyk4a

    Provides a wrapper for Kinect capture, including extracting depth frame.
    Also open a new thread with _update as target to separate frame-taking
    from main thread. However, is not much faster because pyk4a freezes 
    process while waiting for the next frame.

    Attributes
    ----------
    _kinect : Pyk4a object
        Kinect handling object, with specific video settings
    color : OpenCV image
        Latest color frame taken from Kinect
    depth : OpenCV image
        Latest depth frame taken from Kinect
    stopped : bool
        Indicator of frame-capturing thread status

    Parameters
    ----------
    See attributes above.

    Methods
    -------
    read()
        Returns latest color frame taken in thread
    readDepth()
        Returns latest depth frame taken in thread
    stop()
        Stops frame-capturing thread and releases capture
    """
    def __init__(self):
        # Open Kinect with pyK4a for more options and depth mode
        self._kinect = k4a.PyK4A(k4a.Config(
                    color_resolution=k4a.ColorResolution.RES_720P,
                    depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                    camera_fps=k4a.FPS.FPS_30,
                    synchronized_images_only=True,
                ))
        # Starts kinect and obtains first frame to fill attributes
        self._kinect.start()
        frame = self._kinect.get_capture()
        # Gets only BGR components of color
        self.color = frame.color[:, :, :3]
        # Passes depth through conversion function 
        self.depth = self._normalizeDepth(frame.depth)
        # Used to indicate if the thread should be stopped or not
        self.stopped = False
        # Thread used to constantly grab new frames
        Thread(target=self._update, args=(), daemon=True).start()

    def _update(self):
        """Fetches next color and depth frames from kinect

        Returns
        -------
        None.
        """
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            else:
                # Grab next frame
                try:
                    frame  = self._kinect.get_capture()
                    self.color = frame.color[:, :, :3]
                    self.depth = self._normalizeDepth(frame.depth)
                except:
                # Ends thread if get_capture() fails
                    self.stopped = True

    def read(self):
        """Returns latest color frame taken in thread
        """
        return self.color

    def readDepth(self):
        """Returns latest depth frame taken in thread
        """
        return self.depth

    def stop(self):
        """Stops frame-capturing thread and closes camera
        """
        # Stops thread
        self.stopped = True
        # Closes Kinect camera
        self._kinect.stop()
    
    def _normalizeDepth(self, frame, maxValue=4000):
        """Clips and normalizes depth frame for OpenCV display

        Standard depth frame from Kinect has some very high values
        This makes "normal" distances invisible (not enough contrast)

        Parameters
        ----------
        frame : OpenCV monochrome frame
            Single-channel frame to clip and normalize
        maxValue : int, default=4000
            Maximum value to clip frame at

        Returns
        -------
        normalizedFrame : OpenCV monochrome frame
            Frame after adjustments
        """
        # Clip numpy array with maxvalue for better contrast
        clipFrame = frame.clip(0, maxValue)
        # Convert 8U values to standard grayscale 
        # and normalize values to maximise contrast
        return cv.normalize(
            clipFrame, None, 0, 255, cv.NORM_MINMAX, dtype=cv.CV_8U)
    
class StreamType(Enum):
    """Possible types of camera streams
    """
    rgb = 0 # Color
    ir = 1 # Infrared
    depth = 2

def openStream(targetCamera = None, targetHeight = None, flag = "", 
               openedStreams = []):
    """ Creates OpenCV capture from target and checks it is open
    
    Selection of cameras can be done two ways: 
        Directly, using the camera's index
        Indirectly, using the camera frame's height
    Indirect method useful when unsure which index desired camera is on

    Parameters
    ----------
    targetCamera : int
        Index of camera to open directly
    targetHeight : int
        Height, in pixels, of frame of camera to open
    flag : string
        Temporary indicator flag for opening Webcam stream
    openedStreams : list of ints
        Indexes of previously opened cameras
        Prevents double-opening, which can cause crashes
    
    Returns
    -------
    None.
    """
    # Directly opens specified camera indexes
    if targetCamera in openedStreams:
        raise Exception("Camera already opened.")
    elif targetCamera is not None:
        # CAP_DSHOW is essential for FLIR framerate
        cap = cv.VideoCapture(targetCamera, cv.CAP_DSHOW)
        if cap.isOpened():
            print("Found camera number", targetCamera)
            return VideoStream(cap, flag), targetCamera
        else:
            raise Exception(
                "Unable to open camera number " + str(targetCamera))

    # Searches for cameras using height
    elif targetHeight is not None:
        # Starts at first camera
        cameraIndex = 0
        while True:
            while cameraIndex in openedStreams:
                cameraIndex += 1
            # CAP_DSHOW is essential for FLIR framerate
            cap = cv.VideoCapture(cameraIndex, cv.CAP_DSHOW)

            if cap.isOpened():
                frameHeight = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
                if frameHeight == targetHeight:
                    print("Found camera with height", targetHeight, "px")
                    return VideoStream(cap, flag), cameraIndex
                else:
                    # Otherwise, releases capture and moves on to next camera
                    cap.release()
                    cameraIndex += 1
            else: 
            # If we reach the end of available cameras
                raise Exception(
                    "Unable to open camera with height " 
                    + str(targetHeight) + " px")
    # If no camera parameter specified
    else:
        raise Exception("No camera specified.")

def selectStreams(streamLetters, useKVS=False):
    """ Opens several streams based on stream identifier

    Parameters
    ----------
    streamLetters : string of {K, F, W}
        Identifiers of cameras to open
        Each letter is one camera
    useKVS : bool
        Asks to use KinectVideoStream for Kinect instead of VideoStream
    
    Returns
    -------
    streams : list of VideoStream/KinectVideoStream
        Opened stream-handling objects
    types : list of StreamTypes
        Type of cameras opened
    """
    streams = []
    # Keeps track of opened streams to prevent double opening
    streamIndices = []
    types = []

    # We can identify on which port the camera is located by looking at 
    # the height of the image in different ports.
    # The FLIR in IR has a height of 768 pixels 
    # Kinect in RGB has a height of 720 pixels
    for letter in streamLetters:
        if useKVS and letter == "K":
            stream = KinectVideoStream()
            types.append(StreamType.rgb)     
        elif letter == "K":
            stream, streamIndex = openStream(
                targetHeight = 720, openedStreams=streamIndices)
            types.append(StreamType.rgb)
        elif letter == "F":
            stream, streamIndex = openStream(
                targetHeight = 768, openedStreams=streamIndices)
            types.append(StreamType.ir)
        elif letter == "W":
            stream, streamIndex = openStream(
                targetCamera = 0, flag = "W", openedStreams=streamIndices)
            types.append(StreamType.rgb)
        else:
            # If letter does not match any of the options
            for stream in streams:
                stream.stop()
            raise LookupError("Invalid camera types selected.")
            
        streams.append(stream)
        streamIndices.append(streamIndex)

    return streams, types

class FPS:
    """Uses frame counts to generate an FPS number every second

    Must increment self.frames to generate FPS
    Access to FPS is provided by self.fps

    Attributes
    ----------
    frames : int
        Frame counter, should be incremented externally
        Reset every time fps is calculated
    FPS : int
        Frozen count of frames in previous second
        Should only be read externally
    _timer : Timer
        Triggers FPS update thread 1 second from now
    _isRunning : bool
        Prevents FPS from being started twice

    Parameters
    ----------
    None.

    Methods
    -------
    start()
        Enables FPS calculations via a repeating Timer
    stop()
        Disables FPS calculations
    """
    def __init__(self):
        self.frames = 0
        self.fps = 0
        self._timer = None
        self._isRunning = False
        self.start()

    def _takeFPS(self):
        """Puts a snapshot of frames in fps and relaunches _timer

        Returns
        -------
        None.
        """
        # Resets timer for next snapshot of frames
        self._isRunning = False
        self.start()
        # Stores frames count in fps and resets frames count
        self.fps = self.frames
        self.frames = 0

    def start(self):
        """Enables FPS calculations via a repeating Timer

        Returns
        -------
        None.
        """
        # Check to prevent launching twice
        if not self._isRunning:
            # Launches Timer to take snapshot in 1 second
            self._timer = Timer(1, self._takeFPS)
            self._timer.start()
            # Prevents launching twice
            self._isRunning = True

    def stop(self):
        """Disables FPS calculations

        Returns
        -------
        None.
        """
        # Stop timer to stop next FPS count
        self._timer.cancel()
        self._isRunning = False

