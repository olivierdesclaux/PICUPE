import sys
from threading import Thread, Timer
import cv2 as cv


class VideoStream:
    def __init__(self, videocapture):
        # initialize the file video stream
        self.stream = videocapture
        self.stream.set(cv.CAP_PROP_BUFFERSIZE, 1)
        (self.grabbed, self.frame) = self.stream.read()
        # used to indicate if the thread should be stopped or not
        self.stopped = False
        # Threads used to read video and lock video
        Thread(target=self.update, args=(), daemon=True).start()

    def update(self):
        # keep looping infinitely
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            else:
                # Read next frame
                (self.grabbed, self.frame) = self.stream.read()
                # Ends Thread if read() fails
                if not self.grabbed:
                    self.stopped = True

    def read(self):
        # return last frame
        return self.frame

    def stop(self):
        # Stops thread
        self.stopped = True

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

