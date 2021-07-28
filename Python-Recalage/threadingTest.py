import sys
from threading import Thread
import numpy as np
import cv2 as cv

class VideoStream:
    def __init__(self, cameraIndex):
        # initialize the file video stream
        self.stream = cv.VideoCapture(cameraIndex)
        self.stream.set(cv.CAP_PROP_BUFFERSIZE, 200)
        (self.grabbed, self.frame) = self.stream.read()
        # used to indicate if the thread should be stopped or not
        self.stopped = False

    def start(self):
        # start a thread to read frames from the file video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def read(self):
        # return next frame in the queue
        return self.frame

    def stop(self):
        # Stops thread
        self.stopped = True

flirStream = VideoStream(1).start()
frameCounter = 0

while True:
    if not flirStream.stopped:
        flirCapture = flirStream.read()
        cv.putText(flirCapture, str(frameCounter), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (120, 190, 50), 2)
        cv.imshow('frame', flirCapture)
        print(frameCounter)
        frameCounter += 1
        ## Waits 25 ms for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(20) == ord('q'):
            flirStream.stop()
            cv.destroyAllWindows()
            sys.exit("User exited program.")
    else:
        cv.destroyAllWindows()
        sys.exit("Camera disconnected")

