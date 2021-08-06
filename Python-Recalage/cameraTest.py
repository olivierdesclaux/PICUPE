import sys

from matplotlib.pyplot import grid
import cv2 as cv
import numpy as np
import videostream
from circledetector import CircleDetector
from circlegridfinder import CircleGridFinder

camera = cv.VideoCapture(1, cv.CAP_DSHOW)
stream = videostream.VideoStream(camera)
fps = videostream.FPS()
circleDetector = CircleDetector(minArea=10)
gridFinder = CircleGridFinder([stream], ["rgb"], [circleDetector], 15)

while True:
    if not stream.stopped:
        frame = stream.read()
        cv.putText(frame, str(fps.fps), (20, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (20, 20, 250), 2)

        cv.imshow('frame', frame)
        
        fps.frames += 1

        ## Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stream.stop()
            cv.destroyAllWindows()
            fps.stop()
            sys.exit("User exited program.")
    else:
        fps.stop()
        cv.destroyAllWindows()
        sys.exit("Camera disconnected")

