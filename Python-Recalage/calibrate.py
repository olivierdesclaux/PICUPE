from circlegridfinder import CircleGridFinder
import numpy as np
import cv2 as cv
# Local modules
from videostream import openStream, StreamType
from circledetector import CircleDetector
from calibrationhandler import CalibrationHandler
from utils import stop

# Initialize blob detectors with circle parameters for kinect and FLIR
kCircleDetector = CircleDetector()
wCircleDetector = CircleDetector()
fCircleDetector = CircleDetector()#threshold = (30, 221, 20))

## Global calibration parameters
# Maximum number of checkerboard images to obtain
initialCalibImages = 15
# Minimum number of checkerboard images to calculate matrices after removing outliers
minCalibImages = 10
# Maximum error to tolerate on each point during calibration, in pixels
# Beyond this, grid is rejected as outlier
maximumPointError = 9

# Initial message to user
print("Press q to exit program.")

# Opens streams chosen either directly by targetCameras or indirectly with their heights
#streams = openStream(targetCameras=[1, 0], flags = ["","webcam"]) # Kinect, Webcam
streams = openStream(targetHeights = [720])#, 768]) # Kinect, FLIR

# Creates objects to continually find circle grids in images
try:
    gridFinders = [CircleGridFinder("Kinect", [streams[0]], [StreamType.rgb], [kCircleDetector], initialCalibImages)]
                    #CircleGridFinder("FLIR", [streams[0]], [StreamType.ir], [fCircleDetector], initialCalibImages)]
except:
    stop("Could not create both grid finders", streams)

# Main capture and calibrate loop
for (stream, gridFinder) in zip(streams, gridFinders):
    takeMoreImages = True # Used to retake images if too many are removed during calibration
    print("Calibrating " + gridFinder.name)
    # This section asks the user to find initialCalibImages grids in each camera
    while takeMoreImages:
        gridFinder.start()
        # Repeats until enough images found in this stream
        while not gridFinder.finished:
            if stream.stopped:
                stop("Camera disconnnected, not enough " + gridFinder.name + " frames taken for calibration.", streams)
            else:
                # Reads a new frame to display to user as GUI
                frame = stream.read().copy()
                gridFinder.drawCircles([frame]) # Large performance impact, displays circles detected in real time
                gridFinder.drawOutlines([frame]) # Small performance impact, displays previously found grids as green lines
                # Indicates how many images remain to take in upper left
                textColor = (5, 90, 240)
                cv.putText(frame, 'Remaining images: ' + str(initialCalibImages - gridFinder.len()), (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)
                # Shows frame 
                cv.imshow('Calibration', frame)
                # Waits 1 ms for user input, allowing user to quit main loop if 'q' is pressed
                if cv.waitKey(1) == ord('q'):
                    stop("User exited program.", streams)
        # All grids have now been found, proceeding to calibration
        # Gets frame size for this stream
        frameSize = frame[:,:,0].shape[::-1]
        # Creates Calibration object to handle cv.calibrateCamera() and error-checking
        calibration = CalibrationHandler(gridFinder.name, gridFinder.objectPositions, gridFinder.allImagePositions[0], frameSize, minCalibImages, maximumPointError)
        if calibration.calibrate():
        # If the calibration succeeds, display the errors, print to a file and stop taking images from this stream
            print(gridFinder.name + " calibration successful.")
            calibration.displayError()
            calibration.writeToFile()
            takeMoreImages = False
        else:
        # Otherwise, return to grid-finding loop
            print(gridFinder.name + " calibration failed, needs more images.")

stop("Calibration successful", streams)
