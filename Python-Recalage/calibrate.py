from matplotlib.pyplot import grid
from circlegridfinder import CircleGridFinder
import numpy as np
import cv2 as cv
import time
# Local modules
import videostream
from circledetector import CircleDetector
from calibrationhandler import Calibration
from convenience import stop, scaleForHconcat

# Initialize blob detectors with circle parameters for kinect and FLIR
kCircleDetector = CircleDetector()
#fCircleDetector = CircleDetector(threshold = (30, 221, 20))
fCircleDetector = CircleDetector(minArea = 12)

## Global calibration parameters
# Maximum number of checkerboard images to obtain
initialCalibImages = 30
# Minimum number of checkerboard images to calculate matrices after removing outliers
minCalibImages = initialCalibImages - 10
# Maximum error to tolerate on each point, in pixels
maximumPointError = 5

# Initial message to user
print("Press q to exit program.")

# Loops through all cameras connected via USB
streams = [kinectStream, flirStream] = videostream.openStream(targetCameras=[1, 0]) # Only for webcam
#streams = [kinectStream, flirStream] = videostream.openStream(targetHeights = [720, 768]) # FLIR

flirStream.stream.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
flirStream.stream.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

if flirStream is None or kinectStream is None:
    stop("Could not open both cameras.", streams)

# Creates objects to continually find circle grids in images
try:
    gridFinders = [kGridFinder, fGridFinder] = [CircleGridFinder([kinectStream], ["rgb"], [kCircleDetector], initialCalibImages), 
                                                CircleGridFinder([flirStream], ["rgb"], [fCircleDetector], initialCalibImages)]
except:
    stop("Could not create both grid finders", streams)

# Main capture and calibrate loop
takeMoreImages = True # Used to retake images if too many are removed during calibration
while takeMoreImages:
    ## This section asks the user to find initialCalibImages grids in each camera
    frameSizes = []
    for (stream, gridFinder) in zip(streams, gridFinders):
        gridFinder.start()
        # Repeats until enough images found in this stream
        while gridFinder.running:
            if stream.stopped:
                stop("Camera disconnnected, not enough frames taken for calibration.", streams)
            else:
                # Reads a new frame to display to user as GUI
                frame = stream.read()
                gridFinder.drawCircles([frame]) # Large performance impact, displays circles detected in real time
                gridFinder.drawOutlines([frame]) # Small performance impact, displays previously found grids as green lines
                # Indicates how many images remain in upper left
                textColor = (80, 250, 55)
                cv.putText(frame, 'Remaining images: ' + str(initialCalibImages - gridFinder.len()), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)
                # Shows frame 
                cv.imshow('Calibration', frame)
                # Waits 1 ms for user input, allowing user to quit main loop if 'q' is pressed
                if cv.waitKey(1) == ord('q'):
                    stop("User exited program.", streams)
        # Gets frame size for each stream, used for calibration later
        frameSizes.append(frame[:,:,0].shape[::-1])

    # Creates two Calibration objects for handling calibration and error-checking
    kinectCalibration = Calibration("Kinect", kGridFinder.objectPositions, kGridFinder.allImagePositions[0], frameSizes[0], minCalibImages, maximumPointError)
    flirCalibration = Calibration("FLIR", fGridFinder.objectPositions, fGridFinder.allImagePositions[0], frameSizes[1], minCalibImages, maximumPointError)

    # Tries to calibrate Kinect first
    if kinectCalibration.calibrate():
        print("Kinect calibration success")

        # Tries to calibrate FLIR next:
        if flirCalibration.calibrate():
            print("FLIR calibration succcess")
            # Displays error graphics for both calibration, for final manual inspection of results
            kinectCalibration.displayError()
            flirCalibration.displayError()

            # Prints calibration matrices to file
            kinectCalibration.writeToFile()
            flirCalibration.writeToFile()

            # Breaks out of image-taking loop
            takeMoreImages = False
            cv.destroyAllWindows()
        else:
            print("Unable to calibrate FLIR, take more images")
    else:
        print("Unable to calibrate Kinect, take more images")

print("Calibration successful")
