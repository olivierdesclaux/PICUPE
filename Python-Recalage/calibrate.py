import cv2 as cv
import argparse
from datetime import datetime
import os
# Local modules
from videostream import selectStreams
from circledetector import CircleDetector
from calibrationhandler import CalibrationHandler
from circlegridfinder import CircleGridFinder
from utils import stop

def main(cameraType, saveDirectory):
    """Calibrate.py obtains intrinsic camera parameters using a grid pattern 
    Parameters
    ----------
    cameraType : string
        First character indicates type of camera that is to be calibrated
    saveDirectory : string
        Directory in which to save the calibration matrices and error image
    """
    # Global calibration parameters
    # Maximum number of checkerboard images to obtain
    initialCalibImages = 30
    # Minimum number of checkerboard images to calculate matrices from,
    # after removing outliers
    minCalibImages = 27
    # Maximum error to tolerate on each point during calibration, in pixels
    # Beyond this, entire grid is rejected as outlier
    maximumPointError = 3
    # Initialize circle detector for finding circle shapes in images
    circleDetector = CircleDetector()
    # Open stream based on camera index or height, see videostream.py
    stream, type = selectStreams(cameraType[0:1])
    # Creates object that continually finds circle grids in images
    gridFinder = CircleGridFinder(cameraType, [stream], [type], 
        [circleDetector], initialCalibImages)

    # Initial message to user
    print("Press q to exit program.")
    print("Taking grid pictures...")

    # Main capture and calibrate loop
    # This section asks the user to find initialCalibImages grids
    takeMoreImages = True
    while takeMoreImages:
        gridFinder.start()
        # Repeats until enough images found in this stream
        while not gridFinder.finished:
            # Checks that the stream is still open
            if stream.stopped:
                stop("Camera disconnnected, not enough " + gridFinder.name 
                    + " frames taken for calibration.", [stream])
            else:
                # Reads a new frame to display to user as GUI
                frame = stream.read().copy()
                # Large performance impact, displays circles in real time
                gridFinder.drawCircles([frame]) 
                # Small performance impact, displays previously found grids
                gridFinder.drawOutlines([frame]) 
                # Indicates how many images remain to take in upper left
                cv.putText(frame, 'Remaining images: ' 
                    + str(initialCalibImages - gridFinder.len()), 
                    (10, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
                # Shows frame
                cv.imshow('Calibration', frame)
                # Waits 1 ms for user input, allowing user to 
                # quit main loop if 'q' is pressed
                if cv.waitKey(1) == ord('q'):
                    stop("User exited program.", [stream])
        # All grids have now been found, proceeding to calibration
        # Gets frame size for this stream
        frameSize = frame[:, :, 0].shape[::-1]
        # Creates CalibrationHandler object to handle cv.calibrateCamera() 
        # and error-checking
        calibration = CalibrationHandler(gridFinder.objectPositions,
            gridFinder.allImagePositions[0], frameSize, 
            minCalibImages, maximumPointError)
        # If the calibration succeeds
        if calibration.calibrate():
            # Display matplotlib graphics of errors and saves to .png
            calibration.displayError(saveDirectory)
            # Tries to save matrices to directory, can fail if 
            # wrong directory/file in use
            try:
                calibration.writeToFile(saveDirectory)
            except:
                stop("Failed to write matrices to file.", [stream])
            takeMoreImages = False
        else:
        # Otherwise, return to grid-finding loop
            print(gridFinder.name + " calibration failed, needs more images.")

    stop("Calibration successful.", [stream])

# Standard argument parser implementation for command-line use
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Selection of camera type with -c argument. 
    # K, F or W are accepted (see utils, selectStreams)
    parser.add_argument('-c', type=str, dest='cameraType', 
        help='Kinect or FLIR', required=True)
    args = parser.parse_args().__dict__
    cameraType = args["cameraType"]
    #Initialise directory with Date and Time in name
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H-%M")
    saveDirectory = os.path.join("Results", cameraType + "_" + dt_string)
    # Starts true program
    main(cameraType, saveDirectory)
