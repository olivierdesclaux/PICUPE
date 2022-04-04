import cv2.cv2 as cv2
import argparse
from datetime import datetime
import os
import sys
# Local modules
sys.path.append("../../")
from old.videostream import selectStreams2
from Recalage.circledetector import CircleDetector
from Recalage.calibrationhandler import CalibrationHandler
from old.circlegridfinder import CircleGridFinder
from Recalage.cameraUtils import stop


def calibrateCamera(cameraType, saveDirectory, flags, initialGuess, initialNumGrids=40, minNumGrids=35,
                    maxPointError=1.0, rows=9, cols=15):
    """Calibrate.py obtains intrinsic camera parameters using a grid pattern 

    Script asks user to present an asymmetric circle grid to the camera 
    in initialCalibImage positions. Grid should be tilted in every capture,
    not parallel to camera plane. Previously identified grids are shown 
    on screen in green. Grids should cover the entire camera FoV. 

    Parameters
    ----------
    cameraType : {'K', 'F', 'W'}
        Indicates type of camera that is to be calibrated
        K = Kinect, F = Flir, W = Webcam
    saveDirectory : string
        Directory in which to save the calibration matrices and error image
    initialNumGrids : int
        Number of grid images to obtain  before attempting calibration
    minNumGrids : int
        Minimum number of grid images in an accepted calibration
        Must be < initialNumGrids or will never complete
    maxPointError : float/int
        Maximum error to tolerate on each point during calibration, in pixels
        Beyond this, entire grid is rejected as outlier
    rows, cols : int, int
        Dimensions of circle grid used to calibrate

    Returns
    -------
    None.
    """
    # Initialize circle detector for finding circle shapes in images
    circleDetector = CircleDetector()
    # Open stream based on cameraType
    stream = None
    streamType = None
    try:
        # [stream], [streamType] = selectStreams(cameraType)
        [stream], [streamType] = selectStreams2(cameraType)
    # Invalid camera types
    except LookupError as err:
        stop(str(err), stream)
    # Creates object that continually finds circle grids in images
    gridFinder = CircleGridFinder([stream], [streamType], [circleDetector], initialNumGrids, numRows=rows, numCols=cols)

    # Initial message to user
    print("Press q to exit program.")
    print("Taking grid pictures...")

    # Main capture and calibrate loop
    # This section asks the user to find initialNumGrids grids
    while True:
        gridFinder.start()
        # Repeats until enough images found in this stream
        # This loop displays the GUI to the users
        while not gridFinder.finished:
            # Checks that the stream is still open
            if stream.stopped:
                stop("Camera disconnected, not enough frames taken for calibration.", [stream])
            else:
                # Reads a new frame to display to user as GUI
                frame = stream.read().copy()
                # Large performance impact, displays circles in real time
                gridFinder.drawCircles([frame])
                # Small performance impact, displays previously found grids
                gridFinder.drawOutlines([frame])
                # Indicates how many images remain to take in upper left
                cv2.putText(frame, 'Remaining images: ' + str(initialNumGrids - gridFinder.len()), (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
                # Shows frame
                cv2.imshow('Camera Calibration', frame)
                # Waits 1 ms for user input, allowing user to 
                # quit main loop if 'q' is pressed
                if cv2.waitKey(1) == ord('q'):
                    stop("User exited program.", [stream])
        # All grids have now been found, proceeding to calibration
        # Gets frame size for this stream
        frameSize = frame[:, :, 0].shape[::-1]
        # Creates CalibrationHandler object to handle cv2.calibrateCamera()
        # and error-checking
        calibration = CalibrationHandler(gridFinder.objectPositions,
                                         gridFinder.allImagePositions[0], frameSize,
                                         minNumGrids, maxPointError, flags, initialGuess)

        # If the calibration succeeds
        if calibration.calibrate():
            # Tries to save matrices to directory, can fail if
            # wrong directory/file in use
            try:
                calibration.writeToFile(saveDirectory, cameraType + "Calib.json")
            except:
                stop("Failed to write matrices to file.", [stream])
            # Break out of grid-finding loop
            # Display matplotlib graphics of errors and saves to .png
            calibration.displayError(saveDirectory)
            break
        else:
            # Otherwise, return to grid-finding loop
            print("Calibration failed, needs more images.")
    print("Calibration successful.")
    stream.stop()

    return os.path.abspath(os.path.join(saveDirectory, cameraType + "Calib.json"))

# Standard argument parser implementation for command-line use
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Selection of camera type with -c argument. 
    # K, F or W are accepted (see utils, selectStreams)
    parser.add_argument('-c', type=str, dest='cameraType', help='Kinect or FLIR', required=True)
    args = parser.parse_args().__dict__
    cameraType = args["cameraType"]
    # Initialise directory with Date and Time in name
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H-%M")
    saveDirectory = os.path.join("../../sandbox/results", cameraType + "_" + dt_string)
    # Starts true program
    # flirCalibFlags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K3
    initialGuess = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\CalibKinectFactory5Dist.json"
    flirCalibFlags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_K3
    calibrateCamera(cameraType, saveDirectory, flirCalibFlags, initialGuess)
