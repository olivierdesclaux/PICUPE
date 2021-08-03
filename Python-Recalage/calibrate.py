import sys
import numpy as np
import cv2 as cv
import time
import matplotlib.pyplot as plt
# Local modules
import videohelper
from blobdetector import CircleDetector
from calibrationhandler import Calibration
from npencoder import NumpyEncoder


# Convenience function for killing open objects
def stop(message):
    cv.destroyAllWindows()
    try:
        kinectStream.stop()
    except: 
        pass
    try:
        flirStream.stop()
    except:
        pass
    sys.exit(str(message))

## Initial global parameters for calibration
# Corner finder variables
numberOfRows = 12
numberOfColumns = 12
totalCircles = numberOfRows*numberOfColumns
checkerboardSize = (numberOfRows, numberOfColumns)

# Array of float32 points
# Last axis (z) is always 0
# Y axis (rows) increments 0, 1, 2, ... numberOfRows-1
# X axis (columns) increments 0, 1, 2, ... numberOfColumns-1
objectCorners = np.array([[x % numberOfColumns, np.floor(x / numberOfColumns), 0] for x in range(totalCircles)], np.float32)
# Arrays to store object points and image points from all the images.
objectPositions = [] #3d points in real world space
imagePositionsBoth = [[],[]] # 2d points in image plane, one for each camera
# Initialize blob detectors with circle parameters for kinect and FLIR
kCircleDetector = CircleDetector(threshold = (60, 240))
#fCircleDetector = CircleDetector(threshold = (30, 240))
fCircleDetector = CircleDetector(threshold = (50, 200), minArea = 15)

# Skips X seconds between each checkerboard to increase variability of calibration images
secondsToSkip = 2
lastCheckerboardTime = time.time()
# Maximum number of checkerboard images to obtain
maxCalibImages = 15
# Minimum number of checkerboard images to calculate matrices after removing outliers
minCalibImages = maxCalibImages - 5
# Maximum error to tolerate on each point, in pixels
maximumPointError = 4

# Initial message to user
print("Press q to exit program.")

# Loops through all cameras connected via USB
flir, kinect = None, None
cameraIndex = 0 # Should be 0, changed to 1 because on laptops 0 is always webcam
while True:
    # CAP_DSHOW is essential for FLIR framerate
    cap = cv.VideoCapture(cameraIndex, cv.CAP_DSHOW)
    # If we reach the end of available cameras
    if not cap.isOpened():
        break
    else:
        cameraIndex += 1
        frameHeight = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
        # Uses frame height to identify FLIR T1020 vs Kinect, maybe should be changed
        if frameHeight == 768:
            print("Found FLIR")
            flir = cap
            flirStream = videohelper.VideoStream(flir)
        elif frameHeight == 720:
            print("Found Kinect")
            kinect = cap
            kinectStream = videohelper.VideoStream(kinect)
        elif frameHeight == 480:
            print("Found TEMP")
            flir = cap
            flirStream = videohelper.VideoStream(flir)
        # Otherwise, releases capture and moves on to next camera
        else:
            cap.release()
            
if flir is None or kinect is None:
    stop("Could not open both cameras.")

# Main capture and calibrate loop
takeMoreImages = True
while takeMoreImages:
    ## This section finds grids in images and informs the user of the remaining grids to find
    # It loops until at least maxCalibImages grids have been found
    while len(objectPositions) < maxCalibImages:
        # Get next frame from both cameras
        if kinectStream.stopped or flirStream.stopped:
            stop("Camera disconnnected, not enough frames taken for calibration.")
        else:
            kinectFrame = kinectStream.read()
            flirFrame = flirStream.read()

        # Grey Kinect uses red component - blue component, flir uses a simple grayscale
        kinectGreyFrame = cv.subtract(kinectFrame[:,:,2], kinectFrame[:,:,0])
        #flirGreyFrame = cv.cvtColor(flirFrame, cv.COLOR_BGR2GRAY)
        flirGreyFrame = cv.subtract(flirFrame[:,:,2], flirFrame[:,:,0])

        # Waits secondsToSkip between checkerboards
        if (time.time() - lastCheckerboardTime > secondsToSkip):
            # Detects blobs and draws them on each frame, DISABLE TO IMPROVE FRAMERATE
            keypoints = kCircleDetector.get().detect(kinectGreyFrame)
            kinectFrame = cv.drawKeypoints(kinectFrame, keypoints, np.array([]), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            keypoints = fCircleDetector.get().detect(flirGreyFrame)
            flirFrame = cv.drawKeypoints(flirFrame, keypoints, np.array([]), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Tries to find chessboard corners
            kinectBoardFound, kImageCorners = cv.findCirclesGrid(kinectGreyFrame, checkerboardSize, flags=cv.CALIB_CB_SYMMETRIC_GRID+cv.CALIB_CB_CLUSTERING, blobDetector=kCircleDetector.get())
            flirBoardFound, fImageCorners = cv.findCirclesGrid(flirGreyFrame, checkerboardSize, flags=cv.CALIB_CB_SYMMETRIC_GRID+cv.CALIB_CB_CLUSTERING, blobDetector=fCircleDetector.get())

            if kinectBoardFound and flirBoardFound:
                # Add them to permanent lists
                objectPositions.append(objectCorners)
                imagePositionsBoth[0].append(kImageCorners)
                imagePositionsBoth[1].append(fImageCorners)

                # Resets time counter
                lastCheckerboardTime = time.time()

        # Draws previously used checkerboards using OpenCV lines, in each camera view
        for cameraType, imagePositions in enumerate(imagePositionsBoth):
            # Uses cameraType to deduce frame to draw to
            frame = kinectFrame if cameraType == 0 else flirFrame
            for pos in imagePositions:
                # Uses corners of each calibration grid as points for lines
                points = np.array([pos[0], pos[numberOfColumns - 1], pos[totalCircles - 1], pos[numberOfColumns*(numberOfRows - 1)]], np.int32) 
                cv.polylines(frame, [points], True, (20, 220, 90), 2)

        # Indicates how many images remain
        textColor = (80, 250, 55)
        cv.putText(kinectFrame, 'Remaining images: ' + str(maxCalibImages - len(objectPositions)), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)

        # Scales FLIR image to same height as Kinect for display side-by-side
        flirRatio =  flirFrame.shape[1] / flirFrame.shape[0]
        scalingFactor = 1.5 # Reduces overall window size to fit on monitor
        kHeight, kWidth  = kinectFrame.shape[0:2]
        fDim = (int(kHeight*flirRatio/scalingFactor), int(kHeight/scalingFactor))
        kDim = (int(kWidth/scalingFactor), int(kHeight/scalingFactor))
        flirFrame = cv.resize(flirFrame, fDim)
        kinectFrame = cv.resize(kinectFrame, kDim)
        # Side-by-side display, requires both frames have same height
        concatenatedFrames = cv.hconcat([kinectFrame, flirFrame])
        cv.imshow('Calibration', concatenatedFrames)

        # Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.")

    ## This section calculates the calibration from the points collected from all grids
    # Calculates size of kinect and flir frames (using last frame)
    kSize = kinectGreyFrame.shape[::-1]
    fSize = flirGreyFrame.shape[::-1]
    # Creates two Calibration objects for handling calibration and error-checking
    kinectCalibration = Calibration("Kinect", objectPositions, imagePositionsBoth[0], kSize, minCalibImages)
    flirCalibration = Calibration("FLIR", objectPositions, imagePositionsBoth[1], fSize, minCalibImages)

    # Tries to calibrate Kinect first
    if kinectCalibration.calibrate():
        print("Kinect calibration success")
        # Removes outlier images of the Kinect from the FLIR calibration as well
        flirCalibration.pop(kinectCalibration.poppedIndex)
        # Tries to calibrate FLIR next:
        if flirCalibration.calibrate():
            print("FLIR calbiration succcess")
            # Removes outlier images of the FLIR from the Kinect calibration
            # However, Kinect parameters are assumed to still be reasonable, so are not re-calibrated
            kinectCalibration.pop(flirCalibration.poppedIndex)
            # Displays error graphics for both calibration, for final manual inspection of results
            kinectCalibration.displayError()
            flirCalibration.displayError()
            # Breaks out of image-taking loop
            takeMoreImages = False

# END OF while takeMoreImages

# Termination criteria : 1st parameter is flag, 2nd is maximum iterations, 3rd is maximum epsilon
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 50, 0.0001)

# Stereo calibration in OpenCV, using keypoints identified previously
# This obtains the R and T matrices to transform between 2 camera positions, plus improved camera matrices and distortion coefficients
# Uses previous calibration parameters as a starting point (CALIB_USE_INSTRINSIC_GUESS)
stereoSuccess, kMatrix, kDist, fMatrix, fDist, R, T, E, F = cv.stereoCalibrate( 
    objectPositions, imagePositionsBoth[0], imagePositionsBoth[1], 
    kinectCalibration.cameraMatrix, kinectCalibration.distortion,
    flirCalibration.cameraMatrix, flirCalibration.distortion,
    kSize, flags=cv.CALIB_USE_INTRINSIC_GUESS, criteria=criteria)
if stereoSuccess:
    print("Stereo calibration successful")
    # Alpha can be varied from 0 to 1 to get no black pixels or all pixels in undistorted image
    # This obtains precise R and T matrices to transforms PIXELS between images
    R1, R2, P1, P2, Q = cv.stereoRectify(kMatrix, kDist, fMatrix, fDist, kSize, R, T, alpha=1)
    # Calculates maps for undistorting each image into a rectified state
    kMapx, kMapy = cv.initUndistortRectifyMap(kMatrix, kDist, R1, P1, kSize, cv.CV_32FC1) #try CV_32FC2
    fMapx, fMapy = cv.initUndistortRectifyMap(fMatrix, fDist, R1, P1, fSize, cv.CV_32FC1) #try CV_32FC2

# Displayed undistorted image for debugging, should be in another file!
while True:
    kinectFrame = kinectStream.read()
    flirFrame = flirStream.read()
    # Maps pixels from FLIR to Kinect image
    flirRemapped = cv.remap(flirFrame, fMapx, fMapy, cv.INTER_LINEAR)

    # Scales FLIR image to same height as Kinect for display side-by-side
    flirRatio =  flirRemapped.shape[1] / flirRemapped.shape[0]
    scalingFactor = 1.5 # Reduces overall window size to fit on monitor
    kHeight, kWidth  = kinectFrame.shape[0:2]
    fDim = (int(kHeight*flirRatio/scalingFactor), int(kHeight/scalingFactor))
    kDim = (int(kWidth/scalingFactor), int(kHeight/scalingFactor))
    flirRemapped = cv.resize(flirRemapped, fDim)
    kinectFrame = cv.resize(kinectFrame, kDim)
    # Side-by-side display, requires both frames have same height
    concatenatedFrames = cv.hconcat([kinectFrame, flirRemapped])
    cv.imshow('Undistorted', concatenatedFrames)

    # Waits for next frame or quits main loop if 'q' is pressed
    if cv.waitKey(1) == ord('q'):
        stop("User exited program.")
