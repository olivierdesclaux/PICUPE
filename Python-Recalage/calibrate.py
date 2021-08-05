import sys
import numpy as np
import cv2 as cv
import time
# Local modules
import videostream
from circledetector import CircleDetector
from calibrationhandler import Calibration
from convenience import stop

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
kCircleDetector = CircleDetector()
#fCircleDetector = CircleDetector(threshold = (30, 240))
fCircleDetector = CircleDetector(minArea = 15)

# Skips X seconds between each checkerboard to increase variability of calibration images
secondsToSkip = 2
lastCheckerboardTime = time.time()
# Maximum number of checkerboard images to obtain
initialCalibImages = 15
# Minimum number of checkerboard images to calculate matrices after removing outliers
minCalibImages = initialCalibImages - 5
# Maximum error to tolerate on each point, in pixels
maximumPointError = 3

# Initial message to user
print("Press q to exit program.")

# Loops through all cameras connected via USB
[flirStream, kinectStream] = videostream.openStream([480, 720]) # Only for webcam
#[flirStream, kinectStream] = videostream.openStream([768, 720])
            
if flirStream is None or kinectStream is None:
    stop([flirStream, kinectStream], "Could not open both cameras.")

# Main capture and calibrate loop
takeMoreImages = True
while takeMoreImages:
    ## This section finds grids in images and informs the user of the remaining grids to find
    # It loops until at least initialCalibImages grids have been found
    while len(objectPositions) < initialCalibImages:
        # Get next frame from both cameras
        if kinectStream.stopped or flirStream.stopped:
            stop([flirStream, kinectStream], "Camera disconnnected, not enough frames taken for calibration.")
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
            drawingFrame = kinectFrame if cameraType == 0 else flirFrame
            for pos in imagePositions:
                # Uses corners of each calibration grid as points for lines
                points = np.array([pos[0], pos[numberOfColumns - 1], pos[totalCircles - 1], pos[numberOfColumns*(numberOfRows - 1)]], np.int32) 
                cv.polylines(drawingFrame, [points], True, (20, 220, 90), 2)

        # Indicates how many images remain
        textColor = (80, 250, 55)
        cv.putText(kinectFrame, 'Remaining images: ' + str(initialCalibImages - len(objectPositions)), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)

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
            stop([flirStream, kinectStream], "User exited program.")

    ## This section calculates the calibration from the points collected from all grids
    # Calculates size of kinect and flir frames (using last frame)
    kSize = kinectGreyFrame.shape[::-1]
    fSize = flirGreyFrame.shape[::-1]

    # Creates two Calibration objects for handling calibration and error-checking
    kinectCalibration = Calibration("Kinect", objectPositions, imagePositionsBoth[0], kSize, minCalibImages)
    flirCalibration = Calibration("FLIR", objectPositions, imagePositionsBoth[1], fSize, minCalibImages)

    # Tries to calibrate Kinect first
    if kinectCalibration.calibrate(flirCalibration):
        print("Kinect calibration success")

        # Tries to calibrate FLIR next:
        if flirCalibration.calibrate(kinectCalibration):
            print("FLIR calibration succcess")
            # Displays error graphics for both calibration, for final manual inspection of results
            kinectCalibration.displayError()
            flirCalibration.displayError()
            # Prints calibration matrices to file
            kinectCalibration.outputToFile()
            flirCalibration.outputToFile()
            # Breaks out of image-taking loop
            takeMoreImages = False
            cv.destroyAllWindows()
        else:
            print("Unable to calibrate FLIR")
    else:
        print("Unable to calibrate Kinect")

print("Calibration successful")
