import sys
import numpy as np
import cv2 as cv
import time
import json
import matplotlib.pyplot as plt
# Local imports
import videohelper
from blobdetector import CircleDetector

## Encoder for Numpy arrays to JSON
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

# Convenience function for killing open objects
def stop(message):
    cv.destroyAllWindows()
    try:
        fps.stop()
    except:
        pass
    try:
        kinectStream.stop()
    except: 
        pass
    try:
        flirStream.stop()
    except:
        pass
    sys.exit(str(message))

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
kObjectPositions = [] # 3d point in real world space
kImagePositions = [] # 2d points in image plane.
fObjectPositions = [] # 3d point in real world space, flir
fImagePositions = [] # 2d points in image plane, flir
# Initialize blob detectors with circle parameters for kinect and FLIR
kCircleDetector = CircleDetector(60, 240)
fCircleDetector = CircleDetector(30, 240)
# Skips X seconds between each checkerboard to increase variability of calibration images
secondsToSkip = 2
lastCheckerboardTime = time.time()
# Maximum number of checkerboard images to obtain
maximumCalibrationImages = 30
# Minimum number of checkerboard images to calculate matrices after removing outliers
minimumCalibrationImages = maximumCalibrationImages - 5
# Maximum error to tolerate on each point, in pixels
maximumPointError = 10
# Maximum distance within volume of calibration, in meters
maximumDistance = 5

# Initial message to user
print("Press q to exit program.")

# Loops through all cameras connected via USB
flir, kinect = None, None
cameraIndex = 1 # Should be 0, changed to 1 because on laptops 0 is always webcam
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
        # Otherwise, releases capture and moves on to next camera
        else:
            cap.release()
            

if flir is None or kinect is None:
    stop("Could not open both cameras.")

# Creates FPS counter for debugging code speed issues
fps = videohelper.FPS()

# Main capture and calibrate loop
calibrationCompleted = False
while not calibrationCompleted:
    # This section finds grids in images and informs the user of the remaining grids to find
    while len(kImagePositions) < maximumCalibrationImages:
        # Get next frame from both cameras
        if kinectStream.stopped or flirStream.stopped:
            stop("Camera disconnnected, not enough frames taken for calibration.")
        else:
            kinectFrame = kinectStream.read()
            flirFrame = flirStream.read()

        # Grey Kinect uses red component - blue component, flir uses a simple grayscale
        kinectGreyFrame = cv.subtract(kinectFrame[:,:,2], kinectFrame[:,:,0])
        flirGreyFrame = cv.cvtColor(flirFrame, cv.COLOR_BGR2GRAY)

        # Waits secondsToSkip between checkerboards
        if (time.time() - lastCheckerboardTime > secondsToSkip):
            # Detects blobs and draws them on each frame, DISABLE TO IMPROVE FRAMERATE
            keypoints = kCircleDetector.get().detect(kinectGreyFrame)
            kinectFrame = cv.drawKeypoints(kinectFrame, keypoints, np.array([]), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            keypoints = kCircleDetector.get().detect(flirGreyFrame)
            flirFrame = cv.drawKeypoints(flirFrame, keypoints, np.array([]), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

            # Tries to find chessboard corners
            kinectBoardFound, kImageCorners = cv.findCirclesGrid(kinectGreyFrame, checkerboardSize, flags=cv.CALIB_CB_SYMMETRIC_GRID+cv.CALIB_CB_CLUSTERING, blobDetector=kCircleDetector.get())
            flirBoardFound, fImageCorners = cv.findCirclesGrid(flirGreyFrame, checkerboardSize, flags=cv.CALIB_CB_SYMMETRIC_GRID+cv.CALIB_CB_CLUSTERING, blobDetector=kCircleDetector.get())

            if kinectBoardFound and flirBoardFound:
                # Add them to permanent lists
                kObjectPositions.append(objectCorners)
                kImagePositions.append(kImageCorners)
                fObjectPositions.append(objectCorners)
                fImagePositions.append(fImageCorners)

                # Resets time counter
                lastCheckerboardTime = time.time()

        # Draws previously used checkerboards
        poylgonColor = (20, 220, 90)
        for index, pos in enumerate(kImagePositions):
            points = np.array([pos[0], pos[numberOfColumns - 1], pos[totalCircles - 1], pos[numberOfColumns*(numberOfRows - 1)]], np.int32) 
            cv.polylines(kinectFrame, [points], True, poylgonColor, 2)
        for index, pos in enumerate(fImagePositions):
            points = np.array([pos[0], pos[numberOfColumns - 1], pos[totalCircles - 1], pos[numberOfColumns*(numberOfRows - 1)]], np.int32) 
            cv.polylines(flirFrame, [points], True, poylgonColor, 2)

        # Indicates how many images remain
        textColor = (80, 250, 55)
        cv.putText(kinectFrame, 'Remaining images: ' + str(maximumCalibrationImages - len(kImagePositions)), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)
        cv.putText(flirFrame, 'FPS: ' + str(fps.fps), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)


        fDim = (int(flirFrame.shape[1]/1.5), int(kinectFrame.shape[0]/1.5))
        kDim = (int(kinectFrame.shape[1]/1.5), int(kinectFrame.shape[0]/1.5))
        flirFrame = cv.resize(flirFrame, fDim)
        kinectFrame = cv.resize(kinectFrame, kDim)
        concatenatedFrames = cv.hconcat([kinectFrame, flirFrame])
        cv.imshow('Calibration', concatenatedFrames)
        fps.frames += 1

        ## Waits for next frame or quits main loop if 'q' is pressed
        if cv.waitKey(1) == ord('q'):
            stop("User exited program.")

    # This section calculates the calibration from the points collected from all grids
    calibrationCalculated = False
    while not calibrationCalculated:
        # Calculates calibration matrices from positions
        if len(imagePositions) > minimumCalibrationImages:
            retroprojectionError, cameraMatrix, distortion, rotation, translation = cv.calibrateCamera(objectPositions, imagePositions, grayFrame.shape[::-1], None, None, flags=cv.CALIB_RATIONAL_MODEL+cv.CALIB_ZERO_TANGENT_DIST)
            calibrationCalculated = True
        else:
            # Returns to image taking
            break

        # Stores different kinds of errors
        x, y, xError, yError, absError = [], [], [], [], []

        # Iterates through each image to calculate x and y errors of all corners
        for index, imagePositionOld in enumerate(imagePositions):
            # Reprojects objects to positions in image
            imagePositionNew, _ = cv.projectPoints(objectPositions[index], rotation[index], translation[index], cameraMatrix, distortion)
            # Compares pure x and y errors by subtracting two lists
            xError.append(imagePositionNew[:,:,0].ravel() - imagePositionOld[:,:,0].ravel())
            yError.append(imagePositionNew[:,:,1].ravel() - imagePositionOld[:,:,1].ravel())
            # Iterates through each point to calculate more complicated absError (cannot be done directly through list)
            for (oldPoint, newPoint) in zip(imagePositionOld, imagePositionNew):
                x.append(oldPoint[0, 0])
                y.append(oldPoint[0, 1])
                # Calculates error using L2 norm (distances squared)
                error = cv.norm(oldPoint[0], newPoint[0], cv.NORM_L2)
                if error > maximumPointError:
                    # If error is too large, eliminates the image in which the error is found
                    imagePositions.pop(index)
                    objectPositions.pop(index)
                    # Asks to perform calibration calculation again
                    calibrationCalculated = False
                    print("Image removed.")
                    break
                else:
                    absError.append(error)

        # If all errors have been checked and no outliers found, calibration is complete
        # Otherwise, it will return to the grid-finding loop to replace outlier grids
        calibrationCompleted = calibrationCalculated

# Outputs calibration matrices to file
with open('calibrationFile2.json', 'w') as file:
    # Assigns labels to values to make JSON readable
    dumpDictionary = {'Format' : 'OpenCV', 'Model' : 'Rational','CameraMatrix' : cameraMatrix, 'DistortionCoefficients' : distortion[0:7]}
    # Uses NumpyEncoder to convert numpy values to regular arrays for json.dump
    json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)

# Command line error prints
print("Retroprojection error :", retroprojectionError)
print("X error absolute mean, variance:", np.mean(np.abs(xError)), ',', np.var(xError))
print("Y error absolute mean, variance:", np.mean(np.abs(yError)), ',', np.var(yError))

# Graph of relative errors
plot1 = plt.figure(1)
plt.scatter(xError, yError)
plt.ylabel('Y error, pixels')
plt.xlabel('X error, pixels')
plt.title('X and Y error for all points in calibration')

# Intensity chart of errors based on position in image
fig, ax = plt.subplots()
contourPlot = ax.tricontourf(x, y, absError, 100, cmap='magma', extend='both', antialiased=False)
ax.set_title('Absolute error based on position in image')
colorBar = fig.colorbar(contourPlot)
colorBar.ax.set_ylabel('Absolute error')
plt.axis('scaled')
plt.show()

stop("Calibration successful.")