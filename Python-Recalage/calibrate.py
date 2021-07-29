import sys
import numpy as np
import cv2 as cv
import time
import json
import matplotlib.pyplot as plt
import pyk4a as k4a

## Encoder for Numpy arrays to JSON
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

# Convenience function for killing open objects
def stop(message):
    cv.destroyAllWindows()
    if 'k' in camerasToOpen:
        kinect.stop()
    if 'f' in camerasToOpen:
        if flir is not None:
            flir.release()
    sys.exit(str(message))

# Default camera to open
camerasToOpen = ""
# Allows selection of cameras via command-line arguments
if len(sys.argv) > 1:
    cameraName = str(sys.argv[1])
    if cameraName in ["k", "f", "kf"]:
        camerasToOpen = cameraName
    else:
        stop("No camera with that code. Pleas use 'k', 'f' or 'kf'.")

# Corner finder variables
numberOfRows = 12
numberOfColumns = 12
totalCircles = numberOfRows*numberOfColumns
checkerboardSize = (numberOfRows, numberOfColumns)
distanceBetweenRows = 30 # In mm
# Termination criteria for subpixel finding : 1st parameter is flag, 2nd is maximum iterations, 3rd is maximum epsilon
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Array of float32 points
# Last axis (z) is always 0
# Y axis (rows) increments 0, 1, 2, ... numberOfRows-1
# X axis (columns) increments 0, 1, 2, ... numberOfColumns-1
objectCorners = np.array([[x % numberOfColumns, np.floor(x / numberOfColumns), 0] for x in range(totalCircles)], np.float32)
# Blob detector
blobParams = cv.SimpleBlobDetector_Params()
blobParams.minDistBetweenBlobs = 4
# Thresholds used to simplify image for detector
blobParams.minThreshold = 30
blobParams.maxThreshold = 240 

# Filter by area
blobParams.filterByArea = True
blobParams.minArea = 25 # In pixels (circle diameter 5 px)
blobParams.maxArea = 10000 # In pixels

# Filter by circularity
blobParams.filterByCircularity = True
blobParams.minCircularity = 0.1 # Weak circularity filtering for perspective (circles become ellipses)

# Filter by convexity
blobParams.filterByConvexity = True
blobParams.minConvexity = 0.94 # Strong convexity filtering for circles

# Filter by inertia
blobParams.filterByInertia = True
blobParams.minInertiaRatio = 0.01 # Weak elongation filtering for perspective (circles become ellipses)

# Create a detector with the parameters
blobDetector = cv.SimpleBlobDetector_create(blobParams)

# Arrays to store object points and image points from all the images.
objectPositions = [] # 3d point in real world space
imagePositions = [] # 2d points in image plane.
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

# Open FLIR directly with OpenCV
if "f" in camerasToOpen:
    # Loops through all cameras connected via USB
    cameraIndex = 1
    while True:
        cap = cv.VideoCapture(cameraIndex)
        # If we reach the end of available cameras
        if not cap.isOpened():
            sys.exit("Cannot open FLIR camera")
        # Uses unique frame height to identify FLIR T1020
        if int(cap.get(cv.CAP_PROP_FRAME_HEIGHT)) == 768:
            cap.set(cv.CAP_PROP_BUFFERSIZE, 3)
            flir = cap
            break
        else:
            cap.release()
            cameraIndex += 1

if "k" in camerasToOpen:
    # Open Kinect with pyK4a for more options
    kinect = k4a.PyK4A(k4a.Config(
                color_format=k4a.ImageFormat.COLOR_BGRA32,
                color_resolution=k4a.ColorResolution.RES_720P,
                depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                camera_fps=k4a.FPS.FPS_30,
                synchronized_images_only=True,
            ))
    kinect.start()
    #kinect.whitebalance = 2800
    
    # Test capture to check if kinect is open
    capture = kinect.get_capture()
    if not np.any(capture):
        stop("Cannot open Azure Kinect.")

calibrationCompleted = False
while not calibrationCompleted:
    while len(imagePositions) < maximumCalibrationImages:
        # Get next frame from Kinect 
        if "k" in camerasToOpen:
            kinectCapture = kinect.get_capture()
            kinectCapture = kinectCapture.color
            if not np.any(capture):
                stop("Kinect disconnnected, not enough frames taken for calibration.")

        if "f" in camerasToOpen:
            flirOpen, flirCapture = flir.read()
            if not flirOpen:
                stop("FLIR disconnnected, not enough frames taken for calibration.")

        # TEMPORARY to observe IR image
        frame = flirCapture
        if frame is not None:
            # Makes frame gray
            grayFrame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # Waits secondsToSkip between checkerboards
            if (time.time() - lastCheckerboardTime > secondsToSkip):
                # Detects blobs and draws them on the frame
                keypoints = blobDetector.detect(grayFrame)
                frame = cv.drawKeypoints(frame, keypoints, np.array([]), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                # Tries to find chessboard corners
                boardIsFound, imageCorners = cv.findCirclesGrid(grayFrame, checkerboardSize, flags=cv.CALIB_CB_SYMMETRIC_GRID+cv.CALIB_CB_CLUSTERING, blobDetector=blobDetector)

                if boardIsFound == True:
                    # Add them to permanent lists
                    objectPositions.append(objectCorners)
                    imagePositions.append(imageCorners)

                    # Resets time counter
                    lastCheckerboardTime = time.time()

            # Draws previously used checkerboards
            for index, pos in enumerate(imagePositions):
                points = np.array([pos[0], pos[numberOfColumns - 1], pos[totalCircles - 1], pos[numberOfColumns*(numberOfRows - 1)]], np.int32) 
                if index == len(imagePositions) -1:
                    poylgonColor = (240, 20, 20)
                else:
                    poylgonColor = (20, 220, 90)
                cv.polylines(frame, [points], True, poylgonColor, 2)

            # Indicates how many images remain
            textColor = (80, 250, 55)
            cv.putText(frame, 'Remaining images: ' + str(maximumCalibrationImages - len(imagePositions)), (50, 50), cv.FONT_HERSHEY_SIMPLEX, 1, textColor, 2)
            
            cv.imshow('frame', frame)
            ## Waits 25 ms for next frame or quits main loop if 'q' is pressed
            if cv.waitKey(25) == ord('q'):
                stop("User exited program.")

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