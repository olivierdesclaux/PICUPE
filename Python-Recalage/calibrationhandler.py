import cv2 as cv
import matplotlib.pyplot as plt
import json
# Local modules
from npencoder import NumpyEncoder

class Calibration:
    """Class for calculating and recalculating calibrations based on image and object points"""
    def __init__(self, name, objectPositions, imagePositions, imageSize, minimumImages, maximumPointError = 10):
        # Assignment
        self.name, self.objectPositions, self.imagePositions, self.imageSize, self.minimumImages, self.maximumPointError = name, objectPositions, imagePositions, imageSize, minimumImages, maximumPointError        
        self.calibrationDone = False
        # Index of popped positions during calculateErrors
        self.poppedIndex = []

    def calibrate(self):
        # Checks that there are sufficient images remaining to perform an adequate calibration
        if len(self.objectPositions) > self.minimumImages:
            # Calculates calibration parameters
            self.retroprojectionError, self.cameraMatrix, self.distortion, self.rotation, self.translation = cv.calibrateCamera(self.objectPositions, self.imagePositions, self.imageSize, None, None, flags=cv.CALIB_RATIONAL_MODEL+cv.CALIB_ZERO_TANGENT_DIST)
            self.distortion = self.distortion[0:7]
            print(self.distortion)
            # Checks for outliers using calibration parameters
            self.calibrationDone = self.calculateErrors()
        else:
            self.calibrationDone = False
        # Returns False if there are too few images once outliers are removed, True if calibration succeeded
        return self.calibrationDone

    def calculateErrors(self):
        self.x, self.y, self.xError, self.yError, self.absError = [], [], [], [], []
        redoCalibration = False
        # Iterates through each image to calculate x and y errors of all corners
        for index, imagePositionOld in enumerate(self.imagePositions):
            # Reprojects objects to positions in image
            imagePositionNew, _ = cv.projectPoints(self.objectPositions[index], self.rotation[index], self.translation[index], self.cameraMatrix, self.distortion)
            imageHasOutliers = False
            # Iterates through each point to calculate more complicated absError (cannot be done directly through list)
            for (oldPoint, newPoint) in zip(imagePositionOld, imagePositionNew):
                # Calculates error using L2 norm (distances squared)
                error = cv.norm(oldPoint[0], newPoint[0], cv.NORM_L2)
                if error > self.maximumPointError:
                    # If error is too large, eliminates the image in which the error is found
                    self.imagePositions.pop(index)
                    self.objectPositions.pop(index)
                    self.poppedIndex.append(index)
                    print("Image removed.")
                    # Indicates that image has outliers
                    imageHasOutliers = True
                    # Asks to recalculate calibration
                    redoCalibration = True
                    # Does not need to check remaining points in this image
                    break
                else:
                    # Appends error to list for graphic representation later, with x and y coordinates
                    self.absError.append(error)
                    self.x.append(oldPoint[0, 0])
                    self.y.append(oldPoint[0, 1])
            if not imageHasOutliers:
                # Kept for graphic representation after calibration
                self.xError.append(imagePositionNew[:,:,0].ravel() - imagePositionOld[:,:,0].ravel())
                self.yError.append(imagePositionNew[:,:,1].ravel() - imagePositionOld[:,:,1].ravel())

        # Redoes calibration without the popped images
        if redoCalibration:
            # Returns False if there are not enough images left at some iteration
            return self.calibrate()
        else:
            # If no incorrect points detected, returns True
            return True

    def displayError(self):
        # Checks that error values exist to avoid crashing
        if self.calibrationDone:
            # Overall error printed to command line
            print(self.name, "average retroprojection error :", round(self.retroprojectionError, 3))

            fig, ax1, ax2 = plt.subplots(2, 1)
            fig.set_size_inches(15, 8)
            # Scatter plot of relative errors
            scatterPlot = ax1.scatter(self.xError, self.yError)
            ax1.set_title('X and Y error for points in ' + self.name + ' calibration')
            ax2.set_ylabel('Y error, pixels')
            ax2.set_xlabel('X error, pixels')
            # Intensity chart of errors based on position in image
            contourPlot = ax2.tricontourf(self.x, self.y, self.absError, 100, cmap='magma', extend='both', antialiased=False)
            ax2.set_title('Absolute error in ' + self.name + ' image')
            colorBar = fig.colorbar(contourPlot)
            colorBar.ax.set_ylabel('Absolute error')
            plt.axis('scaled')
            plt.show()

    def pop(self, poppedIndex):
        # Removes images using a list generated by another Calibration object's calculateErrors()
        # Used to sync images between two Calibration objects from different cameras
        for index in poppedIndex:
            self.imagePositions.pop(index)

    def len(self):
        return len(self.objectPositions)

    def outputToFile(self, filePrefix="CalibrationFile"):
        try:
            filename = filePrefix + self.name + ".json"
            # Outputs calibration matrices to file
            with open(filename, 'w') as file:
                # Assigns labels to values to make JSON readable
                dumpDictionary = {'Format' : 'OpenCV', 'Model' : 'Rational','CameraMatrix' : self.cameraMatrix, 'DistortionCoefficients' : self.distortion[0:7]}
                # Uses NumpyEncoder to convert numpy values to regular arrays for json.dump
                json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)
                return True
        except:
            return False