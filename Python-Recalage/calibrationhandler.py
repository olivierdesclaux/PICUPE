import cv2 as cv
from threading import Thread

class Calibration:
    """Class for calculating and recalculating calibrations based on image and object points"""
    def __init__(self, objectPositions, imagePositions, imageSize, minimumImages, maximumPointError = 10):
        self.objectPositions, self.imagePositions, self.imageSize, self.minimumImages, self.maxmimumPointError = objectPositions, imagePositions, imageSize, minimumImages, maximumPointError        
        self.calibrationDone = False
        self.poppedIndex = []

    def calibrate(self):
        if len(self.objectPositions) > self.minimumImages:
            self.retroprojectionError, self.cameraMatrix, self.distortion, self.rotation, self.translation = cv.calibrateCamera(self.objectPositions, self.imagePositions, self.imageSize, None, None, flags=cv.CALIB_RATIONAL_MODEL+cv.CALIB_ZERO_TANGENT_DIST)
            self.calibrationDone = self.calculateErrors()
        else:
            self.calibrationDone = False
        return self.calibrationDone

    def calculateErrors(self):
        self.x, self.y, self.xError, self.yError, self.absError = [], [], [], [], []
        # Iterates through each image to calculate x and y errors of all corners
        for index, imagePositionOld in enumerate(self.imagePositions):
            # Reprojects objects to positions in image
            imagePositionNew, _ = cv.projectPoints(self.objectPositions[index], self.rotation[index], self.translation[index], self.cameraMatrix, self.distortion)
            self.redoCalibration = False
            # Compares pure x and y errors by subtracting two lists
            self.xError.append(imagePositionNew[:,:,0].ravel() - imagePositionOld[:,:,0].ravel())
            self.yError.append(imagePositionNew[:,:,1].ravel() - imagePositionOld[:,:,1].ravel())
            self.x.append(imagePositionOld[:,:,0].ravel())
            self.y.append(imagePositionOld[:,:,1].ravel())
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
                    # Asks to recalculate calibration
                    self.redoCalibration = True
                    # Does not need to check remaining points in this image
                    break
                else:
                    self.absError.append(error)
            # Redoes calibration without the popped image 
            # ignores remaining images, they will be checked when self.calibrate() calls self.calculateErrors()
            if self.redoCalibration:
                # Returns False if there are not enough images left at some iteration
                return self.calibrate()
        # If no incorrect points detected, returns True
        return True

    def pop(self, indexToPop):
        for index in indexToPop:
            self.imagePositions.pop(index)
            self.objectPositions.pop(index)
