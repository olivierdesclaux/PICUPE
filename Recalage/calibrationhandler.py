import cv2.cv2 as cv2
import matplotlib.pyplot as plt
import json
import os
import numpy as np
# Local modules
from utils import NumpyEncoder, openCalibrationFile


class CalibrationHandler:
    """Calculates calibration and checks errors using points in image and world

    Attributes
    ----------
    objectPositions : list of float points
        Positions in 3D space of the circles of a grid. 
        Can be relative (i.e. first circle is 0, 0)
    imagePositions : list of float points
        Positions in image (pixels) of grid circles
    imageSize : int, int
        Width and height of image in pixels
    minimumImages : int
        Minimum number of grids for calibration
        If the number of grids drops below this, cancel calibration
    maximumPointError : float/int
        Maximum retroprojection error to tolerate on single points
        If a point in a grid exceeds this, the grid (image) is rejected
    calibrationDone : bool
        Indicates if calibration has completed successfully
    retroprojectionError : float
        Average error, in pixels, returned after cv2.calibrate()
    cameraMatrix : 3x3 np array
        Intrinsic camera parameters
    distortion : np array
        8-length (Rational model) intrinsic distortion parameters
    rotation, translation : list of np array, list of np array
        Series of rotational/translation transformations to obtain true grid 
        positions from base grid positions.
        Used to calculate precise retroprojections after calibration,
        for error checking 
    x, y : int, int
        Pixel coordinates of all grid points, for display of errors
    xError, yError, absError : float, float, float
        Distance between image points and retroprojected points
        absError uses a L2 norm, xError and yError use x and y distance
        X and Y errors are for scatter plot, absError is for heatmap
        
    Parameters
    ----------
    See attributes above.

    Methods
    -------
    calibrate() 
        Attempts to perform a complete calibration, including error checking
    displayError(saveDirectory)
        Prints error graphs from calibration and saves image
    writeToFile(saveDirectory)
        Saves intrinsic matrices in .json to saveDirectory
    len()
        Returns length of objectPositions, aka number of grids remaining
    """

    def __init__(self, objectPositions, imagePositions, imageSize, minimumImages, maximumPointError, flags):
        self.objectPositions = objectPositions
        self.imagePositions = imagePositions
        self.imageSize = imageSize
        self.minimumImages = minimumImages
        self.maximumPointError = maximumPointError
        self.retroprojectionError = np.inf
        self.cameraMatrix = []
        self.rotation = []
        self.translation = []
        self.distortion = []
        self.flags = flags
        # Calibration marked as incomplete
        self.calibrationDone = False

        # Keeps track of errors for graphic display
        self.x, self.y = [], []
        self.xError, self.yError, self.absError = [], [], []

    def calibrate(self):
        """Attempts to perform a complete calibration, including error checking
        
        Calibrate relies on cv2.calibrateCamera from OpenCV to generate
        camera instrinsics from objectPoints and imagePoints. It thens uses
        computeErrors() to check if the instrinsics are good.

        Returns
        -------
        calibrationDone : bool
            True if calibration passes error checks
            False if too many images are rejected and 
            the calibration is impossible
        """
        # Checks that there are sufficient images remaining to perform an adequate calibration
        if len(self.objectPositions) >= self.minimumImages:
            # Calculates calibration parameters

            intrinsicsGuess, _ = openCalibrationFile("Results/CalibFLIRInitialGuess.json")
            # flags = cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_USE_INTRINSIC_GUESS

            # intrinsicsGuess = None
            # flags = cv2.CALIB_RATIONAL_MODEL

            self.retroprojectionError, self.cameraMatrix, self.distortion, self.rotation, self.translation = \
                cv2.calibrateCamera(self.objectPositions, self.imagePositions, self.imageSize, intrinsicsGuess, None,
                                    flags=self.flags)
            # Rational model uses 8 params, but 12 are returned (last 4 are 0)
            # self.distortion = self.distortion[:, 0:8]
            # Checks for outliers using calibration parameters
            self.calibrationDone = self.computeErrors()
        else:
            self.calibrationDone = False

        return self.calibrationDone

    def computeErrors(self):
        """Checks calculated intrinsic parameters for errors

        Performs a retroprojection calculation for every single grid.
        If the error on a single grid point exceeds maximumPointError,
        grid is rejected and a new calibration is asked for.

        Returns
        -------
        bool
            If no errors, returns True to signify good calibration
            Otherwise, returns result of recursive call to calibrate() 
            and computeErrors()
            Allows True or False to propagate back to first calibrate() call
        """
        self.x, self.y = [], []
        self.xError, self.yError, self.absError = [], [], []
        # Used to pop images with errors
        indexesToPop = []
        redoCalibration = False
        # Iterates through each image to calculate x and y errors 
        # of all corners
        for index, imagePosOld in enumerate(self.imagePositions):
            # Retroprojects 3D objects to 2D pixel positions in image
            imagePosNew, _ = cv2.projectPoints(self.objectPositions[index], self.rotation[index],
                                               self.translation[index], self.cameraMatrix, self.distortion)
            # Convert list of initially detected points and their 2D reprojections into numpy arrays
            imagePosOld = np.array(imagePosOld).squeeze()
            imagePosNew = np.array(imagePosNew).squeeze()

            # Compute the reprojection error over all points
            error = np.linalg.norm(imagePosNew - imagePosOld, axis=1)
            if np.max(error) > self.maximumPointError:
                # If maximum error is too large, eliminates the image in which the error is found
                indexesToPop.append(index)
                # Asks to recalculate calibration at the end
                redoCalibration = True
                continue
            else:
                self.absError.extend(error)
                self.x.extend(imagePosOld[:, 0].ravel())
                self.y.extend(imagePosOld[:, 1].ravel())
                # Calculates all xErrors/yErrors for image in single list
                self.xError.append(imagePosNew[:, 0].ravel() - imagePosOld[:, 0].ravel())
                self.yError.append(imagePosNew[:, 1].ravel() - imagePosOld[:, 1].ravel())

        # Deletes all outlier frames,
        # in reverse order to prevent dynamic modification of indices
        for index in reversed(indexesToPop):
            self.imagePositions.pop(index)
            self.objectPositions.pop(index)
        # indexesToPop = []

        if redoCalibration:
            # Performs calibrate() and computeErrors() recursively,
            # without the popped images
            return self.calibrate()
        else:
            # If no incorrect points detected, returns True to calibrate()
            return True

    def displayError(self, saveDirectory):
        """ Prints error graphs from calibration and saves image

        Calibration and computeErrors generate two kinds of errors.
        First are x and y errors for each point, which are used to generate
        a scatter plot that can show missed outliers or biases in data.
        Second are absolute total errors, which are used to generate a 
        heatmap of errors to identify problematic areas of the image.
        If errors are concentrated in the corners, it is a sign of faulty 
        distortion coefficients.

        Parameters
        ----------
        saveDirectory : string
            Directory in which to save .png with error graphs

        Returns
        -------
        None.     
        """
        # Checks that error values exist to avoid crashing
        if self.calibrationDone:
            # Overall error printed to command line
            print("Average retroprojection error :", round(self.retroprojectionError, 3))

            # side-by-side axes that cover screen
            fig, (ax1, ax2) = plt.subplots(1, 2)
            fig.set_size_inches(15, 8)

            # Scatter plot of relative errors
            scatterPlot = ax1.scatter(self.xError, self.yError)
            ax1.set_title('X and Y error for points in calibration')
            ax2.set_ylabel('Y error, pixels')
            ax2.set_xlabel('X error, pixels')

            # Intensity chart of errors based on position in image
            contourPlot = ax2.tricontourf(
                self.x, self.y, self.absError, 100, cmap='magma',
                extend='both', antialiased=False)
            ax2.set_title('Absolute errors across image pixels')
            colorBar = fig.colorbar(contourPlot)
            colorBar.ax.set_ylabel('Absolute error')
            plt.axis('scaled')
            # .savefig must come before .show or saved .png will be blank
            plt.savefig(os.path.join(saveDirectory, 'ErrorCharts' + '.png'))
            plt.show()

    def writeToFile(self, saveDirectory, name):
        """ Saves intrinsic matrices in .json to saveDirectory

        Creates a directory at the specified saveDirectory.
        Prints cameraMatrix and distortion coefficients in .json
        format to "Calib.json" within directory.

        Parameters
        ----------
        saveDirectory : string
            Directory in which to save .png with error graphs

        Returns
        -------
        None.     
        """
        # Creates filepath for results of calibration
        if not os.path.isdir(saveDirectory):
            os.mkdir(saveDirectory)
        filename = os.path.join(saveDirectory, name)
        # Outputs calibration matrices to file
        with open(filename, 'w') as file:
            # Assigns labels to values to make JSON readable
            dumpDictionary = {
                'Format': 'OpenCV', 'Model': 'Rational',
                'CameraMatrix': self.cameraMatrix,
                'DistortionCoefficients': self.distortion}
            # Uses NumpyEncoder to convert numpy values to regular arrays
            # for json.dump
            json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)
            print("Succesfully wrote calibration to file.")

        return filename


    def len(self):
        """ Returns length of objectPositions, aka number of grids remaining
        """
        return len(self.objectPositions)
