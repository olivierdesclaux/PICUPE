import cv2.cv2 as cv2
import sys
import json
import numpy as np
import argparse


def stop(message, streams):
    """Quits program cleanly and closes streams    

    Parameters
    ----------
    message : string
        Message to print upon program exit
    streams : list of VideoStream, optional
        Streams to attempt to stop
        Invalid streams will not raise Exceptions (simplifies use)

    Returns
    -------
    None.
    """
    cv2.destroyAllWindows()
    for stream in streams:
        try:
            stream.stop()
        except:
            pass
    sys.exit(str(message))


def openCalibrationFile(filename):
    """Opens a file with camera intrinsic parameters

    Parameters
    ----------
    filename : string
        Name of file with intrinsics
        Format must be that of calibrationhandler.py
        Relative path possible

    Returns
    -------
    matrix : np array
        3x3 camera intrinsic parameters
    distortion : np array
        Intrinsic distortion coefficients of camera
        Length depends on model used when saving file (8 for Rational)
    """
    with open(filename, 'r') as file:
        calibrationData = json.load(file)
        matrix = np.asarray(calibrationData['CameraMatrix'])
        distortion = np.asarray(calibrationData['DistortionCoefficients'])
        return matrix, distortion


def openRectFile(filename):
    """Open a file with intrinsic and extrinsic parameters for 2 cameras

    Parameters
    ----------
    filename : string
        Name of file with intrinsics and extrinsics
        Format must be that of rectify.py
        Relative path possible
        Expects parameters for 2 cameras only

    Returns
    -------
    matrices : list of np array
        2 sets of 3x3 camera intrinsic parameters
    dists : list of np array
        2 sets of intrinsic distortion coefficients 
        Length depends on model used when saving file (8 for Rational)
    Rs : list of np array
        2 sets of R matrices for rectification rotation
    Ps : list of np array
        2 sets of P matrices for rectification projection
    rois : list of np array
        2 sets of regions of interest in rectified camera views
        ROIs are used to crop images to only useful area
        Format is x, y, w, h
        Unfortunately, does not work if cameras have different resolutions
    """
    with open(filename, 'r') as file:
        rectData = json.load(file)
        dists = [np.asarray(rectData['Dist1']), np.asarray(rectData['Dist2'])]
        matrices = [np.asarray(rectData['Matrix1']),
                    np.asarray(rectData['Matrix2'])]
        Rs = [np.asarray(rectData['R1']), np.asarray(rectData['R2'])]
        Ps = [np.asarray(rectData['P1']), np.asarray(rectData['P2'])]
        rois = [np.asarray(rectData['ROI1']), np.asarray(rectData['ROI2'])]
        return matrices, dists, Rs, Ps, rois


def openStereoCalibrationFile(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    matrixLeft = np.asarray(data['Matrix1'])
    matrixRight = np.asarray(data['Matrix2'])
    distLeft = np.asarray(data['Dist1'])
    distRight = np.asarray(data['Dist2'])
    R = np.asarray(data['R'])
    T = np.asarray(data['T'])
    frameSize = data['frameSize']
    return matrixLeft, distLeft, matrixRight, distRight, frameSize, R, T


def findBoundingRectangle(frame):
    """Searches for cropping rectangle in frame

    If shape of non-zero pixels (region of interest) is tilted, 
    rectangle will be inclusive (includes all non-zeros, even if
    this means including some zero pixels)

    Parameters
    ----------
    frame : OpenCV image
        Frame in which to search for bounding rectangle
        All pixels not inside desired rectangle must be 0

    Returns
    -------
    boundingRectangle : np array
        Array of x, y, w, h describing rectangle in which to crop
    """
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(grey, 1, 255, cv2.THRESH_BINARY)
    nonZeroPoints = cv2.findNonZero(thresh)
    return cv2.boundingRect(nonZeroPoints)


def calculateOptimalMatrix(frame, cameraMatrix, distortion):
    """Convenience wrapper for cv2.getOptimalNewCameraMatrix()

    Parameters
    ----------
    frame : OpenCV image
        Frame whose dimensions are used to calculate optimal matrix
    cameraMatrix : 3x3 np array
        Intrinsic parameters of camera
    distortion : np array
        Intrinsic distortion parameters
        Can be any OpenCV length (4, 5, 8, 14)

    Returns
    -------
    newCameraMatrix : 3x3 np array
        New intrinsic parameters of camera
    """
    h, w = frame.shape[:2]
    return cv2.getOptimalNewCameraMatrix(
        cameraMatrix, distortion, (w, h), 1, (w, h))


def scaleForHconcat(referenceImage, targetImage, scalingFactor=1.0):
    """Scales a targetImage to same height as reference image

    Also provides global scaling with scalingFactor
    Does not modify in place (makes copies)

    Parameters
    ----------
    referenceImage : OpenCV image
        Image on which targetImage's new height is based
        Height of referenceImage will be intialHeight * scalingFactor
    targetImage : OpenCV image
        Image that will be modified to match targetImage's new height
    scalingFactor : float, default=1.0
        Factor by which to scale both images (for better display)

    Returns
    -------
    resizedReferenceImage : OpenCV image
        Reference image with new dimensions
    resizedTargetImage : OpenCV image
        Target image with new dimensions
    """
    # Scales images to have same height for hconcat side-by-side display
    # Preserves aspect ratio of target
    targetRatio = targetImage.shape[1] / targetImage.shape[0]

    # Calculates new dimensions based on ratio, referenceImage height and a potential scaling factor
    referenceH, referenceW = referenceImage.shape[0:2]
    targetDim = (int(referenceH * targetRatio * scalingFactor), int(referenceH * scalingFactor))
    referenceDim = (int(referenceW * scalingFactor), int(referenceH * scalingFactor))
    # Resizes images with calculated dimensions
    resizedTargetImage = cv2.resize(targetImage, targetDim)
    resizedReferenceImage = cv2.resize(referenceImage, referenceDim)
    # Returns but also modifies images in place anyways
    return resizedReferenceImage, resizedTargetImage


# Encodes numpy arrays as normal lists
# Otherwise, they cannot be encode to JSON
# Used when invoking json.dump as cls param
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


# Allows argpase to accept "extend" action from python 3.8,
# so that multiple strings can be used with a single arg
# Add to parser using parser.register('action', 'extend', ExtendAction)
# Invoke in parser.add_argument() using action='extend'
class ExtendAction(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        items = getattr(namespace, self.dest) or []
        items.extend(values)
        setattr(namespace, self.dest, items)
