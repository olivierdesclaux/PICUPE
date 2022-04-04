import multiprocessing
import threading
import cv2.cv2 as cv2
import sys
import time
from datetime import datetime
import numpy as np
import json
import pyk4a as k4a
import os

sys.path.append("../")
from Recalage.cameraUtils import openCalibrationFile, scaleForHconcat, NumpyEncoder
from Recalage.circledetector import CircleDetector
from utils.readerWriterClass import findCameraPort
from utils.myLogger import Logger

class StereoCalibrator:
    """
    Main class for performing stereocalibration between flir and kinect.
    """
    def __init__(self, savePath, names, calibFiles, logger, initialGrids=15, minGrids=10):
        self.savePath = savePath
        self.logger = logger
        # Global variable. All threads/processes run as long as this value is equal to 1
        self.running = multiprocessing.Value('i', 1)

        # Setting up left and right attributes
        self.leftName = names[0]  # flir in our case
        self.rightName = names[1]  # Kinect in our case
        self.leftQueue = multiprocessing.Queue()
        self.rightQueue = multiprocessing.Queue()
        self.leftModality, self.leftFrameSize = self.getModality(self.leftName)
        self.rightModality, self.rightFrameSize = self.getModality(self.rightName)
        self.leftCalibFile = calibFiles[0]
        self.rightCalibFile = calibFiles[1]
        # Retrieve intrinsics and distortion parameters from calibration files
        if type(self.leftCalibFile) == str:
            self.leftMatrix, self.leftDist = openCalibrationFile(self.leftCalibFile)
        else:
            self.leftMatrix, self.leftDist = self.leftCalibFile[0], self.leftCalibFile[1]
        if type(self.rightCalibFile) == str:
            self.rightMatrix, self.rightDist = openCalibrationFile(self.rightCalibFile)
        else:
            self.rightMatrix, self.rightDist = self.rightCalibFile[0], self.rightCalibFile[1]
        # If matrices return empty, quit as well
        if not self.leftDist.any() or not self.rightDist.any():
            raise Exception("Cannot read matrices from calibration files. Format may be incorrect.")

        self.frames = None  # List, will be [leftFrame, rightFrame]

        # Hyperparameters
        self.flags = cv2.CALIB_FIX_INTRINSIC  # cf. OpenCV doc
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)  # cf. OpenCV doc
        self.initialGrids = initialGrids
        self.minGrids = minGrids
        self.allGridsFound = False  # Bool for checking if self.initialGrids were detected
        self.epipolarThreshold = 3  # Threshold for epipolar constraint metric
        self.secondsToSkip = 2.0  # Time to wait for after the calibration object has been detected (to let user move
        # the object around).
        self.rows = 9  # number of rows of the calibration object
        self.cols = 15  # number of cols of the calibration object
        self.boardSize = (self.rows, self.cols)
        self.circleDetector = CircleDetector()
        # Physical distance between calibration object centers
        self.objectCorners = 21.2 * np.array([[2 * (x % self.rows) + np.floor(x / self.rows) % 2,
                                               np.floor(x / self.rows), 0] for x in range(self.rows * self.cols)],
                                             np.float32)

        # Initialising matrices needed to perform stereocalibration
        self.positions3D = []  # Will be filled iteratively with self.objectCorners
        self.left2D = []  # Center positions (in px) of the calib object seen in the left view
        self.right2D = []  # Same but for the right

        # Initialising processes and threads for acquiring and processing data
        self.leftCamProcess = None  # Sub-process connected to left Cam to acquire data and put in self.leftQueue
        self.rightCamProcess = None  # Sub-process connected to right Cam to acquire data and put in self.rightQueue
        self.updateThread = None  # Thread in main process: Reads data in both queues and stores it in self.frames
        self.circleFinderThread = None  # Thread in main process: Catches self.frames and looks for circles in both
        # frames

        # Initialising outputs
        self.retError = None
        self.R = None
        self.T = None
        self.E = None
        self.F = None

        self.tsDeltas = []
        self.aborted = False  # In case user aborts stereo by pressing q.

    @staticmethod
    def getModality(name):
        """
        Gets the associated modality and image size of a specified camera
        Parameters
        ----------
        name: str, "webcam", "flir" or "kinect"

        Returns
        -------
        modality: str, rgb or ir
        frameSize: tuple, (int, int) image size as (width, height)
        """
        if name == "webcam":
            frameSize = (640, 480)
            modality = "rgb"
        elif name == "flir":
            frameSize = (1024, 768)
            modality = "ir"
        elif name == "kinect":
            frameSize = (1020, 720)
            modality = "rgb"
        else:
            raise Exception("Wrong Name")
        return modality, frameSize

    def performStereocalibration(self):
        """
        Main function for performing stereocalibration. Saves results of stereocalibration to a stereo.json file
        Returns
        -------
        None
        """
        while self.running.value:
            self.start()  # Start processes and threads
            while not self.allGridsFound:
                if self.frames is not None:
                    [leftFrameOriginal, rightFrameOriginal] = self.frames
                    rightFrameOriginal = rightFrameOriginal[:, :, :3]  # Remove transparency from kinect
                    # Copy image so that we can draw on it.
                    leftFrame = leftFrameOriginal.copy()
                    rightFrame = rightFrameOriginal.copy()

                    # Draw all the boards that have already been found.
                    self.drawOutlines(leftFrame, self.left2D)
                    self.drawOutlines(rightFrame, self.right2D)
                    # Resize for imshow
                    resizedFrameLeft, resizedFrameRight = scaleForHconcat(leftFrame, rightFrame, 0.6)
                    concatFrame = cv2.hconcat([resizedFrameLeft, resizedFrameRight])
                    cv2.putText(concatFrame, 'Remaining images: ' + str(self.initialGrids - self.len()), (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
                    cv2.imshow('Stereo Calibration', concatFrame)

                    if cv2.waitKey(1) == ord('q'):
                        with self.running.get_lock():
                            self.running.value = 0
                        self.aborted = True
                        return

                else:
                    cv2.waitKey(10)
            # Stop all ongoing acquisition processes before computing stereocalibration
            self.stop()

            self.logger.log("Computing matrices")
            stereoSuccess = self.computeMatrices()
            if stereoSuccess:
                if not os.path.isdir(self.savePath):
                    os.mkdir(self.savePath)
                filename = os.path.join(self.savePath, "stereo.json")
                with open(filename, 'w') as file:
                    # Assigns labels to values to make JSON readable
                    dumpDictionary = {'Format': 'OpenCV',
                                      'Matrix1': self.leftMatrix,
                                      'Matrix2': self.rightMatrix,
                                      'Dist1': self.leftDist,
                                      'Dist2': self.rightDist,
                                      'R': self.R,
                                      'T': self.T,
                                      'E': self.E,
                                      'F': self.F,
                                      'frameSize': self.leftFrameSize,
                                      'Error': self.retError}
                    # Uses NumpyEncoder to convert numpy values
                    # to regular arrays for json.dump
                    json.dump(dumpDictionary, file, indent=4, cls=NumpyEncoder)
                self.logger.log("Stereocalibration was successful.")
            else:
                self.logger.log("Stereocalib failed. New images need to be taken")
                # if the stereocalibration failed, we launch the acquisition processes/threads again
                with self.running.get_lock():
                    self.running.value = 1

    def update(self):
        """
        Reads from left and right camera queues and stores images in self.frames
        Returns
        -------
        None
        """
        while self.running.value:
            try:
                leftFrame, leftTs = self.leftQueue.get_nowait()
                rightFrame, rightTs = self.rightQueue.get_nowait()
                self.frames = [leftFrame, rightFrame]
                self.ts = [leftTs, rightTs]
            except:
                continue

    def startCircleFinder(self):
        """
        Initialises the thread that looks for circles of the calibration object in both frames.
        Returns
        -------
        None
        """
        with self.running.get_lock():
            self.running.value = 1
        self.allGridsFound = False
        if self.circleFinderThread is None:
            self.circleFinderThread = threading.Thread(target=self.findCircles, args=(), daemon=True)
        self.circleFinderThread.start()

    def findCircles(self):
        """
        Threaded function that applies a CircleDetector object on left and right frames. If the calibration object is
        detected in both views, self.positions3D, self.left2D and self.right2D are updated.

        Returns
        -------
        None
        """
        while self.running.value and self.len() < self.initialGrids:
            if self.frames is not None:
                [leftFrameOriginal, rightFrameOriginal] = self.frames
                rightFrameOriginal = rightFrameOriginal[:, :, :3]  # Remove transparency
                # Convert to "grayscale"
                leftGrayFrame = self.grayify(leftFrameOriginal, "left")
                rightGrayFrame = self.grayify(rightFrameOriginal, "right")

                frames = [leftGrayFrame, rightGrayFrame]
                positions = []
                for frame in frames:
                    ret, pos = cv2.findCirclesGrid(frame, self.boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                   blobDetector=self.circleDetector.get())
                    if not ret:
                        break
                    else:
                        positions.append(pos)
                if len(positions) == 2:
                    self.positions3D.append(self.objectCorners)
                    self.left2D.append(positions[0])
                    self.right2D.append(positions[1])
                    time.sleep(self.secondsToSkip)  # Wait 2 secs before looking for circles again, so that user can
                    # move the board around.
            else:
                cv2.waitKey(10)

        self.allGridsFound = True
        return True

    def grayify(self, frame, side):
        """
        Converts frames to grayscale based on frame type. Actually, for the RGB, we only want to subtract the red
        channel from the blue channel. This will make the red color of the checkerboard stand out really well,
        instead of a standard grayscale.
        Parameters
        ----------
        frame: np.array, frame to convert
        side: str, "left" or "right", to know the associated modality and therefore grayification method

        Returns
        -------
        grayified: np.array, grayscale image
        """
        if side == "left":
            modality = self.leftModality
        elif side == "right":
            modality = self.rightModality
        else:
            raise Exception("Wrong camera side")

        if modality == "rgb":
            grayified = cv2.subtract(frame[:, :, 2], frame[:, :, 0])
        elif modality == "ir":
            grayified = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            raise Exception("Wrong modality")

        return grayified

    def drawOutlines(self, frame, positions):
        """
        Draws calibration object outlines on corresponding frame. Will just draw the polygon passing by the 4 most
        outside centers.
        Drawing is made inplace.
        Parameters
        ----------
        frame: np.array, frame to draw on.
        positions: list, 2D positions of all the centers

        Returns
        -------
        None
        """
        # noinspection PyTypeChecker
        if len(positions) > 0:
            for pos in positions:
                # Uses corners of each calibration grid as points for lines
                points = np.array([pos[0], pos[self.rows - 1],
                                   pos[self.cols * self.rows - 1],
                                   pos[self.rows * (self.cols - 1)]], np.int32)
                # Draws lines in place on arguments
                cv2.polylines(frame, [points], True, (20, 220, 90), 2)

    def len(self):
        """
        Get number of found chessboards
        Returns
        -------
        int, number of found chessboards.
        """
        return len(self.positions3D)

    def start(self):
        """
        Initialises all processes/threads for acquisition
        Returns
        -------
        None
        """
        barrier = multiprocessing.Barrier(2)  # Barrier to make sure that the
        camStatus = multiprocessing.Array('i', 2)

        # Circle Finder
        self.startCircleFinder()

        # Update thread that retrieves frames from left and right camera queues
        if self.updateThread is None:
            self.updateThread = threading.Thread(target=self.update, args=(), daemon=True)
        self.updateThread.start()

        # Process for left camera (FLIR in our case)
        if self.leftCamProcess is None:
            flirPort = findCameraPort(self.leftName)
            leftCam = flirReader(flirPort, self.leftQueue, self.running, camStatus, barrier)
            self.leftCamProcess = multiprocessing.Process(target=leftCam.read, args=(), daemon=True)
        self.leftCamProcess.start()

        # Process for right camera (Kinect in our case)
        if self.rightCamProcess is None:
            rightCam = kinectReader(self.rightQueue, self.running, camStatus, barrier)
            self.rightCamProcess = multiprocessing.Process(target=rightCam.read, args=(), daemon=True)
        self.rightCamProcess.start()

        self.logger.log("Started processes.")

    def stop(self, message=None):
        """
        Stops all the processes/threads. Optionally raises an exception with a message for explaining reason of stop.
        Parameters
        ----------
        message: str, optional. Message that will be raised.

        Returns
        -------
        None
        """

        # Set the shared running variable to 0. All while loops in the different processes/threads will break
        with self.running.get_lock():
            self.running.value = 0

        # Empty Queues
        self.emptyQueues()

        # Wait for all threads to finish, and force stopping of processes.
        if self.circleFinderThread is not None:
            self.circleFinderThread.join()
        if self.updateThread is not None:
            self.updateThread.join()
        if self.leftCamProcess is not None:
            self.leftCamProcess.join()
        if self.rightCamProcess is not None:
            self.rightCamProcess.join()

        # Reset all threads and processes
        self.circleFinderThread = None
        self.updateThread = None
        self.leftCamProcess = None
        self.rightCamProcess = None

        cv2.destroyAllWindows()
        if message is not None:
            raise Exception(message)
        self.logger.log("Stopped processes.")

    def emptyQueues(self):
        """
        Fetches elements in the left and right queues until they are empty. Processes won't join if their queues
        aren't empty.
        Returns
        -------
        True, once finished.
        """
        while True:
            try:
                self.leftQueue.get_nowait()
            except:
                break
        while True:
            try:
                self.rightQueue.get_nowait()
            except:
                break
        return True

    def computeMatrices(self):
        """
        Calls the cv2.stereoCalibrate function iteratively. We start with an initial number of grids. We compute the
        matrices, and the associated epipolar error with self.computeErrors(). This function will remove the "bad"
        images from self.positions3D, self.left2D and self.left3D. We check if we still have enough images left (more
        than self.mingrids) and if so, we recompute stereocalibration.

        Returns
        -------
        SUCCESS: bool, True if stereocalibration worked. False otherwise (meaning more images need to be captured).
        """
        SUCCESS = False
        while not SUCCESS:
            if len(self.positions3D) > self.minGrids:  # Check we have enough grids.
                self.retError, self.leftMatrix, self.leftDist, self.rightMatrix, self.rightDist, \
                self.R, self.T, self.E, self.F = cv2.stereoCalibrate(self.positions3D, self.left2D, self.right2D,
                                                                     self.leftMatrix, self.leftDist, self.rightMatrix,
                                                                     self.rightDist, self.leftFrameSize,
                                                                     flags=self.flags, criteria=self.criteria)
                SUCCESS = self.computeErrors()

            else:
                SUCCESS = False
                break

        return SUCCESS

    def computeErrors(self):
        """
        computes the epipolar constraint metric for different views and removes views that have an error greater than
        self.epipolarThreshold. All the views that have an error higher than the threshold will be eliminated from
        the stored views, and we will recompute the stereocalibration matrices without them.

        Returns
        -------
        SUCCESS: bool, True if all views have an  error below the threshold, False otherwise
        """
        SUCCESS = True
        indices2remove = []  # For storing indices of views that have too big an error.
        # errors = []
        for i, (posLeft, posRight) in enumerate(zip(self.left2D, self.right2D)):
            error = epipolarConstraintMetric(self.F, posLeft, posRight,
                                             self.leftMatrix, self.leftDist,
                                             self.rightMatrix, self.rightDist,
                                             False)
            # errors.append(error)
            if error > self.epipolarThreshold:
                indices2remove.append(i)
                SUCCESS = False

        self.logger.log("Removed {} frames".format(len(indices2remove)))
        self.left2D = [self.left2D[i] for i in range(len(self.left2D)) if i not in indices2remove]
        self.right2D = [self.right2D[i] for i in range(len(self.right2D)) if i not in indices2remove]
        self.positions3D = self.positions3D[: len(self.left2D)]

        return SUCCESS


def epipolarConstraintMetric(F, leftPts, rightPts, leftMatrix, leftDist, rightMatrix,
                             rightDist, VIZ):
    """
    Computes the epipolar constraint metric. For the points in the left view, we compute the epipolar line in the
    right view. For each (leftPoint, epipolarLine) the associated rightPoint should lie on the epipolarLine. We
    therefore look at the distance between rightPoint and epipolarLine. We also do this the other way around.
    Parameters
    ----------
    F: np.array, fundamental matrix linking the left and right cameras
    leftPts: np.array, 2D positions of points in the left view
    rightPts: np.array, 2D positions of points in the right view
    leftMatrix: np.array, intrinsics of the left camera
    leftDist: np.array, distortion parameters of the left camera
    rightMatrix: np.array, intrinsics of the right camera
    rightDist: np.array, distortion parameters of the right camera
    VIZ

    Returns
    -------

    """
    leftDist = list(leftDist)  # QUICKFIX TO BE FIXED. Can't remember in which scenario leftDist is an array.
    # Undistort checkerboard centers
    posLeftUndistorted = undistortPoints(leftPts, leftMatrix, leftDist)
    posRightUndistorted = undistortPoints(rightPts, rightMatrix, rightDist)

    # Compute the epipolar lines.
    # leftLines are the product of F.T * x for x in left Frame
    # rightLines are the product of F * x for x in right Frame
    leftLines = cv2.computeCorrespondEpilines(posLeftUndistorted, 1, F)
    rightLines = cv2.computeCorrespondEpilines(posRightUndistorted, 2, F)

    # Convert 2 homogenous
    posLeftUndistorted_homogeneous = np.ones_like(leftLines)
    posLeftUndistorted_homogeneous[:, :, :2] = posLeftUndistorted

    posRightUndistorted_homogeneous = np.ones_like(rightLines)
    posRightUndistorted_homogeneous[:, :, :2] = posRightUndistorted

    # Compute distances from each point to its associated line.
    # Line equation is of type (a, b, c) so distance of point (x, y) to line is simply:
    #  ax + by + c
    # According to epipolar geometry, this distance should be as close as possible to 0

    # For each view, compute the absolute distance, and then get mean result.
    leftDists = np.mean(np.abs(np.sum(posLeftUndistorted_homogeneous * rightLines, axis=-1)))
    rightDists = np.mean(np.abs(np.sum(posRightUndistorted_homogeneous * leftLines, axis=-1)))

    error = np.mean([leftDists, rightDists])

    # For each pair of view, we have 270 distances in total: 135 for left view, 135 for the right view.
    # The final error is the mean epipolar distance over these 270 values.
    return error


def undistort(xy, k, distortion, iter_num=3):
    """
    Manual undistortion function, taken from:
    https://yangyushi.github.io/code/2020/03/04/opencv-undistort.html
    cv2.undistort function gave strange results, and this was visually correct.

    Parameters
    ----------
    xy: list, point coordinates [x, y]
    k: np.array, camera intrinsic parameters
    distortion: np.array, camera distortion parameters
    iter_num: int, number of iterations for undistortion.

    Returns
    -------
    list, undistorted position of xy
    """
    if len(xy) == 1:
        xy = xy[0]
        [k1, k2, p1, p2, k3] = distortion
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[:2, 2]
    [x, y] = xy.astype(float)
    x = (x - cx) / fx
    x0 = x
    y = (y - cy) / fy
    y0 = y
    for _ in range(iter_num):
        r2 = x ** 2 + y ** 2
        k_inv = 1 / (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3)
        delta_x = 2 * p1 * x * y + p2 * (r2 + 2 * x ** 2)
        delta_y = p1 * (r2 + 2 * y ** 2) + 2 * p2 * x * y
        x = (x0 - delta_x) * k_inv
        y = (y0 - delta_y) * k_inv

    return [[x * fx + cx, y * fy + cy]]


def undistortPoints(points, k, distortion):
    """
    undistorts a set of individual points
    Parameters
    ----------
    points: list of [x,y] coordinates of points to undistort
    k: np.array, camera intrinsics parameters
    distortion: np.array, camera distortion parameters.

    Returns
    -------
    pointsUndistorted: np.array, same shape as points, undistorted coordinates.
    """
    pointsUndistorted = []  # Initialise output.

    for point in points:
        pointsUndistorted.append(undistort(point, k, distortion))
    pointsUndistorted = np.array(pointsUndistorted).astype("float32")

    return pointsUndistorted


def drawLines(img, lines, colors, width):
    """
    For visualization purposes only. Used to draw lines on an image.
    Parameters
    ----------
    img: np.array, image to draw on
    lines: list, list of lines stored in format [a,b,c].
    colors: associated colors for each line
    width: line width

    Returns
    -------
    None, drawing is done inplace.
    """
    _, c, _ = img.shape
    # color = (0, 255, 0)
    for r, color in zip(lines, colors):
        color = (int(color[0]), int(color[1]), int(color[2]))
        r = r[0]
        x0, y0 = map(int, [0, -r[2] / r[1]])
        x1, y1 = map(int, [c, -(r[2] + r[0] * c) / r[1]])
        cv2.line(img, (x0, y0), (x1, y1), tuple(color), width)


class flirReader:
    """
    Class for interfacing with the flir and reading data.
    """
    def __init__(self, port, queue, running, camStatus, barrier):
        self.port = port  # Port number for accessing flir
        self.queue = queue  # Queue for putting images
        self.running = running  # Telling the camera to continue acquiring.
        self.camStatus = camStatus  # Shared array between processes to make sure cameras are ready together.
        self.barrier = barrier  # multiprocessing.Barrier to wait between processes before adding new frame in the
        # queue.
        self.cam = None  # Initialise videocapture object.

    def read(self):
        """
        Main reading function.
        Returns
        -------
        None
        """
        self.initCam()  # Create videocapture object
        self.cam.read()  # Read a frame to finalise init. (Sometimes grabbing the first frame can take some time)
        self.camStatus[0] = 1
        # Wait for other camera to be ready.
        while np.sum(self.camStatus) < len(self.camStatus):
            continue
        while self.running.value:
            self.cam.grab()  # Grab bytestream from camera
            ts = datetime.now()
            ret, newFrame = self.cam.retrieve()  # Decode stream
            if not ret:
                break
            else:
                self.queue.put([newFrame, ts])  # add frame to queue
            self.barrier.wait()  # Wait for other camera to finish putting it's frame in associated queue.
        self.cam.release()
        # Have to make sure that the queue is empty for terminating process
        while not self.queue.empty():
            self.queue.get_nowait()

    def initCam(self):
        """
        Initialises a cv2.VideoCapture object.
        Returns
        -------

        """
        cam = cv2.VideoCapture(self.port, cv2.CAP_DSHOW)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cam = cam


class kinectReader:
    def __init__(self, queue, running, camStatus, barrier):
        self.queue = queue  # Queue for putting images
        self.running = running  # Telling the camera to continue acquiring.
        self.camStatus = camStatus  # Shared array between processes to make sure cameras are ready together.
        self.barrier = barrier  # multiprocessing.Barrier to wait between processes before adding new frame in the
        # queue.
        self.frame = None  # Will store the lastly acquired frame
        self.kinect = None  # Initialise pyk4a object
        self.timestamp = datetime.now()

    # def update(self):
    #     """
    #     Threaded function that continuously reads a frame from the kinect and updates self.frame
    #
    #     Returns
    #     -------
    #     None
    #     """
    #     kinect = k4a.PyK4A(k4a.Config(
    #         color_resolution=k4a.ColorResolution.RES_720P,
    #         depth_mode=k4a.DepthMode.NFOV_UNBINNED,
    #         camera_fps=k4a.FPS.FPS_30,
    #         synchronized_images_only=True, ))
    #     kinect.start()
    #     self.kinect = kinect
    #     self.frame = self.kinect.get_capture()
    #     self.timestamp = datetime.now()
    #     # self.camStatus[1] = 1  # Indicate that camera is ready.
    #
    #     while self.running.value:
    #         self.frame = self.kinect.get_capture()
    #         self.timestamp = datetime.now()
    #         cv2.waitKey(1)  # VERY IMPORTANT. Have to wait a bit in while loop otherwise the thread will completely
    #         # overload the process that spawned it.
    #
    #     self.kinect.stop()
    #
    # def read2(self):
    #     """
    #     Communicates with other process. When other process hits barrier, grabs the most recent frame (caught by
    #     update thread) and puts it in the queue.
    #     Returns
    #     -------
    #     None
    #     """
    #     # Spawn update thread
    #     updateThread = threading.Thread(target=self.update, daemon=True)
    #     updateThread.start()
    #
    #     self.camStatus[1] = 1  # Indicate that camera is ready.
    #     # Wait for both cameras to be ready.
    #     while np.sum(self.camStatus) < len(self.camStatus):
    #         continue
    #
    #     while self.running.value:
    #         newFrame = self.frame
    #         # ts = datetime.now()
    #         self.queue.put([newFrame.color, self.timestamp])
    #         self.barrier.wait()
    #     #  Wait on update thread to stop grabbing frames.
    #     updateThread.join()

    def initCam(self):
        pass

    def read(self):
        """
        Read function. Problem is that the kinect has an internal buffer and so there will always be a slight time
        offset between frames. But not too annoying for stereocalibration.
        Returns
        -------
        None
        """
        kinect = k4a.PyK4A(k4a.Config(
            color_resolution=k4a.ColorResolution.RES_720P,
            depth_mode=k4a.DepthMode.NFOV_UNBINNED,
            camera_fps=k4a.FPS.FPS_30,
            synchronized_images_only=True, ))
        kinect.start()
        self.kinect = kinect
        oldFrame = self.kinect.get_capture()
        self.camStatus[1] = 1  # Indicate that camera is ready.
        # Wait for both cameras to be ready.
        while np.sum(self.camStatus) < len(self.camStatus):
            continue

        while self.running.value:
            newFrame = kinect.get_capture()
            ts = datetime.now()
            self.queue.put([newFrame.color, ts])
            self.barrier.wait()

        # Have to make sure that the queue is empty for terminating process
        while not self.queue.empty():
            self.queue.get_nowait()
        kinect.stop()


def main():
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H-%M")
    saveDirectory = os.path.join("Results", dt_string)
    kinectCalib = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\kinect Calib\kinectCalib.json"
    flirCalib = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\FLIRCalibration\flirCalib.json"
    calibFiles = [flirCalib, kinectCalib]
    logger = Logger(os.path.join(saveDirectory, "logger.log"), "Stereo")
    stereoCalibrator = StereoCalibrator(saveDirectory, ["flir", "kinect"], calibFiles, logger)
    stereoCalibrator.performStereocalibration()


if __name__ == "__main__":
    main()
