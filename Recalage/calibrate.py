import multiprocessing
import threading
import sys
import numpy as np
import cv2.cv2 as cv2
import time
import os
import pyk4a as k4a

sys.path.append("../")
# Local modules
from utils.readerWriterClass import findCameraPort
from Recalage.circledetector import CircleDetector
from Recalage.calibrationhandler import CalibrationHandler
from utils.process import Process
from utils.myLogger import Logger

class Calibrator:
    def __init__(self, savePath, cameraName, flags, initialGuess, logger, initialGrids=20, minGrids=15):
        # Initialisation
        self.savePath = savePath
        self.name = cameraName
        self.fileName = os.path.join(self.savePath, self.name + "Calib.json")
        self.modality, self.frameSize = self.getModality(cameraName)
        self.flags = flags  # Opencv calibration flags

        self.logger = logger
        self.queue = multiprocessing.Queue()  # for putting frames read from the camera
        self.frame = None  # currently studied frame

        self.saveFrames = True
        self.savedFrames = []  # optional, if you want to save frames. Then saveFrames must be set to true.
        self.initialGuess = initialGuess  # For calibration parameters initialisation
        # Hyperparameters
        self.initialGrids = initialGrids
        self.minGrids = minGrids
        self.allGridsFound = False  # False until we have found at least initialGrids

        self.maxPointError = 1.0  # Threshold on the retroprojection error
        self.secondsToSkip = 2.0  # Once a grid has been found, time to wait before searching again.
        self.rows = 9  # number of rows of the calibration object
        self.cols = 15  # number of columns of the calibration object
        self.boardSize = (self.rows, self.cols)

        self.circleDetector = CircleDetector()
        self.running = multiprocessing.Value('i', 1)

        # Initialising matrices needed to perform calibration
        self.positions3D = []  # Will be filled iteratively with self.objectCorners
        self.positions2D = []  # Center positions (in px) of the calib object

        # Initialising processes and threads for acquiring and processing data
        self.camProcess = None  # Sub-process connected to camera to acquire data and put in self.leftQueue
        self.rightCamProcess = None  # Sub-process connected to right Cam to acquire data and put in self.rightQueue
        self.updateThread = None  # Thread in main process: Reads data in the queue and stores it in self.frames
        self.circleFinderThread = None  # Thread in main process: Catches self.frame and looks for circles

        # Physical distance between calibration object centers
        # X axis (columns) increments 0, 1, 2, ... numberOfColumns-1,
        # repeating numRows times
        # Y axis (rows) increments 0, 1, 2, ... numberOfRows-1
        # 21.2 mm is distance between circles in current grid
        self.objectCorners = 21.2 * np.array([[2 * (x % self.rows) + np.floor(x / self.rows) % 2,
                                               np.floor(x / self.rows), 0] for x in range(self.rows * self.cols)],
                                             np.float32)

        # If user decides to quit (pressing q on keyboard)
        self.aborted = False

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
            frameSize = (1280, 720)
            modality = "rgb"
        else:
            raise Exception("Wrong Name")
        return modality, frameSize

    def performCalibration(self):
        """
        Main function for performing calibration. Saves results of calibration to a calib.json file. Looking for the
        calibration object centers is done in the background. Here, we mainly take care of displaying and computing
        calibration matrices.
        Returns
        -------
        None
        """
        while self.running.value:  # Continue until calibration succeeds
            self.start()
            while not self.allGridsFound:
                if self.frame is not None:
                    frame = self.frame.copy()
                    self.drawCircles(frame)
                    self.drawOutlines(frame)
                    cv2.putText(frame, 'Remaining images: ' + str(self.initialGrids - self.len()), (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)

                    cv2.imshow('Camera Calibration', frame)
                    if cv2.waitKey(10) == ord('q'):
                        self.aborted = True
                        self.stop()
                else:
                    cv2.waitKey(10)

            self.stop()

            if self.aborted:
                break

            # Creates CalibrationHandler object to handle cv2.calibrateCamera() and error-checking
            calibration = CalibrationHandler(self.positions3D, self.positions2D, self.frameSize, self.minGrids,
                                             self.maxPointError, self.flags, self.initialGuess, self.logger)

            if calibration.calibrate():
                # Tries to save matrices to directory, can fail if
                # wrong directory/file in use
                try:
                    calibration.writeToFile(self.savePath, self.name + "Calib.json")
                    # np.save(os.path.join(self.savePath, "positions3D.npy"), self.positions3D)
                    # np.save(os.path.join(self.savePath, "positions2D.npy"), self.positions2D)

                except:
                    self.stop("Failed to write matrices to file.")
                # Break out of grid-finding loop
                # Display matplotlib graphics of errors and saves to .png
                calibration.displayError(self.savePath)
                break
            else:
                self.positions3D = calibration.positions3D
                self.positions2D = calibration.positions2D
                self.logger.log("Calibration failed, needs more images.")
                with self.running.get_lock():
                    self.running.value = 1
        self.stop()

        if not self.aborted:
            self.logger.log("Calibration successful.")
            self.logger.log("Used {} images".format(len(calibration.positions3D)))
            return self.fileName
        else:
            self.logger.log("Calibration aborted")
            return None

    def update(self):
        """
        Thread that continuously grabs frame (threaded) from the camera.
        Returns
        -------

        """
        while self.running.value:
            try:
                frame = self.queue.get_nowait()
                self.frame = frame
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
        self.circleFinderThread = threading.Thread(target=self.findCircles, args=(), daemon=True)
        self.circleFinderThread.start()

    def start(self):
        """
        Initialises all processes/threads for acquisition
        Returns
        -------
        None
        """
        camReader = cameraReader(self.name, self.running, self.queue)

        self.camProcess = Process("Camera Process", self.logger, camReader, target=camReader.read, args=(), daemon=True)
        self.camProcess.start()
        self.startCircleFinder()
        self.updateThread = threading.Thread(target=self.update, args=(), daemon=True)
        self.updateThread.start()

        return True

    def findCircles(self):
        """
        Threaded function that applies a CircleDetector object on self.frame. If the calibration object is
        detected self.positions3D and self.positions2D are updated.
        Returns
        -------
        None
        """
        while self.running.value and self.len() < self.initialGrids:
            if self.frame is not None:
                frame = self.frame.copy()
                grayFrame = self.grayify(frame)
                ret, pos = cv2.findCirclesGrid(grayFrame, self.boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                               blobDetector=self.circleDetector.get())
                if not ret:
                    cv2.waitKey(10)
                    continue
                else:
                    self.positions3D.append(self.objectCorners)
                    self.positions2D.append(pos)
                    time.sleep(self.secondsToSkip)
                    if self.saveFrames:
                        self.savedFrames.append(frame)
            else:
                cv2.waitKey(10)
        self.allGridsFound = True

        return True

    def drawCircles(self, frame):
        """
        Search for and display circles in given frames
        """
        # Detects circles in gray version of frame
        grayFrame = self.grayify(frame)

        # TODO: Ici il y a un pb. On devrait réutiliser les positions trouvées par find circles grid. Mais on utilise
        #  une autre fonction. Pourquoi? De mémoire, c'est un problème de shape uniquement. Mais j'ai eu la flemme de
        #  chercher correctement.
        keypoints = self.circleDetector.get().detect(grayFrame)
        # Draws circles in place on frames
        frame = cv2.drawKeypoints(frame, keypoints, frame, (255, 255, 255),
                                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
                                  + cv2.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)
        # Displays special color graphics for latest grid found
        if self.positions2D:
            cv2.drawChessboardCorners(frame, self.boardSize, self.positions2D[-1], True)

    def drawOutlines(self, frame):
        """
        Draws calibration object outlines on corresponding frame. Will just draw the polygon passing by the 4 most
        outside centers.
        Drawing is made inplace.
        Parameters
        ----------
        frame: np.array, frame to draw on.
        Returns
        -------
        None
        """

        for pos in self.positions2D:
            # Uses corners of each calibration grid as points for lines
            points = np.array([pos[0], pos[self.rows - 1],
                               pos[self.cols * self.rows - 1],
                               pos[self.rows * (self.cols - 1)]], np.int32)
            # Draws lines in place on arguments
            cv2.polylines(frame, [points], True, (20, 220, 90), 2)

    def grayify(self, frame):
        """
        Converts frames to grayscale based on modality. Actually, for the RGB, we only want to subtract the red
        channel from the blue channel. This will make the red color of the checkerboard stand out really well,
        instead of a standard grayscale.
        Parameters
        ----------
        frame: np.array, frame to convert

        Returns
        -------
        grayified: np.array, grayscale image
        """
        if self.modality == "rgb":
            grayified = cv2.subtract(frame[:, :, 2], frame[:, :, 0])
        elif self.modality == "ir":
            grayified = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            raise Exception("Wrong modality")

        return grayified

    def len(self):
        return len(self.positions3D)

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
        # Set global running value to 0 (stops recording)
        with self.running.get_lock():
            self.running.value = 0

        # Empty the queue by force
        while not self.queue.empty():
            try:
                self.queue.get_nowait()
            except:
                break

        # Wait for threads to join and force process to terminate (avoids possible deadlocks)
        if self.circleFinderThread is not None:
            self.circleFinderThread.join()
        if self.updateThread is not None:
            self.updateThread.join()
        if self.camProcess is not None:
            self.camProcess.terminate()
            # self.camProcess.join()

        # Reset threads and processes
        self.circleFinderThread = None
        self.updateThread = None
        self.camProcess = None

        cv2.destroyAllWindows()
        if message is not None:
            raise Exception(message)

    def saveImages(self):
        """
        For saving images and doing calibration in postprocess
        Returns
        -------
        None
        """
        while self.running.value:  # Continue until calibration succeeds
            self.start()
            while not self.allGridsFound:
                if self.frame is not None:
                    frame = self.frame.copy()
                    self.drawCircles(frame)
                    self.drawOutlines(frame)
                    cv2.putText(frame, 'Remaining images: ' + str(self.initialGrids - self.len()), (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)

                    cv2.imshow('Camera Calibration', frame)
                    if cv2.waitKey(10) == ord('q'):
                        self.aborted = True
                        self.stop()
                else:
                    cv2.waitKey(10)

            self.stop()

        np.save(os.path.join(self.savePath, "frames.npy"), self.savedFrames, allow_pickle=True)

class cameraReader:
    def __init__(self, name, running, queue):
        self.name = name
        self.running = running
        self.queue = queue
        self.cam = None

    def read(self):
        """
        Reads frames and stores them in the queue
        Returns
        -------

        """
        try:
            if self.name == "kinect":
                cam = k4a.PyK4A(k4a.Config(
                    color_resolution=k4a.ColorResolution.RES_720P,
                    depth_mode=k4a.DepthMode.NFOV_UNBINNED,
                    camera_fps=k4a.FPS.FPS_30,
                    synchronized_images_only=True, ))
                cam.start()
            else:
                camPort = findCameraPort(self.name)
                if camPort == -1:
                    raise Exception("Couldn't find {}".format(self.name))
                cam = cv2.VideoCapture(camPort, cv2.CAP_DSHOW)
                cam.set(cv2.CAP_PROP_BUFFERSIZE, 0)
                cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
                cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
                cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        except:
            raise Exception("Failed to open {}".format(self.name))

        self.cam = cam
        while self.running.value:
            if self.name == "kinect":
                frame = self.cam.get_capture()
                self.queue.put(frame.color)
            else:
                ret, frame = self.cam.read()
                if ret:
                    self.queue.put(frame)
            cv2.waitKey(5)
        print("Releasing cam...")
        if self.name == "kinect":
            self.cam.stop()
        else:
            self.cam.release()
        # Have to make sure that the queue is empty for terminating process
        while not self.queue.empty():
            self.queue.get_nowait()
        print("Stopped cam")
        self.cam = None

    def close(self):
        """
        Function just so that cameraReader can be processed by our custom Process function (cf. utils/process.py).
        Checks that cam is closed
        Returns
        -------
        True
        """
        if self.cam is None:
            return True
        elif self.name == "kinect":
            self.cam.stop()
        elif self.name == "flir":
            self.cam.release()
        else:
            return False
        return True

def main():
    savePath = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Calibration Directories\test"

    logger = Logger(savePath, "customLog.log")
    listener = threading.Thread(target=logger.logWriter, daemon=True)
    listener.start()

    flags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_ASPECT_RATIO
    initialGuess = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\kinectInitialGuess.json"

    calibrator = Calibrator(savePath, "kinect", flags, initialGuess, logger=logger, initialGrids=40, minGrids=30)
    calibrator.performCalibration()


if __name__ == "__main__":
    main()
