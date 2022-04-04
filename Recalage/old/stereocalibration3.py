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

sys.path.append("../../")
from Recalage.cameraUtils import openCalibrationFile, scaleForHconcat, NumpyEncoder
from Recalage.circledetector import CircleDetector
from utils.readerWriterClass import findCameraPort
from Recalage.calibrate import Calibrator
from utils.myLogger import Logger


class StereoCalibrator:
    def __init__(self, savePath, names, calibFiles, initialGrids=15, minGrids=10):
        self.savePath = savePath

        # Global variables
        self.allGridsFound = False
        self.running = multiprocessing.Value('i', 1)

        # Setting up left and right attributes
        self.leftName = names[0]
        self.rightName = names[1]
        self.leftQueue = multiprocessing.Queue()
        self.rightQueue = multiprocessing.Queue()
        self.leftModality, self.leftFrameSize = self.getModality(self.leftName)
        self.rightModality, self.rightFrameSize = self.getModality(self.rightName)
        # self.leftFrame = None
        # self.rightFrame = None
        self.frames = None
        self.leftCalibFile = calibFiles[0]
        self.rightCalibFile = calibFiles[1]
        self.leftMatrix, self.leftDist = openCalibrationFile(self.leftCalibFile)
        self.rightMatrix, self.rightDist = openCalibrationFile(self.rightCalibFile)
        # If matrices return empty, quit as well
        if not self.leftDist.any() or not self.rightDist.any():
            sys.exit("Cannot read matrices from calibration files. Format may be incorrect.")

        # Hyperparameters
        self.flags = cv2.CALIB_FIX_INTRINSIC
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
        self.initialGrids = initialGrids
        self.minGrids = minGrids
        self.secondsToSkip = 2.0
        self.rows = 9
        self.cols = 15
        self.boardSize = (self.rows, self.cols)
        self.circleDetector = CircleDetector()
        self.objectCorners = 21.2 * np.array([[2 * (x % self.rows) + np.floor(x / self.rows) % 2,
                                               np.floor(x / self.rows), 0] for x in range(self.rows * self.cols)],
                                             np.float32)

        # Initialising matrices needed to perform stereocalibration
        self.positions3D = []
        self.leftPositions2D = []
        self.rightPositions2D = []
        self.leftFrames = []
        self.rightFrames = []
        # Initialising outputs
        self.retError = None
        self.R = None
        self.T = None
        self.E = None
        self.F = None

        self.tsDeltas = []

    @staticmethod
    def getModality(name):
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
        while self.running.value:
            barrier = multiprocessing.Barrier(2)
            # while self.running.value:  # Continue until calibration succeeds
            self.startCircleFinder()
            threading.Thread(target=self.update, args=(), daemon=True).start()

            flirPort = findCameraPort("flir")
            leftCam = flirReader(flirPort, self.leftQueue, self.running, barrier)
            leftCamProcess = multiprocessing.Process(target=leftCam.read, args=(), daemon=True)
            leftCamProcess.start()

            rightCam = kinectReader(self.rightQueue, self.running, barrier)
            rightCamProcess = multiprocessing.Process(target=rightCam.read, args=(), daemon=True)
            rightCamProcess.start()

            while not self.allGridsFound:
                # if leftFrameOriginal is not None and rightFrameOriginal is not None:
                if self.frames is not None:
                    [leftFrameOriginal, rightFrameOriginal] = self.frames
                    rightFrameOriginal = rightFrameOriginal[:, :, :3]
                    leftFrame = leftFrameOriginal.copy()
                    rightFrame = rightFrameOriginal.copy()
                    self.drawOutlines(leftFrame, self.leftPositions2D)
                    self.drawOutlines(rightFrame, self.rightPositions2D)
                    resizedFrameLeft, resizedFrameRight = scaleForHconcat(leftFrame, rightFrame, 0.6)
                    concatFrame = cv2.hconcat([resizedFrameLeft, resizedFrameRight])
                    cv2.putText(concatFrame, 'Remaining images: ' + str(self.initialGrids - self.len()), (10, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
                    cv2.imshow('Stereo Calibration', concatFrame)
                    if cv2.waitKey(10) == ord('q'):
                        with self.running.get_lock():
                            self.running.value = 0
                        return

                else:
                    cv2.waitKey(10)
                # if self.leftFrame is not None and self.rightFrame is not None:

            self.stop()
            leftCamProcess.join()
            rightCamProcess.join()

            print("Computing stereocalib")
            self.computeMatrices()

            # Calibration successful:
            if self.retError is None:
                raise Exception("Stereo calibration failed")
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

    def drawOutlines(self, frame, positions):
        """Draws outlines on corresponding frame
        """
        if positions:
            for pos in positions:
                # Uses corners of each calibration grid as points for lines
                points = np.array([pos[0], pos[self.rows - 1],
                                   pos[self.cols * self.rows - 1],
                                   pos[self.rows * (self.cols - 1)]], np.int32)
                # Draws lines in place on arguments
                cv2.polylines(frame, [points], True, (20, 220, 90), 2)

    def update(self):
        while self.running.value:
            try:
                leftFrame, leftTs = self.leftQueue.get_nowait()
                rightFrame, rightTs = self.rightQueue.get_nowait()
                # self.leftFrame = leftFrame
                # self.rightFrame = rightFrame
                self.frames = [leftFrame, rightFrame]
                self.tsDeltas.append(abs((leftTs - rightTs).total_seconds()))
            except:
                continue

    def startCircleFinder(self):
        with self.running.get_lock():
            self.running.value = 1
        self.allGridsFound = False
        threading.Thread(target=self.findCircles, args=(), daemon=True).start()

    def findCircles(self):
        self.positions3D = []
        self.leftPositions2D = []
        self.rightPositions2D = []
        while self.running.value and self.len() < self.initialGrids:
            if self.frames is not None:
                [leftFrameOriginal, rightFrameOriginal] = self.frames
                rightFrameOriginal = rightFrameOriginal[:, :, :3]
                leftGrayFrame = self.grayify(leftFrameOriginal, "left")
                rightGrayFrame = self.grayify(rightFrameOriginal, "right")
                frames = [leftGrayFrame, rightGrayFrame]
                positions = []
                for frame in frames:
                    ret, pos = cv2.findCirclesGrid(frame, self.boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                   blobDetector=self.circleDetector.get())
                    if not ret:
                        cv2.waitKey(10)
                        continue
                    else:
                        positions.append(pos)
                if len(positions) == 2:
                    self.positions3D.append(self.objectCorners)
                    self.leftPositions2D.append(positions[0])
                    self.rightPositions2D.append(positions[1])
                    self.leftFrames.append(leftFrameOriginal)
                    self.rightFrames.append(rightFrameOriginal)
                    time.sleep(self.secondsToSkip)
            else:
                cv2.waitKey(10)

        leftFilePath = os.path.join(self.savePath, "leftSet.npy")
        rightFilePath = os.path.join(self.savePath, "rightSet.npy")
        np.save(leftFilePath, self.leftFrames, allow_pickle=True)
        np.save(rightFilePath, self.rightFrames, allow_pickle=True)
        np.save(os.path.join(self.savePath, "positions3D.npy"), self.positions3D, allow_pickle=True)
        self.allGridsFound = True
        return True

    def grayify(self, frame, side):
        # Turns frames gray based on frame type
        # Actually, for the RGB, we only want to subtract the red channel from the blue channel.
        # This will make the red color of the checkerboard stand out really well
        if side == "left":
            modality = self.leftModality
        elif side == "right":
            modality = self.rightModality
        else:
            raise Exception("Wrong camera side")

        if modality == "rgb":
            return cv2.subtract(frame[:, :, 2], frame[:, :, 0])
        elif modality == "ir":
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    def len(self):
        return len(self.positions3D)

    def stop(self, message=None):
        with self.running.get_lock():
            self.running.value = 0
        cv2.destroyAllWindows()
        if message is not None:
            raise Exception(message)

    def computeMatrices(self):
        if len(self.positions3D) > self.minGrids:
            (self.retError, self.leftMatrix, self.leftDist,
             self.rightMatrix, self.rightDist, self.R,
             self.T, self.E, self.F) = cv2.stereoCalibrate(
                self.positions3D, self.leftPositions2D,
                self.rightPositions2D, self.leftMatrix,
                self.leftDist, self.rightMatrix, self.rightDist,
                self.leftFrameSize, flags=self.flags, criteria=self.criteria)
            stereocalibrationDone = self.computeErrors()
        else:
            stereocalibrationDone = False

        return stereocalibrationDone

    def computeErrors(self):
        for left2D, right2D, pos3D in zip(self.leftPositions2D, self.rightPositions2D, self.positions3D):
            # leftReprojected = cv2.projectPoints()
            pass
        return True


class flirReader:
    def __init__(self, port, queue, running, barrier):
        self.port = port
        self.queue = queue
        self.running = running
        self.barrier = barrier
        self.cam = None

    def read(self):
        self.initCam()
        ret, oldFrame = self.cam.read()
        self.barrier.wait()
        while self.running.value:
            self.cam.grab()
            ts = datetime.now()
            ret, newFrame = self.cam.retrieve()
            if not ret:
                break
            else:
                self.queue.put([newFrame, ts])
            self.barrier.wait()
        print("Releasing flir...")
        self.cam.release()
        # Have to make sure that the queue is empty for terminating process
        while not self.queue.empty():
            self.queue.get_nowait()

    def initCam(self):
        cam = cv2.VideoCapture(self.port, cv2.CAP_DSHOW)
        cam.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cam = cam


class kinectReader:
    def __init__(self, queue, running, barrier):
        self.queue = queue
        self.running = running
        self.barrier = barrier
        self.kinect = None

    def read(self):
        kinect = k4a.PyK4A(k4a.Config(
            color_resolution=k4a.ColorResolution.RES_720P,
            depth_mode=k4a.DepthMode.NFOV_UNBINNED,
            camera_fps=k4a.FPS.FPS_30,
            synchronized_images_only=True, ))
        kinect.start()
        self.kinect = kinect
        oldFrame = self.kinect.get_capture()
        # self.queue.put(oldFrame)
        self.barrier.wait()

        while self.running.value:
            newFrame = kinect.get_capture()
            ts = datetime.now()
            self.queue.put([newFrame.color, ts])
            self.barrier.wait()

        print("Releasing kinect...")
        # Have to make sure that the queue is empty for terminating process
        while not self.queue.empty():
            self.queue.get_nowait()
        kinect.stop()

    def initCam(self):
        pass
        # return kinect


def main():
    saveDirectory = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\Numeric Stability"

    calibFlags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_ASPECT_RATIO
    initialGuess = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\FLIRCalibration\flirCalib.json"
    kinectCalib = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\kinect Calib\kinectCalib.json"

    for D in ["A", "B", "C"]:
    # for D in ["C"]:
        for d in ["07", "10", "15"]:
        # for d in ["15"]:
            saveDir = os.path.join(saveDirectory, D + d)
            os.mkdir(saveDir)

            logger = Logger(saveDir, "calib.log")
            listener = threading.Thread(target=logger.logWriter, daemon=True)
            listener.start()

            calibrator = Calibrator(saveDir, "flir", calibFlags, initialGuess=initialGuess, logger=logger,
                                    initialGrids=35,
                                    minGrids=1)
            calibrator.saveImages()
            # flirCalib = os.path.join(saveDir, "flirCalib.json")
            calibFiles = [initialGuess, kinectCalib]

            stereoCalibrator = StereoCalibrator(saveDir, ["flir", "kinect"], calibFiles, initialGrids=35, minGrids=1)
            stereoCalibrator.performStereocalibration()


if __name__ == "__main__":
    main()
