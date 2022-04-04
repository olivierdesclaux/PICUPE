import numpy as np
import cv2.cv2 as cv2
import threading

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


class Rectifier:
    def __init__(self, calibFiles):
        self.calibFiles = calibFiles
        with open(self.calibFiles) as f:
            d = json.load(f)
        self.leftMatrix = np.array(d["Matrix1"])
        self.leftDist = np.array(d["Dist1"])
        self.rightDist = np.array(d["Dist2"])
        self.rightMatrix = np.array(d["Matrix2"])
        self.R = np.array(d["R"])
        self.T = np.array(d["T"])
        self.frameSize = np.array(d["frameSize"])
        self.leftQueue = multiprocessing.Queue()
        self.rightQueue = multiprocessing.Queue()
        self.running = multiprocessing.Value("i", 1)
        self.flirPort = findCameraPort("flir")
        if self.flirPort == -1:
            raise Exception("No Flir")
        self.frames = None
        # Initialising processes and threads for acquiring and processing data
        self.leftCamProcess = None  # Sub-process connected to left Cam to acquire data and put in self.leftQueue
        self.rightCamProcess = None  # Sub-process connected to right Cam to acquire data and put in self.rightQueue
        self.updateThread = None  # Thread in main process: Reads data in both queues and stores it in self.frames

    def main(self):
        # STEREO RECTIFICATION
        RLeft, RRight, PLeft, PRight, Q, roiLeft, roiRight = cv2.stereoRectify(
            self.leftMatrix, self.leftDist, self.rightMatrix, self.rightDist, self.frameSize,
            self.R, self.T, alpha=0, flags=cv2.CALIB_ZERO_DISPARITY)

        print(self.frameSize)
        print(roiLeft)
        print(roiRight)
        y_l, x_l, h_l, w_l = roiLeft
        y_r, x_r, h_r, w_r = roiRight
        stereo = cv2.StereoSGBM_create(
            numDisparities=64,
            blockSize=15,
            minDisparity=0
        )

        leftMap1, leftMap2 = cv2.initUndistortRectifyMap(self.leftMatrix, self.leftDist, RLeft, PLeft, (1024, 768),
                                                         cv2.CV_32FC1)
        rightMap1, rightMap2 = cv2.initUndistortRectifyMap(self.rightMatrix, self.rightDist, RRight, PRight, (1024,
                                                                                                              768),
                                                           cv2.CV_32FC1)
        self.start()  # Start processes and threads
        while self.running.value:
            if self.frames is not None:
                [leftFrame, rightFrame] = self.frames
                rightFrame = rightFrame[:, :, :3]
                leftFrameRectified = cv2.remap(leftFrame, leftMap1, leftMap2, interpolation=cv2.INTER_LINEAR)
                rightFrameRectified = cv2.remap(rightFrame, rightMap1, rightMap2, interpolation=cv2.INTER_LINEAR)
                disparity = stereo.compute(leftFrameRectified, rightFrameRectified)#.astype(np.float32) / 16.0
                disparity = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                          dtype=cv2.CV_32F)
                # points3D = cv2.reprojectImageTo3D(disparity, Q)
                # points3D = cv2.normalize(points3D, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                #                          dtype=cv2.CV_8UC3)

                cv2.imshow("Disparity Map", cv2.resize(disparity, dsize=None, fx=0.4, fy=0.4))
                # cv2.imshow("Disparity Map", disparity)
                cv2.imshow("Rectified Left", cv2.resize(leftFrameRectified, dsize=None, fx=0.4, fy=0.4))
                cv2.imshow("Rectified Right", cv2.resize(rightFrameRectified, dsize=None, fx=0.4, fy=0.4))

                # rightFrameRectified[x_l:x_l+w_l, y_l:y_l + h_l]
                # overlay = leftFrameRectified[x_l:x_l+w_l, y_l:y_l + h_l]
                added_image = cv2.addWeighted(rightFrameRectified, 0.4, leftFrameRectified, 0.1, 0)

                cv2.imshow("Cropped left", added_image)
                if cv2.waitKey(10) == ord("q"):
                    break
        self.stop()

        detector = CircleDetector()
        rightFrameRectified = cv2.subtract(rightFrameRectified[:, :, 2], rightFrameRectified[:, :, 0])
        leftFrameRectified = cv2.cvtColor(leftFrameRectified, cv2.COLOR_BGR2GRAY)

        retL, leftPos = cv2.findCirclesGrid(leftFrameRectified, (9, 15), flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                   blobDetector= detector.get())
        retR, rightPos = cv2.findCirclesGrid(rightFrameRectified, (9, 15), flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                   blobDetector= detector.get())

        # print(leftPos)
        # print(rightPos)
        print(np.max(disparity))
        print(np.min(disparity))
        # if retL and retR:
        #     np.save("blockmatching/left.npy", leftFrameRectified, allow_pickle=True)
        #     np.save("blockmatching/right.npy", rightFrameRectified, allow_pickle=True)
        # leftPos = np.array(leftPos)
        # rightPos = np.array(rightPos)
        # print(leftPos.shape)
        # print(rightPos.shape)

    def update(self):
        while self.running.value:
            try:
                leftFrame, leftTs = self.leftQueue.get_nowait()
                rightFrame, rightTs = self.rightQueue.get_nowait()
                self.frames = [leftFrame, rightFrame]
            except:
                continue

    def start(self):
        print("Starting")
        barrier = multiprocessing.Barrier(2)
        self.updateThread = threading.Thread(target=self.update, args=(), daemon=True)
        self.updateThread.start()

        # flirPort = findCameraPort("flir")
        leftCam = flirReader(self.flirPort, self.leftQueue, self.running, barrier)
        self.leftCamProcess = multiprocessing.Process(target=leftCam.read, args=(), daemon=True)
        self.leftCamProcess.start()

        rightCam = kinectReader(self.rightQueue, self.running, barrier)
        self.rightCamProcess = multiprocessing.Process(target=rightCam.read, args=(), daemon=True)
        self.rightCamProcess.start()

    def stop(self, message=None):
        with self.running.get_lock():
            self.running.value = 0

        # Wait for all threads to finish, and force stopping of processes.
        if self.updateThread is not None:
            self.updateThread.join()
        if self.leftCamProcess is not None:
            self.leftCamProcess.terminate()
        if self.rightCamProcess is not None:
            self.rightCamProcess.terminate()

        # Reset all threads and processes
        self.updateThread = None
        self.leftCamProcess = None
        self.rightCamProcess = None

        cv2.destroyAllWindows()
        if message is not None:
            raise Exception(message)
        print("Stopped")

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
    stereoJson = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Calibration Directories\2022-02-08_17-37\calib\stereo.json"
    rect = Rectifier(stereoJson)
    rect.main()


if __name__ == "__main__":
    main()

