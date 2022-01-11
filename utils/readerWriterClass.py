from datetime import datetime
import os
import time
import sys
import cv2.cv2 as cv2
import numpy as np
import pyk4a as k4a
from utils import extractDepth
import numba as nb
import MTwFunctions as mtw

sys.path.append('../sandbox')


########################################################################################################################
############################################### WEBCAM #################################################################
########################################################################################################################

class Webcam:
    def __init__(self, portNum, webcamQueue, keepGoing, webcamQueueSize, cameraStatus, logger, savePath):
        self.portNum = portNum
        self.webcamQueue = webcamQueue
        self.keepGoing = keepGoing
        self.webcamQueueSize = webcamQueueSize
        self.cameraStatus = cameraStatus
        self.logger = logger
        self.savePath = savePath
        self.cam = None

    def read(self):
        cam = cv2.VideoCapture(self.portNum)
        self.cam = cam
        self.cameraStatus[0] = 1

        while np.sum(self.cameraStatus) < len(self.cameraStatus):
            continue

        # print("Webcam is Recording...")
        self.logger.log("Webcam is Recording...")
        t1 = time.time()
        timestamp = datetime.now()
        ret, oldFrame = cam.read()
        self.webcamQueue.put([oldFrame, timestamp])
        with self.webcamQueueSize.get_lock():
            self.webcamQueueSize.value += 1
        cnt = 1
        while self.keepGoing.value:
            timestamp = datetime.now()
            ret, newFrame = cam.read()  # grab new frame
            if not ret:
                break
            if are_identical(oldFrame, newFrame):
                continue
            else:
                self.webcamQueue.put([newFrame, timestamp])
                cnt += 1
                with self.webcamQueueSize.get_lock():
                    self.webcamQueueSize.value += 1
            oldFrame = newFrame
        t = time.time() - t1

        self.cam.release()
        cv2.destroyAllWindows()
        self.logger.log("Finished Webcam acquisition. Put {} frames in the queue".format(cnt))
        self.logger.log("Webcam acquisition lasted {}s".format(round(t, 3)))
        self.logger.log("Overall Webcam FPS: {}".format(round(cnt / t, 3)))

    def close(self):
        self.logger.log("Closing Webcam...")
        if self.cam:
            self.cam.release()
            cv2.destroyAllWindows()

    def write(self):
        cnt = 0  # Initialise counter. Images will have name 00000, 00001, 00002
        textFile = open(os.path.join(self.savePath, "timestamps.txt"), "a")
        while True:
            try:
                [frame, timestamp] = self.webcamQueue.get_nowait()
                name = str(cnt).zfill(5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
                with open(os.path.join(self.savePath, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                textFile.write("{}\n".format(timestamp))  # Add timestamp to the .txt file
                cnt += 1
                with self.webcamQueueSize.get_lock():  # Have to access lock otherwise it could miss the increment by the reader
                    self.webcamQueueSize.value -= 1  # Decrement the queue size

            except Exception as e:
                if self.keepGoing.value or self.webcamQueueSize.value > 0:
                    continue
                else:
                    break
        textFile.close()
        self.logger.log("Finished Webcam writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### KINECT #################################################################
########################################################################################################################

class Kinect:
    def __init__(self, kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize, kinectDepthQueueSize,
                 cameraStatus, logger, savePathRGB, savePathDepth):
        self.kinectRGBQueue = kinectRGBQueue
        self.kinectDepthQueue = kinectDepthQueue
        self.keepGoing = keepGoing
        self.kinectRGBQueueSize = kinectRGBQueueSize
        self.kinectDepthQueueSize = kinectDepthQueueSize
        self.cameraStatus = cameraStatus
        self.logger = logger
        self.savePathRGB = savePathRGB
        self.savePathDepth = savePathDepth
        self.kinect = None

    def read(self):
        kinect = k4a.PyK4A(k4a.Config(
            color_resolution=k4a.ColorResolution.RES_720P,
            depth_mode=k4a.DepthMode.NFOV_UNBINNED,
            camera_fps=k4a.FPS.FPS_30,
            synchronized_images_only=True, ))
        kinect.start()
        self.kinect = kinect
        self.cameraStatus[1] = 1

        while np.sum(self.cameraStatus) < len(self.cameraStatus):
            continue
        self.logger.log("Kinect is Recording...")
        t1 = time.time()
        timestamp = datetime.now()
        oldFrame = kinect.get_capture()
        self.kinectRGBQueue.put([oldFrame.color, timestamp])
        self.kinectDepthQueue.put([oldFrame.transformed_depth, timestamp])
        with self.kinectDepthQueueSize.get_lock():
            self.kinectDepthQueueSize.value += 1
        with self.kinectRGBQueueSize.get_lock():
            self.kinectRGBQueueSize.value += 1
        cnt = 1  # Counter to compute the FPS (verbosity)
        while self.keepGoing.value:
            timestamp = datetime.now()
            newFrame = kinect.get_capture()  # grab new frame
            if are_identical(oldFrame.depth, newFrame.depth):  # check if new depth is identical to the previous depth
                continue
            else:  # if not, we put it in the queue for writing
                self.kinectRGBQueue.put([newFrame.color, timestamp])
                self.kinectDepthQueue.put([newFrame.transformed_depth, timestamp])
                cnt += 1
                with self.kinectDepthQueueSize.get_lock():
                    self.kinectDepthQueueSize.value += 1
                with self.kinectRGBQueueSize.get_lock():
                    self.kinectRGBQueueSize.value += 1
            oldFrame = newFrame
        t = time.time() - t1
        # kinect.stop()

        self.logger.log("Finished Kinect acquisition. Put {} frames in the queue".format(cnt))
        self.logger.log("Kinect acquisition lasted {}s".format(round(t, 3)))
        self.logger.log("Overall Kinect FPS: {}".format(round(cnt / t, 3)))

    def writeRGB(self):
        cnt = 0  # Initialise counter. Images will have name 00000, 00001, 00002
        textFile = open(os.path.join(self.savePathRGB, "timestamps.txt"), "a")
        while True:
            try:
                [frame, timestamp] = self.kinectRGBQueue.get_nowait()
                name = str(cnt).zfill(5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
                # frame = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2RGB)
                with open(os.path.join(self.savePathRGB, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                textFile.write("{}\n".format(timestamp))  # Add timestamp to the .txt file
                cnt += 1
                with self.kinectRGBQueueSize.get_lock():  # Have to access lock otherwise it could miss the increment by the
                    # reader
                    self.kinectRGBQueueSize.value -= 1  # Decrement the queue size

            except Exception as e:
                if self.keepGoing.value or self.kinectRGBQueueSize.value > 0:
                    continue
                else:
                    break
        textFile.close()
        self.logger.log("Finished RGB writing. Grabbed {} frames in the queue".format(cnt))

    def writeDepth(self):
        cnt = 0
        textFile = open(os.path.join(self.savePathDepth, "timestamps.txt"), "a")
        while True:
            try:
                [frame, timestamp] = self.kinectDepthQueue.get_nowait()
                name = str(cnt).zfill(5)
                frame = extractDepth.colorize(frame, (None, 5000), cv2.COLORMAP_BONE)
                with open(os.path.join(self.savePathDepth, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                textFile.write("{} \n ".format(timestamp))
                cnt += 1
                with self.kinectDepthQueueSize.get_lock():
                    self.kinectDepthQueueSize.value -= 1

            except Exception as e:
                if self.keepGoing.value or self.kinectDepthQueueSize.value > 0:
                    continue
                else:
                    break
        textFile.close()
        self.logger.log("Finished Depth writing. Grabbed {} frames in the queue".format(cnt))

    def close(self):
        self.logger.log("Closing Kinect...")
        if self.kinect:
            self.kinect.stop()


########################################################################################################################
############################################### FLIR ###################################################################
########################################################################################################################

class FLIR:
    def __init__(self, portNum, flirQueue, keepGoing, flirQueueSize, cameraStatus, logger, savePath):
        self.portNum = portNum
        self.flirQueue = flirQueue
        self.keepGoing = keepGoing
        self.flirQueueSize = flirQueueSize
        self.cameraStatus = cameraStatus
        self.logger = logger
        self.savePath = savePath
        self.flir = None

    def read(self):
        flir = cv2.VideoCapture(self.portNum, cv2.CAP_DSHOW)
        flir.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        flir.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        flir.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cameraStatus[0] = 1
        self.flir = flir

        while np.sum(self.cameraStatus) < len(self.cameraStatus):
            continue

        self.logger.log("FLIR is Recording...")
        t1 = time.time()
        timestamp = datetime.now()
        ret, oldFrame = flir.read()
        self.flirQueue.put([oldFrame, timestamp])
        with self.flirQueueSize.get_lock():
            self.flirQueueSize.value += 1
        cnt = 1
        while self.keepGoing.value:
            timestamp = datetime.now()
            ret, newFrame = flir.read()
            if not ret:
                break
            if are_identical(oldFrame, newFrame):
                continue
            else:
                self.flirQueue.put([newFrame, timestamp])
                cnt += 1
                with self.flirQueueSize.get_lock():
                    self.flirQueueSize.value += 1
            oldFrame = newFrame
        t = time.time() - t1
        cv2.destroyAllWindows()
        flir.release()
        self.logger.log("Finished FLIR acquisition. Put {} frames in the queue".format(cnt))
        self.logger.log("FLIR acquisition lasted {}s".format(round(t, 3)))
        self.logger.log("Overall FLIR FPS: {}".format(round(cnt / t, 3)))

    def write(self):
        cnt = 0
        textFile = open(os.path.join(self.savePath, "timestamps.txt"), "a")
        while True:
            try:
                [frame, timestamp] = self.flirQueue.get_nowait()
                name = str(cnt).zfill(5)
                with open(os.path.join(self.savePath, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                textFile.write("{} \n ".format(timestamp))  # Write timestamp to .txt file
                cnt += 1
                with self.flirQueueSize.get_lock():
                    self.flirQueueSize.value -= 1  # Decrement the queue size

            except Exception as e:
                if self.keepGoing.value or self.flirQueueSize.value > 0:
                    continue
                else:
                    break
        textFile.close()
        self.logger.log("Finished FLIR writing. Grabbed {} frames in the queue".format(cnt))

    def close(self):
        self.logger.log("Closing FLIR...")
        self.flir.release()


########################################################################################################################
############################################### XSENS ##################################################################
########################################################################################################################

class XSens:
    def __init__(self, nIMUs, keepGoing, systemStatus, savePath, logger):
        self.nIMUs = nIMUs
        self.keepGoing = keepGoing
        self.systemStatus = systemStatus
        self.savePath = savePath
        self.logger = logger
        self.updateRate = 60
        self.radioChannel = 13
        self.maxBufferSize = 5
        self.awinda = None
        self.mtwCallbacks = None
        self.filenamesPCKL = None
        self.devId = None
        self.devIdUsed = None
        self.nDevs = None
        self.firmware_version = None
        self.controlDev = None
        self.Ports = None

    def initialiseAwinda(self):
        awinda, mtwCallbacks, filenamesPCKL, devId, devIdUsed, nDevs, firmware_version, controlDev, Ports = mtw.initialiseAwinda(
            self.nIMUs,
            self.updateRate,
            self.radioChannel,
            self.savePath, self.maxBufferSize, self.logger)

        self.awinda = awinda
        self.mtwCallbacks = mtwCallbacks
        self.filenamesPCKL = filenamesPCKL
        self.devId = devId
        self.devIdUsed = devIdUsed
        self.nDevs = nDevs
        self.firmware_version = firmware_version
        self.controlDev = controlDev
        self.Ports = Ports

    def readAndWrite(self):
        self.initialiseAwinda()
        self.systemStatus[2] = 1
        while np.sum(self.systemStatus) < len(self.systemStatus):
            continue

        self.logger.log("XSens is Recording")
        while self.keepGoing.value:  # As long as the acquisition goes on !
            mtw.writeXsens(self.mtwCallbacks, self.filenamesPCKL)

        if not self.awinda.abortFlushing():
            raise Exception("Failed to abort flyshing operation")
            # self.logger.log("Failed to abort flushing operation.")
        self.logger.log("Stopping XSens recording...\n")
        if not self.awinda.stopRecording():
            raise Exception("Failed to stop recording. Aborting.")
            # self.logger.log("Failed to stop recording. Aborting.")

        # Writing the data from a pickle txt file to a readable txt file
        self.logger.log("Writing XSens PICKLE data to .txt ... \n")
        mtw.pickle2txt(self.devId, self.devIdUsed, self.nDevs, self.firmware_version, self.filenamesPCKL,
                       self.updateRate, self.savePath,
                       self.maxBufferSize, self.logger)

        self.logger.log("\n Closing XSens log file...")
        if not self.awinda.closeLogFile():
            # self.logger.log(str(error))
            raise Exception("Failed to close log file. Aborting.")
        self.logger.log("Exiting program...")
        mtw.stopAll(self.awinda, self.controlDev, self.Ports, self.logger)
        # print(self.awinda)

    def close(self):
        if self.awinda:
            mtw.stopAll(self.awinda, self.controlDev, self.Ports, self.logger)


###############################################################################################################
###############################################################################################################
###############################################################################################################


@nb.jit(nopython=True)
def are_identical(oldFrame, newFrame):
    """
    :param oldFrame: frame that was caught by the camera at the previous iteration
    :param newFrame: frame that was caught by the camera at the current iteration
    :return: Bool, True if frames are identical, False if they are different
    """
    if oldFrame.shape != newFrame.shape:
        return False
    for ai, bi in zip(oldFrame.flat, newFrame.flat):
        if ai != bi:
            return False
    return True


def initLogging(savePath):
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H-%M")
    newSavePath = os.path.join(savePath, dt_string)
    savePathDepth = os.path.join(newSavePath, "depth")
    savePathRGB = os.path.join(newSavePath, "rgb")
    savePathWebcam = os.path.join(newSavePath, "webcam")

    if not os.path.isdir(os.path.join(savePath, dt_string)):
        os.mkdir(os.path.join(savePath, dt_string))
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)
        os.mkdir(savePathWebcam)

    return newSavePath, savePathRGB, savePathDepth, savePathWebcam


def findCameraPort(name):
    if name == "webcam":
        targetShape = (480, 640, 3)
    elif name == "flir":
        targetShape = (768, 1024, 3)
    else:
        raise Exception ("Wrong camera type specified")
    cnt = 0
    while True:
        try:
            cam = cv2.VideoCapture(cnt)
            ret, frame = cam.read()
            if not ret:
                raise Exception("Couldn't find {} port".format(name))
            elif frame.shape == targetShape:
                break
            else:
                cnt += 1
        except:
            raise Exception("Couldn't find {} port. Went up to port: {}".format(name, cnt))
    cv2.destroyAllWindows()
    cam.release()

    return cnt


def main():
    pass


if __name__ == "__main__":
    main()
