import threading
from datetime import datetime
import os
import time
import cv2.cv2 as cv2
import numpy as np
import pyk4a as k4a
from utils.extractDepth import colorize
import numba as nb
import XSens.MTwFunctions as mtw
from XSens.MTwManager import MTwManager
import multiprocessing


########################################################################################################################
############################################### WEBCAM/FLIR ############################################################
########################################################################################################################

class FLIR:
    def __init__(self, name, portNum, keepGoing, cameraStatus, logger, savePath, barrier):
        self.name = name  # flir or webcam
        self.portNum = portNum  # Associated port number
        self.queue = multiprocessing.Queue()  # For putting and getting frames
        self.queueSize = multiprocessing.Value('i', 0)  # Counter for knowing the queue size
        self.keepGoing = keepGoing  # Shared variable between process to make processes continue
        self.cameraStatus = cameraStatus  # Shared variable between processes. 0 if camera is not ready, 1 otherwise
        self.logger = logger  # For logging purposes
        self.savePath = savePath  # Directory to write images
        self.cam = None  # Initialise camera object
        self.barrier = barrier  # Shared with other camera process to ensure synchronization
        self.writingCounter = 0  # Initialise counter. Images will have name 00000, 00001, 00002
        self.textFile = None  # Text file for writing timestamps

    def initCam(self):
        """
        Launches camera (flir or webcam) using opencv VideoCapture.
        Stores VideoCapture object in self.cam

        Returns
        -------
        None
        """
        if self.name == "webcam":
            cam = cv2.VideoCapture(self.portNum, cv2.CAP_DSHOW)
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cam = cam

        elif self.name == "flir":
            cam = cv2.VideoCapture(self.portNum, cv2.CAP_DSHOW)
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cam = cam
        else:
            raise Exception("Invalid camera name")

    def read(self):
        """
        Function that will be processed in the main. Reads frames, puts them in the queue and then waits for other
        camera at a barrier.
        Returns
        -------

        """
        self.initCam()
        self.cameraStatus[0] = 1  # Inform other processes that camera was initialised and is ready

        ret, oldFrame = self.cam.read()  # Read frame
        oldTimestamp = datetime.now()  # Get associated timestamp
        self.queue.put([oldFrame, oldTimestamp])  # Put them both in the queue
        with self.queueSize.get_lock():
            self.queueSize.value += 1  # Update queue size
        cnt = 1

        while np.sum(self.cameraStatus) < len(self.cameraStatus):
            continue  # Wait for all other processes to be ready

        self.logger.log("{} is Recording...".format(self.name))
        t1 = time.time()  # For computing FPS

        # self.barrier.wait()  # Wait for kinect before grabbing next frame
        while self.keepGoing.value:
            self.cam.grab()
            timestamp = datetime.now()
            ret, newFrame = self.cam.retrieve()  # grab and retrieve more efficient than cam.read()
            if not ret:
                break
            else:
                self.queue.put([newFrame, timestamp])  # Put frame in queue
                cnt += 1
                with self.queueSize.get_lock():  # Update queue size
                    self.queueSize.value += 1
            self.barrier.wait()  # Wait for kinect

        t = time.time() - t1
        self.cam.release()
        cv2.destroyAllWindows()
        self.logger.log("Finished {} acquisition. Put {} frames in the queue".format(self.name, cnt))
        self.logger.log("Webcam acquisition lasted {}s".format(round(t, 3)))
        self.logger.log("Overall {} FPS: {}".format(self.name, round(cnt / t, 3)))

    def close(self):
        """
        Closes camera object and destroys all potential existing windows
        Returns
        -------
        None
        """
        if self.cam:
            self.cam.release()
            cv2.destroyAllWindows()

    def write(self):
        """
        Grabs frames stored in the shared queue and stores them as .npy files.
        Returns
        -------
        None
        """
        self.textFile = open(os.path.join(self.savePath, "timestamps.txt"), "a")  # Open text file
        while True:
            try:
                [frame, timestamp] = self.queue.get_nowait()
                name = str(self.writingCounter).zfill(5)  # Convert counter to string with zeros in front. e.g. 12
                # becomes 00012
                with open(os.path.join(self.savePath, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                self.textFile.write("{}\n".format(timestamp))  # Add timestamp to the .txt file
                self.writingCounter += 1
                with self.queueSize.get_lock():  # Have to access lock otherwise it could miss the increment by
                    # the reader
                    self.queueSize.value -= 1  # Decrement the queue size
            except Exception as e:
                # If you can't grab a new frame, check that the main is still running and that the queue isn't empty.
                # If none of these conditions are met, terminate.
                if self.keepGoing.value or self.queueSize.value > 0:
                    continue
                else:
                    break

        self.textFile.close()
        self.logger.log("Finished Webcam writing. Grabbed {} frames in the queue".format(self.writingCounter))


########################################################################################################################
############################################### KINECT #################################################################
########################################################################################################################

class Kinect:
    def __init__(self, keepGoing, cameraStatus, logger, savePathRGB, savePathDepth, barrier):
        self.RGBQueue = multiprocessing.Queue()  # For putting and getting RGB frames
        self.DepthQueue = multiprocessing.Queue()  # For putting and getting depth frames
        self.RGBQueueSize = multiprocessing.Value('i', 0)  # For counting queue size
        self.DepthQueueSize = multiprocessing.Value('i', 0)  # For counting queue size

        self.keepGoing = keepGoing  # Shared variable between process to make processes continue
        self.cameraStatus = cameraStatus  # Shared variable between processes. 0 if camera is not ready, 1 otherwise
        self.logger = logger  # For logging purposes
        self.savePathRGB = savePathRGB  # For writing rgb images
        self.savePathDepth = savePathDepth  # For writing depth images
        self.kinect = None  # Will store pyk4a kinect object
        self.barrier = barrier  # For synchro with Flir
        self.writingCounterRGB = 0  # For naming the saved images
        self.writingCounterDepth = 0
        self.frame = None
        self.timestamp = datetime.now()

    def update(self):
        kinect = k4a.PyK4A(k4a.Config(
            color_resolution=k4a.ColorResolution.RES_720P,
            depth_mode=k4a.DepthMode.NFOV_UNBINNED,
            camera_fps=k4a.FPS.FPS_30,
            synchronized_images_only=True, ))
        kinect.start()

        self.kinect = kinect
        self.frame = self.kinect.get_capture()
        self.timestamp = datetime.now()

        while self.keepGoing.value:
            self.frame = self.kinect.get_capture()
            self.timestamp = datetime.now()
            cv2.waitKey(1)

    def read2(self):
        updateThread = threading.Thread(target=self.update, daemon=True)
        updateThread.start()

        self.cameraStatus[1] = 1
        while np.sum(self.cameraStatus) < len(self.cameraStatus):
            continue

        self.logger.log("Kinect is Recording...")
        while self.keepGoing.value:
            newFrame = self.frame
            self.RGBQueue.put([newFrame.color, self.timestamp])
            self.DepthQueue.put([newFrame.transformed_depth, self.timestamp])

            with self.DepthQueueSize.get_lock():
                self.DepthQueueSize.value += 1  # Update queue size
            with self.RGBQueueSize.get_lock():
                self.RGBQueueSize.value += 1  # Update queue size

            self.barrier.wait()

        updateThread.join()

    def read(self):
        """
        Initialise kinect camera object. Processed in main to continuously read images and put them in adequate queues.
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
        self.cameraStatus[1] = 1

        oldTimestamp = datetime.now()
        oldFrame = kinect.get_capture()
        self.RGBQueue.put([oldFrame.color, oldTimestamp])
        self.DepthQueue.put([oldFrame.transformed_depth, oldTimestamp])
        with self.DepthQueueSize.get_lock():
            self.DepthQueueSize.value += 1  # Update queue size
        with self.RGBQueueSize.get_lock():
            self.RGBQueueSize.value += 1  # Update queue size

        cnt = 1  # Counter to compute the FPS (verbosity)
        while np.sum(self.cameraStatus) < len(self.cameraStatus):
            continue
        self.logger.log("Kinect is Recording...")
        t1 = time.time()

        newFrame = self.kinect.get_capture()
        while True:
            f = self.kinect.get_capture_timeout(0)
            if f is None:
                break

        while self.keepGoing.value:
            newFrame = self.kinect.get_capture()
            while True:
                f = self.kinect.get_capture_timeout(5)
                if f is None:
                    break
                else:
                    newFrame = f

            timestamp = datetime.now()

            self.RGBQueue.put([newFrame.color, timestamp])
            self.DepthQueue.put([newFrame.transformed_depth, timestamp])
            cnt += 1
            with self.DepthQueueSize.get_lock():
                self.DepthQueueSize.value += 1
            with self.RGBQueueSize.get_lock():
                self.RGBQueueSize.value += 1
            self.barrier.wait()

        t = time.time() - t1

        self.logger.log("Finished Kinect acquisition. Put {} frames in the queue".format(cnt))
        self.logger.log("Kinect acquisition lasted {}s".format(round(t, 3)))
        self.logger.log("Overall Kinect FPS: {}".format(round(cnt / t, 3)))

    def writeRGB(self):
        textFile = open(os.path.join(self.savePathRGB, "timestamps.txt"), "a")
        while True:
            try:
                [frame, timestamp] = self.RGBQueue.get_nowait()
                name = str(self.writingCounterRGB).zfill(
                    5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
                with open(os.path.join(self.savePathRGB, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                textFile.write("{}\n".format(timestamp))  # Add timestamp to the .txt file
                self.writingCounterRGB += 1
                with self.RGBQueueSize.get_lock():  # Have to access lock otherwise it could miss the increment
                    # by the reader
                    self.RGBQueueSize.value -= 1  # Decrement the queue size

            except Exception as e:
                if self.keepGoing.value or self.RGBQueueSize.value > 0:
                    continue
                else:
                    break

        textFile.close()
        self.logger.log("Finished RGB writing. Grabbed {} frames in the queue".format(self.writingCounterRGB))

    def writeDepth(self):
        textFile = open(os.path.join(self.savePathDepth, "timestamps.txt"), "a")
        while True:
            try:
                [frame, timestamp] = self.DepthQueue.get_nowait()
                name = str(self.writingCounterDepth).zfill(5)
                # frame = colorize(frame, (None, 5000), cv2.COLORMAP_BONE)
                with open(os.path.join(self.savePathDepth, name), 'wb') as f:
                    np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
                textFile.write("{}\n".format(timestamp))
                self.writingCounterDepth += 1
                with self.DepthQueueSize.get_lock():
                    self.DepthQueueSize.value -= 1

            except Exception as e:
                if self.keepGoing.value or self.DepthQueueSize.value > 0:
                    continue
                else:
                    break
        textFile.close()
        # self.logger.log(str(self.DepthQueueSize.value))
        self.logger.log("Finished Depth writing. Grabbed {} frames in the queue".format(self.writingCounterDepth))

    def close(self):
        self.logger.log("Closing Kinect...")
        if self.kinect:
            self.kinect.stop()


########################################################################################################################
############################################### XSENS ##################################################################
########################################################################################################################
class XSens:
    def __init__(self, IMUs, keepGoing, systemStatus, savePath, logger):
        self.IMUs = IMUs
        self.keepGoing = keepGoing
        self.systemStatus = systemStatus
        self.savePath = savePath
        self.logger = logger
        self.updateRate = 60
        self.radioChannel = 13
        self.maxBufferSize = 5
        self.mtwManager = MTwManager(self.IMUs, self.savePath, self.logger)

    def readAndWrite(self):
        self.mtwManager.initialise()
        self.systemStatus[2] = 1
        while np.sum(self.systemStatus) < len(self.systemStatus):
            continue

        self.logger.log("XSens is Recording")
        while self.keepGoing.value:  # As long as the acquisition goes on !
            self.mtwManager.writeXsens()

        if not self.mtwManager.device.abortFlushing():
            raise Exception("Failed to abort flushing operation")
        self.logger.log("Stopping XSens recording...\n")

        try:
            self.mtwManager.device.stopRecording()
        except Exception as e:
            print(e)
            raise Exception("Failed to stop recording. Aborting.")

        # Writing the data from a pickle txt file to a readable txt file
        self.logger.log("Writing XSens PICKLE data to .txt ... \n")
        mtw.pickle2txt(self.mtwManager)

        self.mtwManager.stop()

    def close(self):
        # if self.mtwManager.device is not None:
        self.logger.log("Closing")
        self.mtwManager.stop()


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
    # print("Searching for {}".format(name))
    if name == "webcam":
        targetShape = (480, 640, 3)
    elif name == "flir":
        targetShape = (768, 1024, 3)
    else:
        raise Exception("Wrong camera type specified")
    cnt = 0
    cam = None
    while True:
        try:
            cam = cv2.VideoCapture(cnt, cv2.CAP_DSHOW)
            ret, frame = cam.read()
            if not ret:
                cnt = -1
                cam.release()
                break
                # raise Exception("Couldn't find {} port".format(name))
            elif frame.shape == targetShape:
                cam.release()
                break
            else:
                cam.release()
                cnt += 1
        except:
            cnt = -1
            print("Couldn't find {} port.".format(name))
            break
            # raise Exception("Couldn't find {} port. Went up to port: {}".format(name, cnt))
    cv2.destroyAllWindows()
    if cam:
        cam.release()
    if cnt > -1:
        print("Found {}".format(name))
    else:
        print("Couldn't find {}".format(name))
    return cnt


def main():
    pass


if __name__ == "__main__":
    main()
