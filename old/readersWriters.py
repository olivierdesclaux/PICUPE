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
import myLogger


########################################################################################################################
############################################### WEBCAM #################################################################
########################################################################################################################

def webcamReader(portNum, queue_from_cam, keepGoing, webcamQueueSize, cameraStatus, loggerQueue):
    try:
        webcam = cv2.VideoCapture(portNum)
        # webcam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        cameraStatus[0] = 1
    except Exception as e:
        myLogger.log(loggerQueue, "Failed to open webcam")
        sys.exit()
    while np.sum(cameraStatus) < len(cameraStatus):
        continue

    # print("Webcam is Recording...")
    myLogger.log(loggerQueue, "Webcam is Recording...")
    t1 = time.time()
    timestamp = datetime.now()
    ret, oldFrame = webcam.read()
    queue_from_cam.put([oldFrame, timestamp])
    with webcamQueueSize.get_lock():
        webcamQueueSize.value += 1
    cnt = 1
    while keepGoing.value:
        timestamp = datetime.now()
        ret, newFrame = webcam.read()  # grab new frame
        if not ret:
            break
        if are_identical(oldFrame, newFrame):
            continue
        else:
            queue_from_cam.put([newFrame, timestamp])
            cnt += 1
            with webcamQueueSize.get_lock():
                webcamQueueSize.value += 1

        oldFrame = newFrame
    t = time.time() - t1

    webcam.release()
    cv2.destroyAllWindows()
    myLogger.log(loggerQueue, "Finished Webcam acquisition. Put {} frames in the queue".format(cnt))
    myLogger.log(loggerQueue, "Webcam acquisition lasted {}s".format(round(t, 3)))
    myLogger.log(loggerQueue, "Overall Webcam FPS: {}".format(round(cnt / t, 3)))


def webcamWriter(webcamQueue, savePath, keepGoing, webcamQueueSize, loggerQueue):
    cnt = 0  # Initialise counter. Images will have name 00000, 00001, 00002
    textFile = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = webcamQueue.get_nowait()
            name = str(cnt).zfill(5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
            with open(os.path.join(savePath, name), 'wb') as f:
                np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
            textFile.write("{}\n".format(timestamp))  # Add timestamp to the .txt file
            cnt += 1
            with webcamQueueSize.get_lock():  # Have to access lock otherwise it could miss the increment by the reader
                webcamQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or webcamQueueSize.value > 0:
                continue
            else:
                break
    textFile.close()
    myLogger.log(loggerQueue, "Finished Webcam writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### KINECT #################################################################
########################################################################################################################

def kinectReader(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize, kinectDepthQueueSize, cameraStatus,
                 loggerQueue):
    try:
        kinect = k4a.PyK4A(k4a.Config(
            color_resolution=k4a.ColorResolution.RES_720P,
            depth_mode=k4a.DepthMode.NFOV_UNBINNED,
            camera_fps=k4a.FPS.FPS_30,
            synchronized_images_only=True, ))
        kinect.start()
        cameraStatus[1] = 1
    except Exception as e:
        myLogger.log(loggerQueue, "Failed to open Kinect")
        sys.exit()

    while np.sum(cameraStatus) < len(cameraStatus):
        continue
    myLogger.log(loggerQueue, "Kinect is Recording...")
    t1 = time.time()
    timestamp = datetime.now()
    oldFrame = kinect.get_capture()
    kinectRGBQueue.put([oldFrame.color, timestamp])
    kinectDepthQueue.put([oldFrame.transformed_depth, timestamp])
    with kinectDepthQueueSize.get_lock():
        kinectDepthQueueSize.value += 1
    with kinectRGBQueueSize.get_lock():
        kinectRGBQueueSize.value += 1
    cnt = 1  # Counter to compute the FPS (verbosity)
    while keepGoing.value:
        timestamp = datetime.now()
        newFrame = kinect.get_capture()  # grab new frame
        if are_identical(oldFrame.depth, newFrame.depth):  # check if new depth is identical to the previous depth
            continue
        else:  # if not, we put it in the queue for writing
            kinectRGBQueue.put([newFrame.color, timestamp])
            kinectDepthQueue.put([newFrame.transformed_depth, timestamp])
            cnt += 1
            with kinectDepthQueueSize.get_lock():
                kinectDepthQueueSize.value += 1
            with kinectRGBQueueSize.get_lock():
                kinectRGBQueueSize.value += 1
        oldFrame = newFrame
    t = time.time() - t1
    kinect.stop()

    myLogger.log(loggerQueue, "Finished Kinect acquisition. Put {} frames in the queue".format(cnt))
    myLogger.log(loggerQueue, "Kinect acquisition lasted {}s".format(round(t, 3)))
    myLogger.log(loggerQueue, "Overall Kinect FPS: {}".format(round(cnt / t, 3)))


def kinectRGBWriter(kinectRGBQueue, savePath, keepGoing, kinectRGBQueueSize, loggerQueue):
    cnt = 0  # Initialise counter. Images will have name 00000, 00001, 00002
    textFile = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = kinectRGBQueue.get_nowait()
            name = str(cnt).zfill(5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
            # frame = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2RGB)
            with open(os.path.join(savePath, name), 'wb') as f:
                np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
            textFile.write("{}\n".format(timestamp))  # Add timestamp to the .txt file
            cnt += 1
            with kinectRGBQueueSize.get_lock():  # Have to access lock otherwise it could miss the increment by the
                # reader
                kinectRGBQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or kinectRGBQueueSize.value > 0:
                continue
            else:
                break
    textFile.close()
    myLogger.log(loggerQueue, "Finished RGB writing. Grabbed {} frames in the queue".format(cnt))


def kinectDepthWriter(kinectDepthQueue, savePath, keepGoing, kinectDepthQueueSize, loggerQueue):
    cnt = 0
    textFile = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = kinectDepthQueue.get_nowait()
            name = str(cnt).zfill(5)
            frame = extractDepth.colorize(frame, (None, 5000), cv2.COLORMAP_BONE)
            with open(os.path.join(savePath, name), 'wb') as f:
                np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
            textFile.write("{} \n ".format(timestamp))
            cnt += 1
            with kinectDepthQueueSize.get_lock():
                kinectDepthQueueSize.value -= 1

        except Exception as e:
            if keepGoing.value or kinectDepthQueueSize.value > 0:
                continue
            else:
                break
    textFile.close()
    myLogger.log(loggerQueue, "Finished Depth writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### FLIR ###################################################################
########################################################################################################################

def flirReader(portNum, queue_from_cam, keepGoing, flirQueueSize, cameraStatus, loggerQueue):
    try:
        flir = cv2.VideoCapture(portNum, cv2.CAP_DSHOW)
        flir.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        flir.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        flir.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cameraStatus[0] = 1

    except Exception as e:
        myLogger.log(loggerQueue, "Failed to open FLIR")
        sys.exit()

    while np.sum(cameraStatus) < len(cameraStatus):
        continue

    myLogger.log(loggerQueue, "FLIR is Recording...")
    t1 = time.time()
    timestamp = datetime.now()
    ret, oldFrame = flir.read()
    queue_from_cam.put([oldFrame, timestamp])
    with flirQueueSize.get_lock():
        flirQueueSize.value += 1
    cnt = 1
    while keepGoing.value:
        timestamp = datetime.now()
        ret, newFrame = flir.read()
        if not ret:
            break
        if are_identical(oldFrame, newFrame):
            continue
        else:
            queue_from_cam.put([newFrame, timestamp])
            cnt += 1
            with flirQueueSize.get_lock():
                flirQueueSize.value += 1
        oldFrame = newFrame
    t = time.time() - t1
    cv2.destroyAllWindows()
    flir.release()
    myLogger.log(loggerQueue, "Finished FLIR acquisition. Put {} frames in the queue".format(cnt))
    myLogger.log(loggerQueue, "FLIR acquisition lasted {}s".format(round(t, 3)))
    myLogger.log(loggerQueue, "Overall FLIR FPS: {}".format(round(cnt / t, 3)))


def flirWriter(flirQueue, savePath, keepGoing, flirQueueSize, loggerQueue):
    cnt = 0
    textFile = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = flirQueue.get_nowait()
            name = str(cnt).zfill(5)
            with open(os.path.join(savePath, name), 'wb') as f:
                np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
            textFile.write("{} \n ".format(timestamp))  # Write timestamp to .txt file
            cnt += 1
            with flirQueueSize.get_lock():
                flirQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or flirQueueSize.value > 0:
                continue
            else:
                break
    textFile.close()
    myLogger.log(loggerQueue, "Finished FLIR writing. Grabbed {} frames in the queue".format(cnt))

########################################################################################################################
############################################### XSENS ##################################################################
########################################################################################################################


def xsensReaderWriter(nIMUs, keepGoing, systemStatus, savePathXsens, loggerQueue):
    XSupdateRate = 60
    XSradioChannel = 13
    maxBufferSize = 5
    awinda, mtwCallbacks, filenamesPCKL, devId, devIdUsed, nDevs, firmware_version, controlDev, Ports = mtw.initialiseAwinda(
        nIMUs,
        XSupdateRate,
        XSradioChannel,
        savePathXsens, maxBufferSize, loggerQueue)
    systemStatus[2] = 1
    while np.sum(systemStatus) < len(systemStatus):
        continue

    myLogger.log(loggerQueue, "XSens is Recording")
    while keepGoing.value:  # As long as the acquisition goes on !
        mtw.writeXsens(mtwCallbacks, filenamesPCKL)

    if not awinda.abortFlushing():
        myLogger.log(loggerQueue, "Failed to abort flushing operation.")
    myLogger.log(loggerQueue, "Stopping XSens recording...\n")
    if not awinda.stopRecording():
        myLogger.log(loggerQueue, "Failed to stop recording. Aborting.")

    # Writing the data from a pickle txt file to a readable txt file
    myLogger.log(loggerQueue, "Writing XSens PICKLE data to .txt ... \n")
    mtw.pickle2txt(devId, devIdUsed, nDevs, firmware_version, filenamesPCKL, XSupdateRate, savePathXsens,
                   maxBufferSize, loggerQueue)

    myLogger.log(loggerQueue, "\n Closing XSens log file...")
    if not awinda.closeLogFile():
        error = RuntimeError("Failed to close log file. Aborting.")
        myLogger.log(loggerQueue, str(error))
        raise error
    myLogger.log(loggerQueue, "Exiting program...")
    mtw.stopAll(awinda, controlDev, Ports)

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
        raise ValueError("Wrong camera type specified")
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
