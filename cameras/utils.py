from datetime import datetime
import os
import time

import cv2
import numpy as np
import pyk4a as k4a
from PIL import Image
import extractDepth
import numba as nb


########################################################################################################################
############################################### WEBCAM #################################################################
########################################################################################################################

def webcamReader(portNum, queue_from_cam, keepGoing, webcamQueueSize, cameraStatus):
    webcam = cv2.VideoCapture(portNum)
    # webcam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cameraStatus[0] = 1
    while np.sum(cameraStatus) < 2:
        continue

    print("Webcam is Recording...")
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
    print("Finished Webcam acquisition. Put {} frames in the queue".format(cnt))
    print("Webcam acquisition lasted {}s".format(round(t, 3)))
    print("Overall Webcam FPS: {}".format(round(cnt / t, 3)))


def webcamWriter(webcamQueue, savePath, keepGoing, webcamQueueSize):
    cnt = 0  # Initialise counter. Images will have name 00000, 00001, 00002
    textFile = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = webcamQueue.get_nowait()
            name = str(cnt).zfill(5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
            with open(os.path.join(savePath, name), 'wb') as f:
                np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
            textFile.write("{} \n ".format(timestamp))  # Add timestamp to the .txt file
            cnt += 1
            with webcamQueueSize.get_lock():  # Have to access lock otherwise it could miss the increment by the reader
                webcamQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or webcamQueueSize.value > 0:
                continue
            else:
                break
    textFile.close()
    print("Finished Webcam writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### KINECT #################################################################
########################################################################################################################

def kinectReader(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize, kinectDepthQueueSize, cameraStatus):
    kinect = k4a.PyK4A(k4a.Config(
        color_resolution=k4a.ColorResolution.RES_720P,
        depth_mode=k4a.DepthMode.NFOV_UNBINNED,
        camera_fps=k4a.FPS.FPS_30,
        synchronized_images_only=True, ))
    kinect.start()
    cameraStatus[1] = 1
    while np.sum(cameraStatus) < 2:
        continue
    print("Kinect is Recording...")
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

    print("Finished Kinect acquisition. Put {} frames in the queue".format(cnt))
    print("Kinect acquisition lasted {}s".format(round(t, 3)))
    print("Overall Kinect FPS: {}".format(round(cnt / t, 3)))


def kinectRGBWriter(kinectRGBQueue, savePath, keepGoing, kinectRGBQueueSize):
    cnt = 0  # Initialise counter. Images will have name 00000, 00001, 00002
    textFile = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = kinectRGBQueue.get_nowait()
            name = str(cnt).zfill(5)  # Convert counter to string with zeros in front. e.g. 12 becomes 00012
            # frame = cv2.cvtColor(frame[:, :, :3], cv2.COLOR_BGR2RGB)
            with open(os.path.join(savePath, name), 'wb') as f:
                np.save(f, frame, allow_pickle=False)  # Write the image to a .npy file
            textFile.write("{} \n ".format(timestamp))  # Add timestamp to the .txt file
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
    print("Finished RGB writing. Grabbed {} frames in the queue".format(cnt))


def kinectDepthWriter(kinectDepthQueue, savePath, keepGoing, kinectDepthQueueSize):
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
    print("Finished Depth writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### FLIR ###################################################################
########################################################################################################################

def flirReader(portNum, queue_from_cam, keepGoing, flirQueueSize, cameraStatus):
    flir = cv2.VideoCapture(portNum, cv2.CAP_DSHOW)
    # flir = cv2.VideoCapture(1)
    flir.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    flir.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    flir.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

    cameraStatus[0] = 1
    while np.sum(cameraStatus) < 2:
        continue

    print("FLIR is Recording...")
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
    print("Finished FLIR acquisition. Put {} frames in the queue".format(cnt))
    print("FLIR acquisition lasted {}s".format(round(t, 3)))
    print("Overall FLIR FPS: {}".format(round(cnt / t, 3)))


def flirWriter(flirQueue, savePath, keepGoing, flirQueueSize):
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
    print("Finished FLIR writing. Grabbed {} frames in the queue".format(cnt))


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
                raise Exception("Blq Couldn't find {} port".format(name))
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
