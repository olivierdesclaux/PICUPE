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

def webcamReader(queue_from_cam, keepGoing, webcamQueueSize, cameraStatus):
    webcam = cv2.VideoCapture(0)
    cameraStatus[0] = 1
    while np.sum(cameraStatus) < 2:
        continue

    print("Webcam is Recording...")
    t1 = time.time()
    timestamp = datetime.now()
    ret, oldFrame = webcam.read()
    queue_from_cam.put([oldFrame, timestamp])
    cnt = 1
    while keepGoing.value:
        timestamp = datetime.now()
        ret, newFrame = webcam.read()
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
    cv2.destroyAllWindows()
    webcam.release()
    print("Finished Webcam acquisition. Put {} frames in the queue".format(cnt))
    print("Webcam acquisition lasted {}s".format(round(t, 3)))
    print("Overall Webcam FPS: {}".format(round(cnt / t, 3)))


def webcamWriter(webcamQueue, savePath, keepGoing, webcamQueueSize):
    cnt = 0
    while True:
        try:
            frame = webcamQueue.get_nowait()
            name = str(cnt)
            frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frameRGB)
            im.save(os.path.join(savePath, name + ".png"))
            cnt += 1
            with webcamQueueSize.get_lock():
                webcamQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or webcamQueueSize.value > 0:
                continue
            else:
                break

    print("Finished Webcam writing. Grabbed {} frames in the queue".format(cnt))


def webcamWriter2(webcamQueue, savePath, keepGoing, webcamQueueSize):
    cnt = 0
    f = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = webcamQueue.get_nowait()
            name = str(cnt)
            frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frameRGB)
            im.save(os.path.join(savePath, name + ".png"))
            f.write("{} \n ".format(timestamp))
            cnt += 1
            with webcamQueueSize.get_lock():
                webcamQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or webcamQueueSize.value > 0:
                continue
            else:
                break
    f.close()
    print("Finished Webcam writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### KINECT #################################################################
########################################################################################################################

def kinectReader(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize, kinectDepthQueueSize, cameraStatus):
    kinect = k4a.PyK4A(k4a.Config(
        color_resolution=k4a.ColorResolution.RES_720P,
        depth_mode=k4a.DepthMode.NFOV_UNBINNED,
        camera_fps=k4a.FPS.FPS_30,
        synchronized_images_only=True,
    ))
    kinect.start()
    cameraStatus[1] = 1
    while np.sum(cameraStatus) < 2:
        continue
    print("Kinect is Recording...")
    t1 = time.time()
    oldFrame = kinect.get_capture()
    kinectRGBQueue.put(oldFrame.color)
    kinectDepthQueue.put(oldFrame.transformed_depth)
    with kinectDepthQueueSize.get_lock():
        kinectDepthQueueSize.value += 1
    with kinectRGBQueueSize.get_lock():
        kinectRGBQueueSize.value += 1
    cntRGB = 1
    cntDepth = 1
    while keepGoing.value:
        newFrame = kinect.get_capture()  # grab new frame
        if are_identical(oldFrame.depth, newFrame.depth):  # check if it is identical to the previous
            continue
        else:  # if not, we put it in the queue for writing
            kinectRGBQueue.put(newFrame.color)
            kinectDepthQueue.put(newFrame.transformed_depth)
            cntRGB += 1
            cntDepth += 1
            with kinectDepthQueueSize.get_lock():
                kinectDepthQueueSize.value += 1
            with kinectRGBQueueSize.get_lock():
                kinectRGBQueueSize.value += 1
        oldFrame = newFrame
    t = time.time() - t1
    kinect.stop()

    print("Finished Kinect acquisition. Put {} frames in the queue".format(cntRGB))
    # print("Finished Kinect acquisition. Put {} frames in the Depth queue".format(cntDepth))
    print("Kinect acquisition lasted {}s".format(round(t, 3)))
    print("Overall Kinect FPS: {}".format(round(cntRGB / t, 3)))


def kinectRGBWriter(kinectRGBQueue, savePath, keepGoing, kinectRGBQueueSize):
    cnt = 0
    timer = []
    while True:
        try:
            t1 = time.time()
            frame = kinectRGBQueue.get_nowait()
            name = str(cnt)
            frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frameRGB)
            im.save(os.path.join(savePath, name + ".png"))
            cnt += 1
            with kinectRGBQueueSize.get_lock():
                kinectRGBQueueSize.value -= 1
            timer.append(time.time() - t1)
        except Exception as e:
            if keepGoing.value or kinectRGBQueueSize.value > 0:
                continue
            else:
                break
    t = float(np.mean(np.array(timer)))
    print("Finished RGB writing. Grabbed {} frames in the queue".format(cnt))
    print("Writing a frame takes an average of: {}s".format(round(t, 3)))


def kinectRGBWriterVideo(kinectRGBQueue, savePath, keepGoing, kinectRGBQueueSize):
    cnt = 0
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    # fourcc = cv2.VideoWriter_fourcc(*'FFV1')
    # fourcc = cv2.VideoWriter_fourcc(*'FFV1')
    # writer = cv2.VideoWriter(os.path.join(savePath, 'output.avi'), fourcc, 30.0, (1280, 720))
    # writer = cv2.VideoWriter(os.path.join(savePath, 'output.mp4'), fourcc, 30.0, (1280, 720))
    writer = cv2.VideoWriter(os.path.join(savePath, 'output.avi'), fourcc, 20.0, (1280, 720))
    while True:
        try:
            frame = kinectRGBQueue.get_nowait()
            writer.write(frame[:, :, :3])
            cnt += 1
            with kinectRGBQueueSize.get_lock():
                kinectRGBQueueSize.value -= 1
        except Exception as e:
            if keepGoing.value or kinectRGBQueueSize.value > 0:
                continue
            else:
                break
    writer.release()
    print("Finished RGB writing. Grabbed {} frames in the queue".format(cnt))


def kinectDepthWriter(kinectDepthQueue, savePath, keepGoing, kinectDepthQueueSize):
    cnt = 0
    while True:
        try:
            frame = kinectDepthQueue.get_nowait()
            name = str(cnt)
            frame = extractDepth.colorize(frame, (None, 5000), cv2.COLORMAP_BONE)
            im = Image.fromarray(frame)
            im.save(os.path.join(savePath, name + ".png"))
            cnt += 1
            with kinectDepthQueueSize.get_lock():
                kinectDepthQueueSize.value -= 1

        except Exception as e:
            if keepGoing.value or kinectDepthQueueSize.value > 0:
                continue
            else:
                break

    print("Finished Depth writing. Grabbed {} frames in the queue".format(cnt))


########################################################################################################################
############################################### FLIR ###################################################################
########################################################################################################################

def flirReader(queue_from_cam, keepGoing, flirQueueSize, cameraStatus):
    flir = cv2.VideoCapture(2, cv2.CAP_DSHOW)
    flir.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
    flir.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
    flir.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # flir = cv2.VideoCapture(2)
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
    while True:
        try:
            frame = flirQueue.get_nowait()
            name = str(cnt)
            frameRGB = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frameRGB)
            im.save(os.path.join(savePath, name + ".png"))
            cnt += 1
            with flirQueueSize.get_lock():
                flirQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or flirQueueSize.value > 0:
                continue
            else:
                break

    print("Finished Webcam writing. Grabbed {} frames in the queue".format(cnt))


def flirWriter2(flirQueue, savePath, keepGoing, flirQueueSize):
    cnt = 0
    f = open(os.path.join(savePath, "timestamps.txt"), "a")
    while True:
        try:
            [frame, timestamp] = flirQueue.get_nowait()
            name = str(cnt)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            im = Image.fromarray(frame)
            im.save(os.path.join(savePath, name + ".png"))
            f.write("{} \n ".format(timestamp))
            cnt += 1
            with flirQueueSize.get_lock():
                flirQueueSize.value -= 1  # Decrement the queue size

        except Exception as e:
            if keepGoing.value or flirQueueSize.value > 0:
                continue
            else:
                break
    f.close()
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
    dt_string = now.strftime("%d-%m-%Y_%H-%M")
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


def main():
    pass


if __name__ == "__main__":
    main()
