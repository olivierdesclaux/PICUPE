from datetime import datetime
import os
import time

import cv2
import numpy as np
import pyk4a as k4a
from PIL import Image
import extractDepth
import numba as nb

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
