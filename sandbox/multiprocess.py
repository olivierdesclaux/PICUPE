import multiprocessing
import os
import sys
import time
from datetime import datetime
import msvcrt

import cv2
import numpy as np
import numba as nb
import pyk4a as k4a
from PIL import Image

sys.path.append('../Python-Recalage')
import extractDepth


def webcamStream(queue_from_cam, keepGoing, webcamQueueSize):
    webcam = cv2.VideoCapture(0)
    ret, oldFrame = webcam.read()
    queue_from_cam.put(oldFrame)
    cnt = 1
    while keepGoing.value:
        ret, newFrame = webcam.read()
        if not ret:
            break
        if are_identical(oldFrame, newFrame):
            continue
        else:
            queue_from_cam.put(newFrame)
            cnt += 1
            with webcamQueueSize.get_lock():
                webcamQueueSize.value += 1
        oldFrame = newFrame
    print("Finished Webcam acquisition. Put {} frames in the queue".format(cnt))
    cv2.destroyAllWindows()
    webcam.release()


def kinectStream(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize, kinectDepthQueueSize):
    kinect = k4a.PyK4A(k4a.Config(
        color_resolution=k4a.ColorResolution.RES_720P,
        depth_mode=k4a.DepthMode.NFOV_UNBINNED,
        camera_fps=k4a.FPS.FPS_30,
        synchronized_images_only=True,
    ))
    kinect.start()
    oldFrame = kinect.get_capture()
    kinectRGBQueue.put(oldFrame.color)
    kinectDepthQueue.put(oldFrame.transformed_depth)
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

    print("Finished Kinect acquisition. Put {} frames in the RGB queue".format(cntRGB))
    print("Finished Kinect acquisition. Put {} frames in the Depth queue".format(cntDepth))
    kinect.stop()


@nb.jit(nopython=True)
def are_identical(oldFrame, newFrame):
    """
    :param oldFrame: frame that was caught by the camera at the previous iteratiom
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
    savePathDepth = os.path.join(savePath, dt_string, "depth")
    savePathRGB = os.path.join(savePath, dt_string, "rgb")
    savePathWebcam = os.path.join(savePath, dt_string, "webcam")

    if not os.path.isdir(os.path.join(savePath, dt_string)):
        os.mkdir(os.path.join(savePath, dt_string))
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)
        os.mkdir(savePathWebcam)

    return savePathRGB, savePathDepth, savePathWebcam


def visualizeResults(savePath):
    savePathRGB, savePathDepth, savePathWebcam = initLogging(savePath)
    RGB = [os.path.join(savePathRGB, image) for image in os.listdir(savePathRGB)]
    Depth = [os.path.join(savePathDepth, image) for image in os.listdir(savePathDepth)]
    Webcam = [os.path.join(savePathWebcam, image) for image in os.listdir(savePathWebcam)]
    cnt = 0
    while True:
        try:
            rgbFrame = cv2.imread(RGB[cnt])
            cv2.imshow("RGB", rgbFrame)
        except:
            pass
        try:
            depthFrame = cv2.imread(Depth[cnt])
            cv2.imshow("Depth", depthFrame)
        except:
            pass
        try:
            webcamFrame = cv2.imread(Webcam[cnt])
            cv2.imshow("Webcam", webcamFrame)
        except:
            pass

        if cv2.waitKey(1) == ord("q"):
            break
        cv2.destroyAllWindows()


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
    t = np.mean(np.array(timer))
    print("Finished RGB writing. Grabbed {} frames in the queue".format(cnt))
    print("Writing a frame takes an average of: {}s".format(round(t, 3)))


def kinectRGBWriter2(kinectRGBQueue, savePath, keepGoing, kinectRGBQueueSize):
    cnt = 0
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    writer = cv2.VideoWriter(os.path.join(savePath, 'output.avi'), fourcc, 20.0, (1280, 720), 0)
    while True:
        try:
            frame = kinectRGBQueue.get_nowait()
            writer.write(frame[:, :, -1])
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


def main():
    savePath = r"C:\Users\picup\OneDrive\Recherche\PICUPE\sandbox\results"
    savePathRGB, savePathDepth, savePathWebcam = initLogging(savePath)

    readerProcesses = []
    writerProcesses = []
    queues = []
    keepGoing = multiprocessing.Value('i', 1)
    kinectRGBQueueSize = multiprocessing.Value('i', 0)
    kinectDepthQueueSize = multiprocessing.Value('i', 0)
    webcamQueueSize = multiprocessing.Value('i', 0)

    # # # # # # # # # # # # # # WEBCAM # # # # # # # # # # # # # # # # # #
    # Webcam object
    # webcam = cv2.VideoCapture(0)
    # Webcam Queue
    webcamQueue = multiprocessing.Queue()
    queues.append(webcamQueue)
    # Webcam Process
    webcamReaderProcess = multiprocessing.Process(target=webcamStream, args=(webcamQueue, keepGoing, webcamQueueSize))
    webcamWriterProcess = multiprocessing.Process(target=webcamWriter,
                                                  args=(webcamQueue, savePathWebcam, keepGoing, webcamQueueSize))
    readerProcesses.append(webcamReaderProcess)
    writerProcesses.append(webcamWriterProcess)

    # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
    # Kinect Object
    # kinect = k4a.PyK4A(k4a.Config(
    #     color_resolution=k4a.ColorResolution.RES_720P,
    #     depth_mode=k4a.DepthMode.NFOV_UNBINNED,
    #     camera_fps=k4a.FPS.FPS_30,
    #     synchronized_images_only=True,
    # ))
    # kinect.start()
    # Kinect Queue
    kinectRGBQueue = multiprocessing.Queue()
    kinectDepthQueue = multiprocessing.Queue()
    queues.append(kinectRGBQueue)
    queues.append(kinectDepthQueue)
    # Kinect Process
    kinectReaderProcess = multiprocessing.Process(target=kinectStream,
                                                  args=(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize,
                                                        kinectDepthQueueSize))
    kinectRGBWriterProcess = multiprocessing.Process(target=kinectRGBWriter,
                                                     args=(kinectRGBQueue, savePathRGB, keepGoing, kinectRGBQueueSize))
    kinectDepthWriterProcess = multiprocessing.Process(target=kinectDepthWriter,
                                                       args=(kinectDepthQueue, savePathDepth, keepGoing,
                                                             kinectDepthQueueSize))
    readerProcesses.append(kinectReaderProcess)
    writerProcesses.extend([kinectRGBWriterProcess, kinectDepthWriterProcess])

    # Start processes
    for proc in readerProcesses:
        proc.start()
    for proc in writerProcesses:
        proc.start()

    t1 = time.time()
    kinectCnt = 0
    webcamCnt = 0
    print("Recording ...")
    while True:
        if msvcrt.kbhit():
            if msvcrt.getwche() == '\r':
                keepGoing.value = 0
                print("\n Stopping recording... \n")
                break

    t2 = time.time()

    print("Destroying reader processes...")
    for proc in readerProcesses:
        proc.join()
        proc.terminate()
    print("Successfully destroyed reader processes \n")

    print("Destroying writer processes...")
    for proc in writerProcesses:
        proc.join()
    for proc in writerProcesses:
        proc.terminate()

    print("Successfully destroyed writer processes... \n")

    print("Acquisition time", t2 - t1)
    print("Kinect FPS: ", round(kinectCnt / (t2 - t1), 3))
    print("Webcam FPS: ", round(webcamCnt / (t2 - t1), 3))

    # time.sleep(2)
    VIZ_RESULTS = False

    if VIZ_RESULTS:
        visualizeResults(savePath)


if __name__ == '__main__':
    main()
