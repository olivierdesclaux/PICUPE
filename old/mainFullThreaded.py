import sys
import os
import cv2
import threading
import time
from datetime import datetime
from PIL import Image
import msvcrt
import xsensdeviceapi as xda
import argparse

sys.path.append('./Python-Recalage')
sys.path.append('./XSens')

import videostream
from utils import extractDepth
import MTwFunctions as mtw


class ImageSaver:
    def __init__(self, mode, savePath):
        self.mode = mode
        self.savePath = savePath
        threading.Thread(target=self.save, args=(), daemon=True).start()

    def save(self, image, name):
        if self.mode == "ir":
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        elif self.mode == 'depth':
            image = extractDepth.colorize(image, (None, 5000), cv2.COLORMAP_BONE)
        else:
            pass
        im = Image.fromarray(image)
        im.save(os.path.join(self.savePath, name))


def imageSaver(image, name, mode, savePath):
    if mode == "ir" or mode == 'rgb':
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    elif mode == 'depth':
        image = extractDepth.colorize(image, (None, 5000), cv2.COLORMAP_BONE)
    else:
        raise NameError("Wrong argument")
    im = Image.fromarray(image)
    im.save(os.path.join(savePath, name + ".png"))


def initLogging(savePath):
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%Y_%H-%M")
    savePathIR = os.path.join(savePath, dt_string, "ir")
    savePathDepth = os.path.join(savePath, dt_string, "depth")
    savePathRGB = os.path.join(savePath, dt_string, "rgb")
    savePathXSens = os.path.join(savePath, dt_string, "XSens")
    if not os.path.isdir(os.path.join(savePath, dt_string)):
        os.mkdir(os.path.join(savePath, dt_string))
        os.mkdir(savePathIR)
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)
        os.mkdir(savePathXSens)

    return savePathIR, savePathRGB, savePathDepth, savePathXSens


def writeXsens(mtwCallbacks, filenamesPCKL):
    for callback, filenamePCKL in zip(mtwCallbacks, filenamesPCKL):
        if callback.packetAvailable():
            callback.writeData(filenamePCKL)


def main(XSupdateRate, XSradioChannel):
    SAVE = True
    VIZ = True
    cameras = 'K'
    global keep_going
    keep_going = True
    maxBufferSize = 25
    if SAVE:
        savePath = r"C:\Users\Olivier Desclaux\OneDrive\Recherche\PICUPE\Results"
        savePathIR, savePathRGB, savePathDepth, savePathXSens = initLogging(savePath)

    awinda, mtwCallbacks, filenamesPCKL, devId, devIdUsed, nDevs, firmware_version, controlDev, Ports = mtw.initialiseAwinda(
        XSupdateRate,
        XSradioChannel,
        savePathXSens, maxBufferSize)
    # awinda.abortFlushing()
    # awinda.stopRecording()
    # mtw.stopAll(awinda, controlDev, Ports)
    # return True
    streams, _ = videostream.selectStreams(cameras, True)
    nStreams = len(streams)
    if nStreams == 2:
        flirStream, kinectStream = streams
    else:
        kinectStream = streams[0]

    input("Press enter to start recording...\n")

    print("Recording... Press enter to stop.")
    if not awinda.startRecording():
        raise RuntimeError("Failed to start recording. Aborting.")

    # startTime = xda.XsTimeStamp_nowMs()
    t1 = time.time()
    startTime = xda.XsTimeStamp_nowMs()

    now = datetime.now().time()  # time object
    print("now =", now)

    print("Laptop start time", t1)
    print("XS start time", startTime)
    while keep_going:
        cnt = 1

        # Read Cameras
        if nStreams == 2:
            kinectFrame = kinectStream.read()
            flirFrame = flirStream.read()
        else:
            kinectFrame = kinectStream.read()

        if SAVE:
            imnum = str(cnt)
            # threading.Thread(target=imageSaver, args=(flirFrame, imnum, "ir", savePathIR), daemon=True).start()
            # threading.Thread(target=imageSaver, args=(kinectFrame.depth, imnum, "depth", savePathDepth), daemon=True).start()
            # threading.Thread(target=imageSaver, args=(kinectFrame.color, imnum, "rgb", savePathRGB), daemon=True).start()
            # threading.Thread(target=writeXsens, args=(mtwCallbacks, filenamesPCKL), daemon=True).start()
            writeXsens(mtwCallbacks, filenamesPCKL)
            if msvcrt.kbhit():
                if msvcrt.getwche() == '\r':
                    print("Stopping recording")
                    break
        if VIZ:
            if kinectFrame.color is not None:
                cv2.imshow("Color", kinectFrame.color)
            if kinectFrame.transformed_depth is not None:
                cv2.imshow("Transformed Depth",
                           extractDepth.colorize(kinectFrame.transformed_depth, (None, 5000), cv2.COLORMAP_BONE))
            if nStreams == 2:
                cv2.imshow("FLIR", flirFrame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                cv2.destroyAllWindows()
                break

        if not VIZ and not SAVE:
            if msvcrt.kbhit():
                if msvcrt.getwche() == "\r":
                    break

        cnt += 1
        # if cnt == 30:
        #     break
    now = datetime.now().time()  # time object
    print("now =", now)
    t2 = time.time()
    startEnd = xda.XsTimeStamp_nowMs() - startTime
    print("Time of recording (s) according to XS: ", startEnd / 1000, "\n")
    print("Number of packets that should be acquired by each MTw:", round(XSupdateRate * startEnd / 1000))
    print("Acquisition time according to the laptop: ", round(t2 - t1, 4), " s")
    print("Number of frames: ", cnt)

    # Destroy everything
    if nStreams == 2:
        kinectStream.stop()
        flirStream.stop()
    else:
        kinectStream.stop()

    if not awinda.abortFlushing():
        print("Failed to abort flushing operation.")
    print("Stopping recording...\n")
    if not awinda.stopRecording():
        print("Failed to stop recording. Aborting.")

    # Writing the data from a pickle txt file to a readable txt file
    mtw.pickle2txt(devId, devIdUsed, nDevs, firmware_version, filenamesPCKL, XSupdateRate, savePathXSens, maxBuffer=maxBufferSize)

    print("\n Closing log file...")
    if not awinda.closeLogFile():
        raise RuntimeError("Failed to close log file. Aborting.")

    print("\nExiting program...")
    mtw.stopAll(awinda, controlDev, Ports)

    return True


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Select update rate with -rate argument.
    parser.add_argument('--XSrate', type=int, dest='XSupdateRate', default=40,
                        help='Select an update rate for the Xsens (40 - 120) Hz',
                        required=False)
    # Select radio channel with -radio argument
    parser.add_argument('--XSradio', type=int, dest='XSradioChannel', default=11,
                        help='Select a radio channel between 11 to 25 for the XSens communication',
                        required=False)
    args = parser.parse_args().__dict__

    XSupdateRate = args['XSupdateRate']
    XSradioChannel = args['XSradioChannel']

    main(XSupdateRate, XSradioChannel)
