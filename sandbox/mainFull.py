import multiprocessing
import sys
import msvcrt
sys.path.append('../cameras')
sys.path.append('../XSens')
from utils import *
import MTwFunctions as mtw

def initLogger(savePath):
    now = datetime.now()
    dt_string = now.strftime("%Y-%m-%d_%H-%M")
    newSavePath = os.path.join(savePath, dt_string)
    savePathDepth = os.path.join(newSavePath, "depth")
    savePathRGB = os.path.join(newSavePath, "rgb")
    savePathFlir = os.path.join(newSavePath, "flir")
    savePathXsens = os.path.join(newSavePath, "XSens")

    if not os.path.isdir(os.path.join(savePath, dt_string)):
        os.mkdir(os.path.join(savePath, dt_string))
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)
        os.mkdir(savePathFlir)
        os.mkdir(savePathXsens)
    return newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens


def xsensReaderWriter(nIMUs, keepGoing, systemStatus, savePathXsens):
    XSupdateRate = 60
    XSradioChannel = 13
    maxBufferSize = 5
    awinda, mtwCallbacks, filenamesPCKL, devId, devIdUsed, nDevs, firmware_version, controlDev, Ports = mtw.initialiseAwinda(
        nIMUs,
        XSupdateRate,
        XSradioChannel,
        savePathXsens, maxBufferSize)
    systemStatus[2] = 1
    while np.sum(systemStatus) < len(systemStatus):
        continue

    print("XSens is Recording")
    while keepGoing.value:  # As long as the acquisition goes on !
        mtw.writeXsens(mtwCallbacks, filenamesPCKL)

    if not awinda.abortFlushing():
        print("Failed to abort flushing operation.")
    print("Stopping XSens recording...\n")
    if not awinda.stopRecording():
        print("Failed to stop recording. Aborting.")

    # Writing the data from a pickle txt file to a readable txt file
    print("Writing XSens PICKLE data to .txt ... \n")
    mtw.pickle2txt(devId, devIdUsed, nDevs, firmware_version, filenamesPCKL, XSupdateRate, savePathXsens,
                   maxBuffer=maxBufferSize)

    print("\n Closing XSens log file...")
    if not awinda.closeLogFile():
        raise RuntimeError("Failed to close log file. Aborting.")

    print("\nExiting program...")
    mtw.stopAll(awinda, controlDev, Ports)


def main():
    savePath = r"C:\Users\picup\Desktop\PICUPE\sandbox\results"
    newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens = initLogger(savePath)

    print("Initialising processes... \n \n")
    readerProcesses = []
    writerProcesses = []
    keepGoing = multiprocessing.Value('i', 1)  # Shared value between readers to know when to stop recording
    systemStatus = multiprocessing.Array('i', 3)  # Index 0 for webcam, index 1 for kinect, index 2 for XSens
    kinectRGBQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectRGBWriter
    kinectDepthQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectDepthWriter
    webcamQueueSize = multiprocessing.Value('i', 0)  # Shared value between the WebcamReader and WebcamWriter

    # Webcam Queue
    webcamQueue = multiprocessing.Queue()
    WEBCAM = False
    # Webcam Process
    if WEBCAM:
        webcamPort = findCameraPort("webcam")
        webcamReaderProcess = multiprocessing.Process(target=webcamReader,
                                                      args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize,
                                                            systemStatus))
        webcamWriterProcess = multiprocessing.Process(target=webcamWriter,
                                                      args=(webcamQueue, savePathFlir, keepGoing, webcamQueueSize))

    else:
        webcamPort = findCameraPort("flir")
        webcamReaderProcess = multiprocessing.Process(target=flirReader,
                                                      args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize,
                                                            systemStatus))
        webcamWriterProcess = multiprocessing.Process(target=flirWriter,
                                                      args=(webcamQueue, savePathFlir, keepGoing, webcamQueueSize))
    readerProcesses.append(webcamReaderProcess)
    writerProcesses.append(webcamWriterProcess)

    # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
    # Kinect Queue
    kinectRGBQueue = multiprocessing.Queue()
    kinectDepthQueue = multiprocessing.Queue()
    # Kinect Process
    kinectReaderProcess = multiprocessing.Process(target=kinectReader,
                                                  args=(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize,
                                                        kinectDepthQueueSize, systemStatus))
    kinectRGBWriterProcess = multiprocessing.Process(target=kinectRGBWriter,
                                                     args=(kinectRGBQueue, savePathRGB, keepGoing, kinectRGBQueueSize))
    kinectDepthWriterProcess = multiprocessing.Process(target=kinectDepthWriter,
                                                       args=(kinectDepthQueue, savePathDepth, keepGoing,
                                                             kinectDepthQueueSize))
    readerProcesses.append(kinectReaderProcess)
    writerProcesses.extend([kinectRGBWriterProcess, kinectDepthWriterProcess])

    # # # # # # # # # # # # # XSENS # # # # # # # # # # # # # # # # # # # # # #
    nIMUs = 8  # Number of IMUs we want to work with
    XsensProcess = multiprocessing.Process(target=xsensReaderWriter, args=(nIMUs, keepGoing, systemStatus, savePathXsens))
    readerProcesses.append(XsensProcess)

    for proc in readerProcesses:
        proc.start()
    pass
    for proc in writerProcesses:
        proc.start()

    while True:
        if msvcrt.kbhit():
            if msvcrt.getwche() == '\r':
                with keepGoing.get_lock():
                    keepGoing.value = 0  # Setting the keepGoing value to 0 indicates to the processes to stop recording
                print("\n Stopping recording... \n")
                break
    print("Destroying writer processes...")
    for proc in writerProcesses:
        proc.join()
        proc.terminate()
    print("Successfully destroyed writer processes... \n")

    # Making sure that all our queues are empty and that no frames are hidden (to avoid deadlocks)
    while not webcamQueue.empty():
        webcamQueue.get()
    while not kinectRGBQueue.empty():
        kinectRGBQueue.get()
    while not kinectDepthQueue.empty():
        kinectDepthQueue.get()

    print("Destroying reader processes...")
    for proc in readerProcesses:
        proc.join()
        proc.terminate()
    print("Successfully destroyed reader processes \n")

    print("\n Acquisition was a success ! \n")

if __name__ == "__main__":
    main()
