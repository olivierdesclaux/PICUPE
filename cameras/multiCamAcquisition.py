import multiprocessing
import msvcrt
from utils import *



def main():
    print("Creating log directory... \n")
    savePath = r"C:\Users\picup\Desktop\PICUPE\sandbox\results"
    newSavePath, savePathRGB, savePathDepth, savePathWebcam = initLogging(savePath)

    print("Initialising processes... \n \n")
    readerProcesses = []
    writerProcesses = []
    keepGoing = multiprocessing.Value('i', 1)  # Shared value between readers to know when to stop recording
    cameraStatus = multiprocessing.Array('i', 2)  # Index 0 for webcam, index 1 for kinect
    kinectRGBQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectRGBWriter
    kinectDepthQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectDepthWriter
    webcamQueueSize = multiprocessing.Value('i', 0)  # Shared value between the WebcamReader and WebcamWriter

    # # # # # # # # # # # # # # WEBCAM # # # # # # # # # # # # # # # # # #

    # Webcam Queue
    webcamQueue = multiprocessing.Queue()
    WEBCAM = False
    # Webcam Process
    if WEBCAM:
        webcamPort = findCameraPort("webcam")
        webcamReaderProcess = multiprocessing.Process(target=webcamReader,
                                                      args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize, cameraStatus))
        webcamWriterProcess = multiprocessing.Process(target=webcamWriter,
                                                      args=(webcamQueue, savePathWebcam, keepGoing, webcamQueueSize))

    else:
        webcamPort = findCameraPort("flir")
        webcamReaderProcess = multiprocessing.Process(target=flirReader,
                                                      args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize, cameraStatus))
        webcamWriterProcess = multiprocessing.Process(target=flirWriter,
                                                      args=(webcamQueue, savePathWebcam, keepGoing, webcamQueueSize))
    readerProcesses.append(webcamReaderProcess)
    writerProcesses.append(webcamWriterProcess)

    # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
    # Kinect Queue
    kinectRGBQueue = multiprocessing.Queue()
    kinectDepthQueue = multiprocessing.Queue()
    # Kinect Process
    kinectReaderProcess = multiprocessing.Process(target=kinectReader,
                                                  args=(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize,
                                                        kinectDepthQueueSize, cameraStatus))
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


if __name__ == '__main__':
    main()
