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
    queues = []
    keepGoing = multiprocessing.Value('i', 1)  # Shared value between readers to know when to stop recording
    cameraStatus = multiprocessing.Array('i', 2)  # Index 0 for webcam, index 1 for kinect
    kinectRGBQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectRGBWriter
    kinectDepthQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectDepthWriter
    webcamQueueSize = multiprocessing.Value('i', 0)  # Shared value between the WebcamReader and WebcamWriter

    # # # # # # # # # # # # # # WEBCAM # # # # # # # # # # # # # # # # # #
    # Webcam Queue
    webcamQueue = multiprocessing.Queue()
    queues.append(webcamQueue)
    # Webcam Process
    # webcamReaderProcess = multiprocessing.Process(target=webcamReader,
    #                                               args=(webcamQueue, keepGoing, webcamQueueSize, cameraStatus))
    # webcamWriterProcess = multiprocessing.Process(target=webcamWriter2,
    #                                               args=(webcamQueue, savePathWebcam, keepGoing, webcamQueueSize))
    webcamReaderProcess = multiprocessing.Process(target=flirReader,
                                                  args=(webcamQueue, keepGoing, webcamQueueSize, cameraStatus))
    webcamWriterProcess = multiprocessing.Process(target=flirWriter2,
                                                  args=(webcamQueue, savePathWebcam, keepGoing, webcamQueueSize))
    readerProcesses.append(webcamReaderProcess)
    writerProcesses.append(webcamWriterProcess)

    # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
    # Kinect Queue
    kinectRGBQueue = multiprocessing.Queue()
    kinectDepthQueue = multiprocessing.Queue()
    queues.append(kinectRGBQueue)
    queues.append(kinectDepthQueue)
    # Kinect Process
    kinectReaderProcess = multiprocessing.Process(target=kinectReader,
                                                  args=(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize,
                                                        kinectDepthQueueSize, cameraStatus))
    kinectRGBWriterProcess = multiprocessing.Process(target=kinectRGBWriterVideo,
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


if __name__ == '__main__':
    main()
