import msvcrt
import traceback
from utils.myLogger import Logger, createSaveDirectories, copyCalibrationFiles
from utils.cleanUp import cleanUp, removeFlirFirstImage
from utils.process import Process, monitorProcesses
from utils.readerWriterClass import *
import gui.guiMain as gui


def main():
    logger = None  # Initialise logger object
    procs = []  # Initialise list of processes that will be used
    try:
        # Launch GUI to retreive user input
        guiConfig = gui.main()

        IMUs = guiConfig["IMUs"]

        saveName = guiConfig["Experiment Name"]

        calibrationDir = guiConfig["Calibration Directory"]

        resultsDir = guiConfig["Results Directory"]

        # # # # # # # # # # # # # # # # # # LOGGER # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens, savePathCalib = createSaveDirectories(
            resultsDir,
            saveName)
        copyCalibrationFiles(calibrationDir, savePathCalib)
        logger = Logger(newSavePath, "acquisition.log")
        listener = threading.Thread(target=logger.logWriter, daemon=True)
        listener.start()

        logger.log("Initialising processes... \n \n")
        readerProcesses = []
        writerProcesses = []
        keepGoing = multiprocessing.Value('i', 1)  # Shared value between readers to know when to stop recording
        systemStatus = multiprocessing.Array('i', 4)  # Index 0 for webcam, index 1 for kinect, index 2 for XSens,
        # index 3 is the global lock
        barrier = multiprocessing.Barrier(2)  # Barrier to make sure both cameras do their acquisition simultaneously

        # # # # # # # # # # # # # # FLIR # # # # # # # # # # # # # # #
        # Flir Process
        useFLIR = True  # For debug purposes only. Replaces FLir with the webcam. Set to True for Flir
        if not useFLIR:
            flirPort = findCameraPort("webcam")
            flir = FLIR("webcam", flirPort, keepGoing, systemStatus, logger, savePathFlir, barrier)

            flirReaderProcess = Process("Webcam Reader", logger, flir, target=flir.read)
            flirWriterProcess = Process("Webcam Writer", logger, flir, target=flir.write)

        else:
            flirPort = findCameraPort("flir")
            if flirPort == -1:
                raise Exception("Couldn't find Flir")
            flir = FLIR("flir", flirPort, keepGoing, systemStatus, logger, savePathFlir, barrier)

            flirReaderProcess = Process("FLIR Reader", logger, flir, target=flir.read)
            flirWriterProcess = Process("FLIR Writer", logger, flir, target=flir.write)

        readerProcesses.append(flirReaderProcess)
        writerProcesses.append(flirWriterProcess)

        # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
        # Kinect Object
        kinect = Kinect(keepGoing, systemStatus, logger, savePathRGB, savePathDepth, barrier)

        # Initialise processes
        kinectReaderProcess = Process("Kinect Reader", logger, kinect, target=kinect.read2)
        kinectRGBWriterProcess = Process("RGB Writer", logger, kinect, target=kinect.writeRGB)
        kinectDepthWriterProcess = Process("Depth Writer", logger, kinect, target=kinect.writeDepth)

        readerProcesses.append(kinectReaderProcess)
        writerProcesses.extend([kinectRGBWriterProcess, kinectDepthWriterProcess])

        # # # # # # # # # # # # # XSENS # # # # # # # # # # # # # # # # # # # # # #
        useXsens = True  # Set to false if you want acquisition without XSens data. In the gui, you will have to
        # check at least 1 IMU.
        if useXsens:
            xsens = XSens(IMUs, keepGoing, systemStatus, savePathXsens, logger)

            XsensProcess = Process("Xsens", logger, xsens, target=xsens.readAndWrite)
            readerProcesses.append(XsensProcess)

        else:
            systemStatus[2] = 1

        # Start the processes
        procs = readerProcesses + writerProcesses
        for proc in readerProcesses:
            proc.start()
        for proc in writerProcesses:
            proc.start()

        while True:  # Wait for all processes to finish initialisation
            if np.sum(systemStatus) < len(systemStatus) - 1:
                continue
            else:
                break

        # Now we leave some time to the user to correctly place IMUs on the patient
        logger.log("Set up all the IMUs. When ready, press s to start... \n")
        while True:
            if msvcrt.kbhit():
                if msvcrt.getwche() == 's':
                    systemStatus[3] = 1
                    break

        logger.log("Recording, press enter to stop \n")
        while True:
            monitorProcesses(procs)  # Continuously listen to processes, and make sure they aren't raising any error.
            # If one subprocess raises an error and terminates, all other processes will.
            if msvcrt.kbhit():
                if msvcrt.getwche() == '\r':
                    with keepGoing.get_lock():
                        keepGoing.value = 0  # Setting the keepGoing value to 0 indicates to the processes to stop
                        # recording
                    logger.log("\nStopping recording... \n")
                    break

        logger.log("Destroying writer processes...")
        for proc in writerProcesses:
            proc.join()
            proc.terminate()
        logger.log("Successfully destroyed writer processes... \n")

        # Making sure that all our queues are empty and that no frames are hidden (to avoid deadlocks)
        while not flir.queue.empty():
            flir.writingCounter = int(os.listdir(savePathFlir)[-2]) + 1  # QUICKFIX
            flir.write()
        while not kinect.RGBQueue.empty():
            kinect.writeRGB()
        while not kinect.DepthQueue.empty():
            kinect.writeDepth()

        logger.log("Destroying reader processes...")
        for proc in readerProcesses:
            proc.join()
            proc.terminate()
        logger.log("Successfully destroyed reader processes \n")

        logger.log("\n Acquisition was a success ! \n")
        logger.stop()  # Pass None to the logger queue to tell it to stop recording
        listener.join()  # Wait for the listening thread to stop writing

    except Exception as e:
        if logger:
            logger.log(traceback.format_exc())
            logger.stop()
            for proc in procs:
                proc.terminate()
        else:
            raise e

    # CLEAN UP
    while not os.path.exists(savePathFlir):
        time.sleep(5)

    removeFlirFirstImage(newSavePath)
    cleanUp(newSavePath, 5)

if __name__ == "__main__":
    main()
