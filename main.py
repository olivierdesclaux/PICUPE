import argparse
import msvcrt
import multiprocessing
import shutil
import traceback
from utils.myLogger import Logger, createSaveDirectories
import threading
from utils.process import Process, monitorProcesses
from calibrate import calibrateCamera
from stereocalibration import stereoCalibrate, rectify
from utils.readerWriterClass import *



def main(args):
    logger = None  # Initialise logger object
    procs = []  # Initialise list of processes that will be used
    try:
        nIMU = args['nIMU']
        FLIR = args['FLIR']
        saveName = args["saveName"]
        calibrationFile = args["calibrationFile"]
        calibInitialGrids = args["calibInitialGrids"]
        calibMinGrids = args["calibMinGrids"]
        kinectCalib = args["kinectCalib"]
        stereocalibInitialGrids = args["stereocalibInitialGrids"]
        if calibrationFile != "" and (not os.path.isdir(calibrationFile)):
            raise Exception("Invalid calibration directory.")
        resultsDir = args["resultsDir"]

        # # # # # # # # # # # # # # # # # # LOGGER # # # # # # # # # # # # # # # # # # # # # # # # # # # #
        newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens, savePathCalib = createSaveDirectories(
            resultsDir,
            saveName)
        logger = Logger(newSavePath, "customLog.log")
        listener = threading.Thread(target=logger.logWriter, daemon=True)
        listener.start()

        # # #  # # # # # # # # # # CALIBRATION # # # # # # # # # # # # # # # #
        if calibrationFile == '':
            # No calibration file is specified, this means we are going to do calibration and stereocalibration all over
            # again.
            flirCalibFlags = cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K3
            flirCalib = calibrateCamera("F", savePathCalib, flirCalibFlags, initialNumGrids=calibInitialGrids,
                                        minNumGrids=calibMinGrids)

            stereoFlags = cv2.CALIB_FIX_INTRINSIC
            stereoCalibrationFile = stereoCalibrate("FK", flirCalib, kinectCalib, savePathCalib, stereoFlags,
                                                    initialNumGrids=stereocalibInitialGrids)
            # stereoCalibrationFile = stereoCalibrate("KF", kinectCalib, flirCalib, savePathCalib, stereoFlags,
            #                                         initialNumGrids=20)

            # Compute rectification matrices
            rectify(savePathCalib)
        else:
            shutil.copytree(calibrationFile, savePathCalib, dirs_exist_ok=True)
            logger.log("Copying calibration files... \n")

        logger.log("Initialising processes... \n \n")
        readerProcesses = []
        writerProcesses = []
        keepGoing = multiprocessing.Value('i', 1)  # Shared value between readers to know when to stop recording
        systemStatus = multiprocessing.Array('i', 4)  # Index 0 for webcam, index 1 for kinect, index 2 for XSens,
        # index 3 is the global lock
        kinectRGBQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectRGBWriter
        kinectDepthQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectDepthWriter
        webcamQueueSize = multiprocessing.Value('i', 0)  # Shared value between the WebcamReader and WebcamWriter

        procs = []

        # # # # # # # # # # # # # # WEBCAM # # # # # # # # # # # # # # #
        # Webcam Queue
        webcamQueue = multiprocessing.Queue()
        # Webcam Process
        if not FLIR:
            webcamPort = findCameraPort("webcam")
            webcam = Webcam(webcamPort, webcamQueue, keepGoing, webcamQueueSize, systemStatus, logger,
                            savePathFlir)

            webcamReaderProcess = Process("Webcam Reader", logger, webcam, target=webcam.read)
            webcamWriterProcess = Process("Webcam Writer", logger, webcam, target=webcam.write)

        else:
            webcamPort = findCameraPort("flir")
            flir = FLIR(webcamPort, webcamQueue, keepGoing, webcamQueueSize, systemStatus, logger, savePathFlir)

            webcamReaderProcess = multiprocessing.Process(target=flir.read)
            webcamReaderProcess.name = "FLIR Reader"

            webcamWriterProcess = multiprocessing.Process(target=flir.write)
            webcamWriterProcess.name = "FLIR Writer"

        readerProcesses.append(webcamReaderProcess)
        writerProcesses.append(webcamWriterProcess)

        # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
        # Kinect Queue
        kinectRGBQueue = multiprocessing.Queue()
        kinectDepthQueue = multiprocessing.Queue()
        # Kinect Object
        kinect = Kinect(kinectRGBQueue, kinectDepthQueue, keepGoing, kinectRGBQueueSize, kinectDepthQueueSize,
                        systemStatus, logger, savePathRGB, savePathDepth)

        kinectReaderProcess = Process("Kinect Reader", logger, kinect, target=kinect.read)
        kinectRGBWriterProcess = Process("RGB Writer", logger, kinect, target=kinect.writeRGB)
        kinectDepthWriterProcess = Process("Depth Writer", logger, kinect, target=kinect.writeDepth)

        readerProcesses.append(kinectReaderProcess)
        writerProcesses.extend([kinectRGBWriterProcess, kinectDepthWriterProcess])

        # # # # # # # # # # # # # XSENS # # # # # # # # # # # # # # # # # # # # # #
        xsens = XSens(nIMU, keepGoing, systemStatus, savePathXsens, logger)

        XsensProcess = Process("Xsens", logger, xsens, target=xsens.readAndWrite)
        # XsensProcess = Process("Xsens", logger, xsens, target=readAndWrite, args=(xsens,))
        readerProcesses.append(XsensProcess)

        # Start the processes
        for proc in readerProcesses:
            proc.start()
        for proc in writerProcesses:
            proc.start()

        # Start monitoring processes
        procs = readerProcesses + writerProcesses
        # threading.Thread(target=monitorProcesses, args=(procs,))

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
            monitorProcesses(procs)
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
        while not webcamQueue.empty():
            webcamQueue.get()
        while not kinectRGBQueue.empty():
            kinectRGBQueue.get()
        while not kinectDepthQueue.empty():
            kinectDepthQueue.get()

        logger.log("Destroying reader processes...")
        for proc in readerProcesses:
            proc.join()
            proc.terminate()
        logger.log("Successfully destroyed reader processes \n")

        logger.log("\n Acquisition was a success ! \n")
        logger.stop()

    except Exception as e:
        if logger:
            logger.log(traceback.format_exc())
            logger.stop()
            for proc in procs:
                proc.terminate()
        else:
            raise e

if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--nIMU', type=int, dest='nIMU',
                        help='Select a number of IMUs for your experiment',
                        required=True)
    parser.add_argument('--FLIR', type=bool, dest='FLIR',
                        help='Select if you want to work with FLIR or Webcam',
                        required=False, default=False)
    parser.add_argument('--calibFile', type=str, dest='calibrationFile', help="Path to the calibration directory",
                        required=False, default='')
    parser.add_argument('--saveName', type=str, dest='saveName', help="Select experiment name", required=False,
                        default=datetime.now().strftime("%Y-%m-%d_%H-%M"))
    parser.add_argument('--resultsDir', type=str, dest='resultsDir', help="directory for storing results",
                        required=False, default=r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results")
    parser.add_argument('--calibInitialGrids', type=int, dest='calibInitialGrids', help="Number of initial grids for "
                                                                                        "FLIR calibration",
                        required=False, default=20)
    parser.add_argument('--calibMinGrids', type=int, dest='calibMinGrids', help="Minimum number of grids for FLIR "
                                                                                "calibration to succeed",
                        required=False, default=12)
    parser.add_argument('--stereocalibInitialGrids', type=int, dest='stereocalibInitialGrids', help="Number of "
                                                                                                    "initial grids "
                                                                                                    "for "
                                                                                                    "stereocalibration",
                        required=False, default=15)
    parser.add_argument('--kinectCalib', type=str, dest='kinectCalib', help="Path to kinect calibration file",
                        required=False,
                        default=r"C:\Users\Recherche\OneDrive - "
                                r"polymtl.ca\PICUPE\Recalage\Results\CalibKinectFactory5Dist.json")
    args = parser.parse_args().__dict__
    main(args)
