import argparse
import msvcrt
import multiprocessing
import shutil
import sys
import traceback
from utils import myLogger
import threading

sys.path.append('../cameras')
sys.path.append('../XSens')
sys.path.append('../Recalage')
from calibrate import calibrateCamera
from stereocalibration import stereoCalibrate, rectify
from old.readersWriters import *


def initLogger(resultsDir, saveName):
    newSavePath = os.path.join(resultsDir, saveName)
    savePathDepth = os.path.join(newSavePath, "depth")
    savePathRGB = os.path.join(newSavePath, "rgb")
    savePathFlir = os.path.join(newSavePath, "flir")
    savePathXsens = os.path.join(newSavePath, "XSens")
    savePathCalib = os.path.join(newSavePath, "calib")

    if not os.path.isdir(newSavePath):
        os.mkdir(newSavePath)
        os.mkdir(savePathDepth)
        os.mkdir(savePathRGB)
        os.mkdir(savePathFlir)
        os.mkdir(savePathXsens)
        os.mkdir(savePathCalib)
    else:
        raise ValueError("Specified saving parameters lead to an already existing directory.")

    return newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens, savePathCalib


def monitorProcesses(procs):
    """
    Function that will be threaded in the main process. Verifies that all child processes are running correctly. If
    one of the child processes fails, will terminate all child processes. Terminates in the end after all child
    processes have been terminated.

    Parameters
    ----------
    procs: list, list of processes

    Returns
    -------
    None
    """
    while any([proc.is_alive() for proc in procs]):
        for proc in procs:
            if proc.exception:
                for allOtherProc in procs:
                    allOtherProc.terminate()
    print("Finished monitoring child processes.")


class Process(multiprocessing.Process):
    """
    Class which returns child Exceptions to Parent.
    https://stackoverflow.com/a/33599967/4992248
    """

    def __init__(self, *args, **kwargs):
        multiprocessing.Process.__init__(self, *args, **kwargs)
        self._parent_conn, self._child_conn = multiprocessing.Pipe()
        self._exception = None
        self.loggerQueue = None

    def run(self):
        try:
            multiprocessing.Process.run(self)
            self._child_conn.send(None)
        except Exception as e:
            tb = traceback.format_exc()
            myLogger.log(self.loggerQueue, tb)
            self._child_conn.send((e, tb))
            # myLogger.log()
            # raise e  # You can still rise this exception if you need to

    def terminate(self):
        multiprocessing.Process.terminate(self)

    @property
    def exception(self):
        if self._parent_conn.poll():
            self._exception = self._parent_conn.recv()
        return self._exception


def main(args):
    nIMU = args['nIMU']
    FLIR = args['FLIR']
    saveName = args["saveName"]
    calibrationFile = args["calibrationFile"]
    calibInitialGrids = args["calibInitialGrids"]
    calibMinGrids = args["calibMinGrids"]
    kinectCalib = args["kinectCalib"]
    stereocalibInitialGrids = args["stereocalibInitialGrids"]
    if calibrationFile != "" and (not os.path.isdir(calibrationFile)):
        raise ValueError("Invalid calibration directory.")
    resultsDir = args["resultsDir"]

    # # # # # # # # # # # # # # # # # # LOGGER # # # # # # # # # # # # # # # # # # # # # # # # # # # #
    newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens, savePathCalib = initLogger(resultsDir,
                                                                                                     saveName)
    customLog = myLogger.initLogger(newSavePath, "customLog.log")
    loggerQueue = multiprocessing.Queue()
    listener = threading.Thread(target=myLogger.logWriter, args=(customLog, loggerQueue), daemon=True)
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
        myLogger.log(loggerQueue, "Copying calibration files... \n")

    # myLogger.log(loggerQueue, "############ \n")
    myLogger.log(loggerQueue, "Initialising processes... \n \n")
    readerProcesses = []
    writerProcesses = []
    keepGoing = multiprocessing.Value('i', 1)  # Shared value between readers to know when to stop recording
    systemStatus = multiprocessing.Array('i', 4)  # Index 0 for webcam, index 1 for kinect, index 2 for XSens,
    # index 3 is the global lock
    kinectRGBQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectRGBWriter
    kinectDepthQueueSize = multiprocessing.Value('i', 0)  # Shared value between the KinectReader and KinectDepthWriter
    webcamQueueSize = multiprocessing.Value('i', 0)  # Shared value between the WebcamReader and WebcamWriter

    procs = []
    try:
        # # # # # # # # # # # # # # WEBCAM # # # # # # # # # # # # # # #
        # Webcam Queue
        webcamQueue = multiprocessing.Queue()
        # Webcam Process
        if not FLIR:
            webcamPort = findCameraPort("webcam")
            # webcamReaderProcess = multiprocessing.Process(target=webcamReader,
            #                                               args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize,
            #                                                     systemStatus, loggerQueue))
            webcamReaderProcess = Process(target=webcamReader,
                                          args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize,
                                                systemStatus, loggerQueue))
            webcamReaderProcess.name = "Webcam Reader"
            webcamReaderProcess.loggerQueue = loggerQueue

            # webcamWriterProcess = multiprocessing.Process(target=webcamWriter,
            #                                               args=(webcamQueue, savePathFlir, keepGoing, webcamQueueSize,
            #                                                     loggerQueue))
            webcamWriterProcess = Process(target=webcamWriter,
                                          args=(webcamQueue, savePathFlir, keepGoing, webcamQueueSize,
                                                loggerQueue))
            webcamWriterProcess.name = "Webcam Writer"
            webcamWriterProcess.loggerQueue = loggerQueue

        else:
            webcamPort = findCameraPort("flir")
            webcamReaderProcess = multiprocessing.Process(target=flirReader,
                                                          args=(webcamPort, webcamQueue, keepGoing, webcamQueueSize,
                                                                systemStatus, loggerQueue))
            webcamReaderProcess.name = "FLIR Reader"
            webcamWriterProcess = multiprocessing.Process(target=flirWriter,
                                                          args=(webcamQueue, savePathFlir, keepGoing, webcamQueueSize,
                                                                loggerQueue))

            webcamWriterProcess.name = "FLIR Writer"
        readerProcesses.append(webcamReaderProcess)
        writerProcesses.append(webcamWriterProcess)

        # # # # # # # # # # # # # KINECT # # # # # # # # # # # # # # # # # # # # # #
        # Kinect Queue
        kinectRGBQueue = multiprocessing.Queue()
        kinectDepthQueue = multiprocessing.Queue()
        # Kinect Process
        # kinectReaderProcess = multiprocessing.Process(target=kinectReader,
        #                                               args=(
        #                                                   kinectRGBQueue, kinectDepthQueue, keepGoing,
        #                                                   kinectRGBQueueSize,
        #                                                   kinectDepthQueueSize, systemStatus, loggerQueue))
        kinectReaderProcess = Process(target=kinectReader,
                                      args=(
                                          kinectRGBQueue, kinectDepthQueue, keepGoing,
                                          kinectRGBQueueSize,
                                          kinectDepthQueueSize, systemStatus, loggerQueue))
        kinectReaderProcess.name = "Kinect Reader"
        kinectReaderProcess.loggerQueue = loggerQueue

        # kinectRGBWriterProcess = multiprocessing.Process(target=kinectRGBWriter,
        #                                                  args=(
        #                                                      kinectRGBQueue, savePathRGB, keepGoing, kinectRGBQueueSize,
        #                                                      loggerQueue))
        kinectRGBWriterProcess = Process(target=kinectRGBWriter,
                                         args=(
                                             kinectRGBQueue, savePathRGB, keepGoing, kinectRGBQueueSize, loggerQueue))
        kinectRGBWriterProcess.name = "RGB Writer"
        kinectRGBWriterProcess.loggerQueue = loggerQueue

        kinectDepthWriterProcess = Process(target=kinectDepthWriter,
                                           args=(kinectDepthQueue, savePathDepth, keepGoing,
                                                 kinectDepthQueueSize, loggerQueue))
        kinectDepthWriterProcess.name = "Depth Writer"
        kinectDepthWriterProcess.loggerQueue = loggerQueue

        readerProcesses.append(kinectReaderProcess)
        writerProcesses.extend([kinectRGBWriterProcess, kinectDepthWriterProcess])

        # # # # # # # # # # # # # XSENS # # # # # # # # # # # # # # # # # # # # # #
        # XsensProcess = multiprocessing.Process(target=xsensReaderWriter,
        #                                        args=(nIMU, keepGoing, systemStatus, savePathXsens, loggerQueue))
        XsensProcess = Process(target=xsensReaderWriter,
                               args=(nIMU, keepGoing, systemStatus, savePathXsens, loggerQueue))
        XsensProcess.name = "Xsens"
        XsensProcess.loggerQueue = loggerQueue
        readerProcesses.append(XsensProcess)

        # Start the processes
        for proc in readerProcesses:
            proc.start()
        # pass
        for proc in writerProcesses:
            proc.start()

        # Start monitoring processes
        procs = readerProcesses + writerProcesses
        threading.Thread(target=monitorProcesses, args=(procs,))

        while True:  # Wait for all processes to finish initialisation
            if np.sum(systemStatus) < len(systemStatus) - 1:
                continue
            else:
                break

        # Now we leave some time to the user to correctly place IMUs on the patient
        myLogger.log(loggerQueue, "Set up all the IMUs. When ready, press s to start... \n")
        while True:
            if msvcrt.kbhit():
                if msvcrt.getwche() == 's':
                    systemStatus[3] = 1 # We set the system Status
                    break

        myLogger.log(loggerQueue, "Recording, press enter to stop \n")

        while True:
            if msvcrt.kbhit():
                if msvcrt.getwche() == '\r':
                    with keepGoing.get_lock():
                        keepGoing.value = 0  # Setting the keepGoing value to 0 indicates to the processes to stop recording
                    myLogger.log(loggerQueue, "\nStopping recording... \n")
                    break

        myLogger.log(loggerQueue, "Destroying writer processes...")
        for proc in writerProcesses:
            proc.join()
            proc.terminate()
        myLogger.log(loggerQueue, "Successfully destroyed writer processes... \n")

        # Making sure that all our queues are empty and that no frames are hidden (to avoid deadlocks)
        while not webcamQueue.empty():
            webcamQueue.get()
        while not kinectRGBQueue.empty():
            kinectRGBQueue.get()
        while not kinectDepthQueue.empty():
            kinectDepthQueue.get()

        myLogger.log(loggerQueue, "Destroying reader processes...")
        for proc in readerProcesses:
            proc.join()
            proc.terminate()
        myLogger.log(loggerQueue, "Successfully destroyed reader processes \n")

        loggerQueue.put_nowait(None)

        myLogger.log(loggerQueue, "\n Acquisition was a success ! \n")

    except Exception as e:
        myLogger.log(loggerQueue, traceback.format_exc())
        loggerQueue.put_nowait(None)
        for proc in procs:
            proc.terminate()


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
                        default=r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\CalibKinectFactory5Dist.json")
    args = parser.parse_args().__dict__
    main(args)
