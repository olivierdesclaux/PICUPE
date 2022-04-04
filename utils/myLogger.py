import logging
import multiprocessing
import sys
import traceback
import os
import shutil

class Logger:
    def __init__(self, savePath, name):
        self.savePath = savePath
        self.logger = self.initLogger(self.savePath, name)
        self.loggerQueue = multiprocessing.Queue()

    def initLogger(self, savePath, name):
        """
        Parameters
        ----------
        savePath: str, directory to save the log file
        name: str, log file name, e.g. "myLogFile.log"

        Returns
        -------
        logger: object of class logging.
        """
        # Initialise logging object
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        # create file handler which logs even debug messages
        fh = logging.FileHandler(os.path.join(savePath, name))
        fh.setLevel(logging.INFO)
        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.ERROR)
        # create formatter and add it to the handlers
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        ch.setFormatter(formatter)
        # add the handlers to the logger
        logger.addHandler(fh)
        logger.addHandler(ch)

        return logger

    def logWriter(self):
        """
        Function that will actually write to file. Will be threaded in main process. Reads data from the loggerQueue,
        and outputs it to the self.savePath file

        Returns
        -------

        """
        while True:
            try:
                record = self.loggerQueue.get()
                if record is None:
                    break
                print(record)
                self.logger.info(record)
            except Exception:
                print('Whoops! Problem:', file=sys.stderr)
                traceback.print_exc(file=sys.stderr)
                sys.exit()

    def log(self, msg):
        """
        Puts a message in the self.loggerQueue, alongside the associated process this message comes froms
        Parameters
        ----------
        msg: str, the message

        Returns
        -------
        None
        """
        currentProcess = str(multiprocessing.current_process().name) + ": "  # Get current process name
        self.loggerQueue.put(currentProcess + msg)

    def stop(self):
        """
        By putting a None in the queue, the logging thread will stop as soon as it sees the None

        Returns
        -------
        None
        """
        self.loggerQueue.put_nowait(None)


def createSaveDirectories(resultsDir, saveName):
    """
    Generates the main directory and subdirectories for storing all data. Architecture is as follows:
    - resultsDir
        - saveName
            - calib
            - depth
            - flir
            - rgb
            - XSens

    Parameters
    ----------
    resultsDir: str, path to the main directory for saving data
    saveName: str, subdirectory name

    Returns
    -------
    newSavePath: path to saveName dir
    savePathRGB: path to rgb dir
    savePathDepth: path to depth dir
    savePathFlir: path to flir dir
    savePathXsens: path to Xsens dir
    savePathCalib: path to calib dir
    """
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
        raise Exception("Specified saving parameters lead to an already existing directory.")

    return newSavePath, savePathRGB, savePathDepth, savePathFlir, savePathXsens, savePathCalib

def copyCalibrationFiles(calibDir, savePathCalib):
    """
    Copies calibration parameters from a previous experiment to the directory of the specified new experiment.
    Checks that we are only copying the calibration files, and not the entire experiment data.
    Parameters
    ----------
    calibDir: string, path to the experiment directory whose calibration parameters we want to copy
    savePathCalib: string, path to the "calib" directory where we want to store the copied calibration parameters

    Returns
    -------
    True, if succeeds. If calibration data is missing in the calibDir, raises an exception.

    """
    files = os.listdir(calibDir)
    if ("calib" in files) and os.path.isdir(os.path.join(calibDir, "calib")):
        calibDir = os.path.join(calibDir, "calib")
    calibrationFiles = ['ErrorCharts.png', 'flirCalib.json', 'stereo.json', "calibParameters.json", "customLog.log"]
    if not set(calibrationFiles) == set(os.listdir(calibDir)):
        for x in calibrationFiles:
            if x not in os.listdir(calibDir):
                raise Exception("Missing {} in specified calibration directory".format(x))
        raise Exception("Specified calibration directory must contain exclusively {}".format(calibrationFiles))
    else:
        shutil.copytree(calibDir, savePathCalib, dirs_exist_ok=True)

