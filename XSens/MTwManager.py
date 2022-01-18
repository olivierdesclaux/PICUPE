import xsensdeviceapi as xda
import os
import time
from XSens.MTwCallback import MTwCallback


class MTwManager:
    def __init__(self, IMUs, savePath, logger):
        self.logger = logger
        self.savePath = savePath
        self.IMUs = IMUs

        # Hardcoded variables
        self.updateRate = 60
        self.radioChannel = 13
        self.maxBufferSize = 5

        self.pickleDir, self.logfile = self.makeLogFile()
        self.Port = None
        self.control = None
        self.awinda = None
        self.deviceId = None
        self.firmwareVersion = None
        self.MTws = None
        self.callbacks = []
        self.filenamesPCKL = []

        self.IMUMappings = {
            "Head": "00B43B3E",
            "Torso": "00B43B4B",
            "Hips": "00B43CC1"
        }
        # self.IMUMappings = {
        #     "Head": "00B48765",
        #     "Torso": "00B4875B",
        #     "Hips": "00B48772",
        #     "Left Upper Arm": "00B4875C",
        #     "Right Upper Arm": "00B48770",
        #     "Left Forearm": "00B487B6",
        #     "Right Forearm": "00B4876D",
        #     "Left Hand": '00B48784',
        #     "Right Hand": "00B4875D",
        #     "Left Upper Leg": "00B48773",
        #     "Right Upper Leg": "00B4876F",
        #     "Left Lower Leg": "00B48769",
        #     "Right Lower Leg": "00B4876C",
        #     "Left Foot": "00B48768",
        #     "Right Foot": "00B48761",
        #     "Right Shoulder": "00B48760",
        #     "Left Shoulder": "00B4876E"
        #     }

    def initialise(self):
        self.logger.log("Creating an XsControl object...")
        self.control = xda.XsControl_construct()
        self.Port = self.findPort()
        self.deviceId = self.Port.deviceId()
        self.control.openPort(self.Port.portName(), self.Port.baudrate())

        self.awinda = self.control.device(self.deviceId)
        self.firmwareVersion = self.awinda.firmwareVersion()

        self.logger.log("Putting device into configuration mode...\n")
        if not self.awinda.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        self.configure()

        self.MTws = self.findIMUs()
        for imu in self.MTws:
            # add callback to handle data transmission to each MTw
            callback = MTwCallback(self.maxBufferSize)
            self.callbacks.append(callback)
            imu.addCallbackHandler(callback)

            # Create pickle file for writing data that will be communicated from the callback
            filePCKL = "PCKL_" + str(self.deviceId.toXsString()) + "_" + str(imu.deviceId()) + ".txt"
            filePCKL = os.path.join(self.pickleDir, filePCKL)
            open(filePCKL, 'wb')
            self.filenamesPCKL.append(filePCKL)

        try:
            self.awinda.gotoMeasurement()
        except Exception as e:
            raise e
            # raise RuntimeError("Could not put device into measurement mode. Aborting.")

        self.logger.log("Wait 10 seconds before starting data acquisition. XSens filters are warming up... \n")
        time.sleep(10)
        self.logger.log("IMUs are ready.\n")
        # Reset the yaw (XRM_Heading), pitch and roll (XRM_Alignment) angles to 0
        for imu in self.MTws:
            if not imu.resetOrientation(xda.XRM_Heading + xda.XRM_Alignment):
                self.logger.log("Could not reset the header for imu {}".format(str(imu.deviceId())))

        if not self.awinda.startRecording():
            raise RuntimeError("Failed to start recording. Aborting.")

    def makeLogFile(self):
        """
        Creates a logfile of type .mtb. Supposedly can be read when using the XSens app, but doesn't work.
        Online support shows that other people had this issue but never managed to find the solution.

        Returns
        -------
        pickleDir: string, path to the Pickle directory where all pickle files will be stored
        logfile: string, path to the .mtb logfile stored in the same directory as the pickle directory.
        """
        pickleDir = os.path.join(self.savePath, "MTw Pickle")
        if not os.path.isdir(pickleDir):
            os.mkdir(pickleDir)
        logfile = os.path.join(self.savePath, "logfile.mtb")
        return pickleDir, logfile

    def configure(self):
        """
        Sets up hyperparameters for our acquisition: update rate, radio channel, the saved data, etc.
        Returns
        -------
        None
        """
        # XSO_Orientation : keep the orientation data of MTw
        # XSO_Calibrate : compute calibrated data from raw data obtained by the MTw
        # XSO_RetainLiveData: keep the currently streaming data from MTw
        self.awinda.setOptions(xda.XSO_Orientation + xda.XSO_Calibrate + xda.XSO_RetainLiveData, 0)
        # self.logger.log("Appending log file ...")
        if self.awinda.createLogFile(self.logfile) != xda.XRV_OK:
            raise Exception("Failed to append a log file to the device. Aborting.")
        if not self.awinda.setUpdateRate(self.updateRate):
            raise Exception("Could not set up the update rate. Aborting.")
        try:
            self.awinda.enableRadio(self.radioChannel)
        except:
            raise Exception("The radio is still active, please undock the device from the computer and try again.")

    def findPort(self):
        """
        Scans all ports of the PC to find the port where the awinda station is connected.

        Returns
        -------
        Port: xda.XsPortInfo, contains info on the port.
        """
        self.logger.log("Detecting ports...\n")
        ports_Scan = xda.XsScanner_scanPorts(0, 100, True, True)
        Port = xda.XsPortInfo()
        for i in range(ports_Scan.size()):
            if ports_Scan[i].deviceId().isWirelessMaster() or ports_Scan[i].deviceId().isAwindaXStation():
                Port = ports_Scan[i]
                break
        if Port.empty():
            raise RuntimeError("Abort. No devices where detected.")
        else:
            return Port

    def findIMUs(self):
        """
        Uses the awinda station to find the imus specified in IMUs. Makes sure to keep only the imus from the IMUs
        variable, and not take into consideration other imus that might have been unintentionally turned on and
        connected
        to the station

        Parameters
        ----------
        awinda: xda.XsControlDevice, object representing the awinda station
        IMUs: list, names of IMUs that were selected by the user. Names correspond to the keys in the IMUMappings
        dictionary.
        logger: myLogger.logger, logging object

        Returns
        -------
        MTws: xda.XsDevicePtrArray, contains all the IMU xda objects
        """
        # Attaching XsDevice objects to each MTw wireless connected to the awinda station
        MTws = self.awinda.children()
        MTwsAddresses = [MTws[i].deviceId().toXsString() for i in range(len(MTws))]  # Associated i.p. addresses of
        # the IMUs

        # Waiting for all IMUs to be attached
        # 2 step process:
        # 1.We wait for all IMUs in the "IMUs" variable to be contained in MTws
        # 2. We then look in MTws to see of there are some IMUs that shouldn't be here, and remove them
        foundAllImus = False
        # Step 1
        while not foundAllImus:
            missingImus = []
            for imu in self.IMUs:
                imuAddress = self.name2Address(imu)
                if imuAddress in MTwsAddresses:
                    continue
                else:
                    missingImus.append(imu)
            if len(missingImus) > 0:
                self.logger.log("Missing : {}".format(missingImus))
                time.sleep(2)
                MTws = self.awinda.children()
                MTwsAddresses = [MTws[i].deviceId().toXsString() for i in range(len(MTws))]
            else:
                foundAllImus = True
        # Step 2:
        if len(MTws) > len(self.IMUs):
            #     This means that at least one extra IMU is here that shouldn't
            tempMTws = xda.XsDevicePtrArray()
            MTwsAddresses = [MTws[i].deviceId().toXsString() for i in range(len(MTws))]
            for imuAddress, imu in zip(MTwsAddresses, MTws):
                imuName = self.address2Name(imuAddress)
                if imuName in self.IMUs:  # We check that this IMU was really desired
                    tempMTws.insert(imu, 0)
            MTws = tempMTws

        return MTws

    def name2Address(self, imuName):
        """
        Converts an imu name to it's serial number, stored in self.IMUMappings
        Parameters
        ----------
        imuName: string, must exist in self.IMUMappings.keys()

        Returns
        -------
        imuAddress: str, imu serial number
        """
        return self.IMUMappings[imuName]

    def address2Name(self, imuAddress):
        """
        Converts an imu serial number to it's name, stored in self.IMUMappings
        Parameters
        ----------
        imuAddress: str, must be contained in self.IMUMappings.values()

        Returns
        -------
        imuName: str, imu name
        """
        for imuName in self.IMUMappings.keys():
            if self.IMUMappings[imuName] == imuAddress:
                return imuName

    def writeXsens(self):
        """
        Called at each timestamp, writes data stored in the IMU buffers. Writes for all IMUs.

        Parameters
        ----------
        mtwCallbacks: list, contains all MTwCallback classes associated to IMUs
        filenamesPCKL: list, contains paths to .txt files for pickling the IMU data in real time

        Returns
        -------
        None
        """
        for callback, filenamePCKL in zip(self.callbacks, self.filenamesPCKL):
            if callback.packetAvailable():
                callback.writeData(filenamePCKL)

    def stop(self):
        self.logger.log("\n Closing XSens log file...")
        try:
            self.awinda.closeLogFile()
        except:
            raise Exception("Failed to close log file. Aborting.")

        self.logger.log("Exiting program...")
        self.logger.log("Putting device back into configuration mode.")
        self.awinda.gotoConfig()
        self.awinda.disableRadio()

        self.logger.log("Closing ports...")
        self.control.closePort(self.Port)
        self.control.close()
        self.logger.log("Destroying XsControl object.")
        xda.XsControl.destruct(self.control)
        self.awinda = None

