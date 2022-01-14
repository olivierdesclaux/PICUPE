import numpy as np
import xsensdeviceapi as xda
import sys
import re
import os
import time
import pickle
import datetime
from threading import Lock

sys.path.append("../sandbox")
import myLogger

IMUMappings = {
    'HAND_L': '00B48784',
    'fARM_L': '00B487B6',
    'SHOU':'00B48760',
    'FOOT':'00B48761',
    'fARM_R': '00B4876D',
    'HAND_R': '00B4875D',
    'PELV': '00B48772',
    'PROP 1': '00B487B7',
    'uLEG_R':'00B4876F',
    'uARM_L': '00B4875C',
    'uARM_R': '00B48770',
    'STERN': '00B4875B',
    'uLEG_L': '00B48773',
    'LLEG_L': '00B48769',
    'LLEG_R': '00B4876C',
    'FOOT_L': '00B48768',
    'HEAD': '00B48765',
    'SHOU_L': '00B4876E',
    'Head': '00B43B4B',
    'Torso': '00B43CC1'
}

def name2Adress(imuName):
    return IMUMappings[imuName]

def adress2Name(imuAdress):
    for imuName in IMUMappings.keys():
        if IMUMappings[imuName] == imuAdress:
            return imuName

def stopAll(device, control, Ports, logger):
    """Quits program cleanly and closes all ports of XSens devices
    Parameters
    ----------
    device : class XsDevice object
        Object that stores all the functions of a XSens device
    control : class XsControl object
        Object that stores all the functions of XsControl
    Ports : class XsPortInfo object
        Object that stores all the functions of XsPorts
    Returns
    -------
    None.
    """
    logger.log("Putting device back into configuration mode.")
    device.gotoConfig()
    device.disableRadio()

    logger.log("Closing ports...")
    control.closePort(Ports)
    control.close()
    logger.log("Destroying XsControl object.")
    xda.XsControl.destruct(control)


def checkConnectedSensors(devIdAll, children, control, device, Ports, logger):
    """Verifies the connectivity of the MTw
    Parameters
    ----------
    devIdAll : list
        List of the children devices IDs
    children: list
        List of de MTw class XsDevice objects
    device : class XsDevice object
        Object that stores all the functions of a XSens device
    control : class XsControl object
        Object that stores all the functions of XsControl
    Ports : class XsPortInfo object
        Object that stores all the functions of XsPorts
    Returns
    -------
    devicesUsed : list
        List of MTw class XsDevice objects of accepted devices
    devIdUsed : list
        List of MTw IDs of accepted devices
    nDevs : int
        Number of accepted devices
    """
    childUsed = np.full(np.shape(children), False).tolist()
    if children.empty():
        stopAll(device, control, Ports)
        raise RuntimeError("No MTw devices found. Aborting.")
    else:
        for i in range(len(children)):
            if children[i].connectivityState() == xda.XCS_Wireless:
                childUsed[i] = True

        logger.log("\n Rejected devices: ")
        rejects = np.array(devIdAll)[[not elem for elem in childUsed]].tolist()
        for i in range(len(rejects)):
            index = devIdAll.index(rejects[i])
            children[index].requestBatteryLevel()
            time.sleep(0.1)
            logger.log("%d - " % index + "%s" % rejects[i] + " || battery percentage: " + " %d" % children[
                index].batteryLevel())

        logger.log("\n Accepted devices: ")
        accepted = np.array(devIdAll)[childUsed].tolist()
        for acc in accepted:
            index = devIdAll.index(acc)
            mt = children[index]
            mt.requestBatteryLevel()
            time.sleep(0.1)
            logger.log("%d - " % index + "%s" % accepted[i] + " || battery percentage: " + " %d" % mt.batteryLevel())

        if sum(childUsed) == 0:
            stopAll(device, control, Ports)
            raise RuntimeError("No MTw devices found. Aborting.")
    devicesUsed = np.array(children)[childUsed].tolist()
    devIdUsed = np.array(devIdAll)[childUsed].tolist()
    nDevs = sum(childUsed)

    return devicesUsed, devIdUsed, nDevs


def pickle2txt(devId, devIdUsed, nDevs, firmware_version, filenames, updateRate, savePath, maxBuffer, logger):
    """Write readable txt files from pickle txt files and applies interpolation and quick fixes on missing packets
    Parameters
    ----------
    devId : string
        ID of the awinda station
    devIdUsed: list strings
        List of the connected MTw devices IDs
    nDevs : int
        Number of the connected MTw devices
    firmware_version : string
        Firmware version uploaded to the devices
    filenames : list strings
        List of the directories of the pickle txt files
    updateRate : int
        Update rate of the data packets of the devices
    maxBuffer : int
        Maximum data packets stored in the buffer of devices
    Returns
    -------
    None.
    """
    f_path = os.path.join(savePath, "MTw data")
    if not os.path.isdir(f_path):
        os.mkdir(f_path)

    nPacketFinal = []
    nPacketInitial = []
    for n in range(nDevs):
        dataPackets = []
        with (open(filenames[n], 'rb')) as openfile:
            while True:
                try:
                    dataPackets.append(pickle.load(openfile))
                except EOFError:
                    break
        packetCounter = [item[0] for item in dataPackets][maxBuffer - 1:]
        nPacketFinal.append(packetCounter[-1])
        nPacketInitial.append(packetCounter[0])
    finalPacket = int(np.median(nPacketFinal))  # Quick fix for all devices to start at the same data packet counter
    initialPacket = int(np.median(nPacketInitial))  # Quick fix for all devices to end at the same data packet counter

    for n in range(nDevs):
        dataPackets = []
        filename = "MT_" + str(devId.toXsString()) + "_" + str(devIdUsed[n]) + ".txt"
        with (open(filenames[n], 'rb')) as openfile:
            while True:
                try:
                    dataPackets.append(pickle.load(openfile))
                except EOFError:
                    break
        packetCounter = [item[0] for item in dataPackets][maxBuffer - 1:]
        acceleration = [item[1] for item in dataPackets][maxBuffer - 1:]
        orientationMatrix = [item[2] for item in dataPackets][maxBuffer - 1:]
        timeOfArrival = [item[3] for item in dataPackets[maxBuffer - 1:]]
        # timeOfArrival = [item[3] for item in dataPackets]
        numberPacketsRaw = len(packetCounter)

        # Remove the packet counter wrapper
        packetCounter = removeResetCounter(packetCounter)

        # Interpolate missing data packets
        packetCounter, acceleration, orientationMatrix = interpolateData(packetCounter, acceleration, orientationMatrix,
                                                                         timeOfArrival)

        # Make sure all packets start at the same packet counter
        packetCounter, acceleration, orientationMatrix, timeOfArrival = setPacketStart(packetCounter, acceleration,
                                                                                       orientationMatrix, timeOfArrival,
                                                                                       initialPacket)

        # Make sure all packets end at the same packet counter
        packetCounter, acceleration, orientationMatrix, timeOfArrival = setPacketEnd(packetCounter, acceleration,
                                                                                     orientationMatrix, timeOfArrival,
                                                                                     finalPacket)

        # Get number of interpolated data packets
        numberPackets = len(packetCounter)
        packetsMissing = abs(numberPackets - numberPacketsRaw)

        # Estimate the utc time of data packets from time of arrival
        # timeMeasurement = timeOfArrival2timeMeasurement(timeOfArrival, updateRate, numberPackets)
        timeMeasurement = timeOfArrival

        filepath = os.path.join(f_path, filename)
        file_txt = open(filepath, "w")
        file_txt.write("// Start Time: Unknown: \n")
        file_txt.write("// Update Rate: " + str(float(updateRate)) + "Hz \n")
        file_txt.write("// Filter Profile: human (46.1) \n")
        file_txt.write("// Firmware Version: " + str(firmware_version.toXsString()) + "\n")
        file_txt.write("// Option Flags: AHS Disabled ICC Disabled \n")
        file_txt.write(
            "packetCounter\tSampleTimeFine\tYear\tMonth\tDay\tSecond\tUTC_Nano\tUTC_Year\tUTC_Month\tUTC_Day"
            "\tUTC_Hour\tUTC_Minute\tUTC_Second\tUTC_Valid\tAcc_X\tAcc_Y\tAcc_Z\tMat[1][1]\tMat[2][1]\tMat[3]["
            "1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3] \n")

        for i in range(len(packetCounter)):
            file_txt.write(str(packetCounter[i]) + "\t\t\t\t\t\t\t\t")
            date = timeMeasurement[i].split(' ')[0]
            temps = timeMeasurement[i].split(' ')[1]
            file_txt.write(date.split('/')[0] + "\t")
            file_txt.write(date.split('/')[1] + "\t")
            file_txt.write(date.split('/')[2] + "\t")

            file_txt.write(temps.split(':')[0] + "\t")
            file_txt.write(temps.split(':')[1] + "\t")
            file_txt.write(temps.split(':')[2] + "\t")

            matrix = orientationMatrix[i].reshape(9, )
            for k in range(3):
                file_txt.write('{:.6f}'.format(round(acceleration[i][k], 6)) + "\t")
            for j in range(9):
                file_txt.write('{:.6f}'.format(round(matrix[j], 6)) + "\t")
            file_txt.write("\n")
        file_txt.close()

        logger.log("{} Number of data packets: {} with {} packets interpolated, ({} %)".format(devIdUsed[n],
                                                                                               len(packetCounter),
                                                                                               packetsMissing, round(
                packetsMissing / len(packetCounter) * 100, 4)))


def setPacketStart(packetCounter, acceleration, orientationMatrix, timeOfArrival, initialPacket):
    # If we start after the reference initial packet, add packets iteratively
    while packetCounter[0] > initialPacket:
        packetCounter.insert(0, packetCounter[0] - 1)
        acceleration.insert(0, acceleration[0])
        orientationMatrix.insert(0, orientationMatrix[0])
        timeOfArrival.insert(0, timeOfArrival[0])

    # If we start before the reference initial packet, remove them iteratively.
    while packetCounter[0] < initialPacket:
        packetCounter.pop(0)
        acceleration.pop(0)
        orientationMatrix.pop(0)
        timeOfArrival.pop(0)

    return packetCounter, acceleration, orientationMatrix, timeOfArrival,


def setPacketEnd(packetCounter, acceleration, orientationMatrix, timeOfArrival, finalPacket):
    # If missing final packets, add them iteratively
    while packetCounter[-1] < finalPacket:
        packetCounter.append(packetCounter[-1] + 1)
        acceleration.append(acceleration[-1])
        orientationMatrix.append(orientationMatrix[-1])
        timeOfArrival.append(timeOfArrival[-1])

    # If extra final packets, remove them iteratively
    while packetCounter[-1] > finalPacket:
        packetCounter.pop()
        acceleration.pop()
        orientationMatrix.pop()
        timeOfArrival.pop()

    return packetCounter, acceleration, orientationMatrix, timeOfArrival


def timeOfArrival2timeMeasurement(timeOfArrival, updateRate):
    """Estimates the utc time of data packets sent by the MTw to the awinda station
    Parameters
    ----------
    timeOfArrival : list strings
        List of utc times that the data packets where received by the awinda station
    updateRate : int
        Update rate of the data packets of the devices
    numberPackets : int
        number of data packets
    Returns
    -------
     timeMeasurement : list strings
        List of estimated utc times of the data packets
    """
    nPackets = len(timeOfArrival)
    t0 = time.strptime(timeOfArrival[0][11:-1].split('.')[0], '%H:%M:%S')
    t0_s = datetime.timedelta(hours=t0.tm_hour, minutes=t0.tm_min, seconds=t0.tm_sec).total_seconds()
    t0_ms = int(timeOfArrival[0][11:].split('.')[1]) / 1000

    t = time.strptime(timeOfArrival[-1][11:-1].split('.')[0], '%H:%M:%S')
    t_s = datetime.timedelta(hours=t0.tm_hour, minutes=t0.tm_min, seconds=t0.tm_sec).total_seconds()
    t_ms = int(timeOfArrival[-1][11:].split('.')[1]) / 1000

    delta_t_dot = round((t_s + t_ms) - (t0_s + t0_ms), 3)
    delta = round(np.abs(delta_t_dot - nPackets / updateRate), 3)

    timeMeasurement = []
    temps_init = t0_s + t0_ms - delta
    for n in range(nPackets):
        seconds = round(temps_init + n / 60, 3)
        m_s = round(seconds % 1, 3)
        ty_res = time.gmtime(seconds)
        temps = time.strftime("%H:%M:%S", ty_res)
        temps = temps + str(m_s)[1:]
        timeMeasurement.append(timeOfArrival[0].split(' ')[0] + ' ' + temps)

    return timeMeasurement


def removeResetCounter(packetCounter, maxWrap=65535):
    """QUICKFIX to remove the reset counter at packet count # 65535
    Parameters
    ----------
    packetCounter : list int
        List of packet count for the data packets
    maxWrap : int
        The number that the packet count reset to 0
    Returns
    -------
     packetCounter : list int
        List of packet counter without any resets at count # 65535
    """
    index = []
    for i in range(len(packetCounter)):
        if packetCounter[i] // 10 == 0 and packetCounter[i - 1] // 10 != 0:
            index.append(i)

    if index:
        for n in index:
            for i in range(n, len(packetCounter)):
                packetCounter[i] = packetCounter[i] + maxWrap + 1
    return packetCounter


def interpolateData(packetCounter: list, accelerationMatrix: list, orientationMatrix: list, timeOfArrival: list) -> [
    list, list, list, list]:
    """QUICKFIX to interpolate missing data packets
    Parameters
    ----------
    packetCounter : list int
        List of packet count for the data packets
    accelerationMatrix : list numpy array [1,3]
        List of the calibrated XYZ accelerations of the data packets
    orientationMatrix : list numpy array [3,3]
        List of the rotation matrix of the data packets
    Returns
    -------
     packetCounter :
        List of packet counter with the interpolated missing data packets
    accelerationMatrix : list numpy array [1,3]
        List of the calibrated XYZ accelerations with the interpolated accelerations
    orientationMatrix : list numpy array [3,3]
        List of the rotation matrix with the interpolated rotation matrix
    """
    i = 0
    while True:
        if packetCounter[i + 1] != packetCounter[i] + 1:
            # Compute mean values
            acc = np.mean([accelerationMatrix[i], accelerationMatrix[i + 1]], 0)
            orientation = np.mean([orientationMatrix[i], orientationMatrix[i + 1]], 0)
            t1 = datetime.datetime.strptime(timeOfArrival[i], '%Y/%m/%d %H:%M:%S.%f')
            t2 = datetime.datetime.strptime(timeOfArrival[i + 1], '%Y/%m/%d %H:%M:%S.%f')
            t = (t1 + 0.5 * (t2 - t1)).strftime('%Y/%m/%d %H:%M:%S.%f')  # Compute mean timestamp
            t = t[:-3]  # datetime obj has precision to nanosec, but we stop at micro (3 digits instead of 6)
            # Insert them at the appropriate indices
            packetCounter.insert(i + 1, packetCounter[i] + 1)
            accelerationMatrix.insert(i + 1, acc)
            orientationMatrix.insert(i + 1, orientation)
            timeOfArrival.insert(i + 1, t)
        i += 1

        if packetCounter[-1] - packetCounter[i] == 1:
            break

    return packetCounter, accelerationMatrix, orientationMatrix


class MTwCallback(xda.XsCallback):
    """XsCallback class that manages data packets transmission between the MTw devices and the awinda station while on
    recording mode. Also, we can add events and flags to enhance data transmission of packets.
    Parameters
    ----------
    xda.XsCallback : class XsCallback object
        Constructs an XSens callback class
    Returns
    -------
    None.
    """

    def __init__(self, maxBufferSize):
        """Initialise the MTwCallback
        Parameters
        ----------
        maxBufferSize : int
            The number of data packets a device can store before sending them to the awinda station via a flushing
            operation
        Returns
        -------
        None.
        """
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = maxBufferSize
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        """Event that verifies that a data packet was receive by the device before transmission to the master device
        Parameters
        ----------
        None.
        Returns
        -------
        res : bool
            True is the a data packet is available on the buffer of the device
        """
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        # assert (packet is not 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()

    def writeData(self, filename):
        """Event that writes the data packet receive by the device to a pickle txt file
        Parameters
        ----------
        filename : string
            directory of the pickle txt file in MTw Pickle folder
        Returns
        -------
        None.
        """
        self.m_lock.acquire()
        # assert (len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        with open(filename, "ab") as file_handle:
            pickle.dump((oldest_packet.packetCounter(), oldest_packet.calibratedAcceleration(),
                         oldest_packet.orientationMatrix(),
                         oldest_packet.timeOfArrival().utcToLocalTime().toXsString().__str__()),
                        file_handle)
        self.m_lock.release()


def initialiseAwinda(IMUs, updateRate, radioChannel, savePath, maxBufferSize, logger):
    MTw_pickle = os.path.join(savePath, "MTw Pickle")
    if not os.path.isdir(MTw_pickle):
        os.mkdir(MTw_pickle)
    logFileName = os.path.join(savePath, "logfile.mtb")

    # Extract the XsDeviceApi version used

    logger.log("Creating a XsControl object...")
    controlDev = xda.XsControl_construct()
    # assert (controlDev is not 0)

    xdaVersion = xda.XsVersion()
    xda.xdaVersion(xdaVersion)
    logger.log("Using the following XDA version: %s \n" % xdaVersion.toXsString())

    logger.log("Detecting ports...\n")
    ports_Scan = xda.XsScanner_scanPorts(0, 100, True, True)

    # Detecting an awinda station
    Ports = xda.XsPortInfo()
    for i in range(ports_Scan.size()):
        if ports_Scan[i].deviceId().isWirelessMaster() or ports_Scan[i].deviceId().isAwindaXStation():
            Ports = ports_Scan[i]
            break

    if Ports.empty():
        raise RuntimeError("Abort. No devices where detected.")

    devId = Ports.deviceId()
    logger.log(" Found awinda station with: ")
    logger.log(" Device ID: %s" % devId.toXsString())
    logger.log(" Port name: %s" % Ports.portName())
    logger.log(" Port baudrate: %s" % Ports.baudrate())

    logger.log("Opening port...")
    if not controlDev.openPort(Ports.portName(), Ports.baudrate()):
        raise RuntimeError("Could not open port. Aborting")

    # Get the awinda station object
    awinda = controlDev.device(devId)
    firmware_version = awinda.firmwareVersion()

    # assert (awinda is not 0)
    logger.log("Device: %s, with ID: %s open. \n" % (awinda.productCode(), awinda.deviceId().toXsString()))

    # Put the device in configuration mode
    logger.log("Putting device into configuraiton mode...\n")
    if not awinda.gotoConfig():
        raise RuntimeError("Could not put device into configuration mode. Aborting.")

    """
    XSO_Orientation : keep the orientation data of MTw

    XSO_Calibrate : compute calibrated data from raw data obtained by the MTw

    XSO_RetainLiveData: keep the currently streaming data from MTw
    """
    awinda.setOptions(xda.XSO_Orientation + xda.XSO_Calibrate + xda.XSO_RetainLiveData, 0)

    logger.log("Creating a log file...")
    if awinda.createLogFile(logFileName) != xda.XRV_OK:
        raise RuntimeError("Failed to create a log file. Aborting.")
    else:
        logger.log("Created a log file: %s" % logFileName)

    # Implementing the update rate selected by the user
    logger.log("Update rate chosen: {} \n".format(updateRate))

    if not awinda.setUpdateRate(updateRate):
        raise RuntimeError("Could not set up the update rate. Aborting.")

    # Implementing the chosen radio channel
    logger.log("Radio channel chosen: {} \n".format(radioChannel))

    try:
        awinda.enableRadio(radioChannel)
    except ValueError:
        logger.log("The radio is still active, please undock the device from the computer and try again.")

    logger.log("Undock the MTw devices from the Awinda station and wait until the devices are connected (synced leds)")

    # Attaching XsDevice objects to each MTw wireless connected to the awinda station
    MTws = awinda.children()
    MTwsAdresses = [MTws[i].deviceId().toXsString() for i in range(len(MTws))]  # Associated i.p. addresses of the IMUs

    # Waiting for all IMUs to be attached
    # 2 step process:
    # 1.We wait for all IMUs in the "IMUs" variable to be contained in MTws
    # 2. We then look in MTws to see of there are some IMUs that shouldn't be here, and remove them
    foundAllImus = False
    # Step 1
    while not foundAllImus:
        missingImus = []
        for imu in IMUs:
            imuAdress = name2Adress(imu)
            if imuAdress in MTwsAdresses:
                continue
            else:
                missingImus.append(imu)
        if len(missingImus) > 0:
            logger.log("Missing : {}".format(missingImus))
            time.sleep(2)
            MTws = awinda.children()
            MTwsAdresses = [MTws[i].deviceId().toXsString() for i in range(len(MTws))]
        else:
            foundAllImus = True
    # Step 2:
    if len(MTws) > len(IMUs):
        #     This means that at least one extra IMU is here that shouldn't
        tempMTws = []
        MTwsAdresses = [MTws[i].deviceId().toXsString() for i in range(len(MTws))]
        for imuAdress, imu in zip(MTwsAdresses, MTws):
            imuName = adress2Name(imuAdress)
            if imuName in IMUs:  # We check that this IMU was really desired
                tempMTws.append(imu)
        MTws = tempMTws


    devIdAll = []
    mtwCallbacks = []
    filenamesPCKL = []
    for i in range(len(MTws)):
        devIdAll.append(MTws[i].deviceId())
        mtwCallbacks.append(MTwCallback(maxBufferSize))
        MTws[i].addCallbackHandler(mtwCallbacks[i])  # add callback to handle data transmission to each MTw

    devicesUsed, devIdUsed, nDevs = checkConnectedSensors(devIdAll, MTws, controlDev, awinda, Ports, logger)

    # Create pickle txt files for each MTw connected
    for n in range(nDevs):
        filePCKL = "PCKL_" + str(devId.toXsString()) + "_" + str(devIdUsed[n]) + ".txt"
        filenamesPCKL.append(os.path.join(MTw_pickle, filePCKL))
        open(filenamesPCKL[-1], 'wb')

    # Put devices to measurement mode
    logger.log("Putting devices into measurement mode...\n")

    if not awinda.gotoMeasurement():
        raise RuntimeError("Could not put device into measurement mode. Aborting.")

    # Wait period to let the filters of the devices warm up before acquiring data
    logger.log("Wait 10 seconds before starting data acquisition. XSens filters are warming up... \n")
    time.sleep(10)
    logger.log("IMUs are ready.\n")
    # Reset the yaw (XRM_Heading), pitch and roll (XRM_Alignment) angles to 0
    for n in range(nDevs):
        if not devicesUsed[n].resetOrientation(xda.XRM_Heading + xda.XRM_Alignment):
            logger.log("Could not reset the header.")

    if not awinda.startRecording():
        raise RuntimeError("Failed to start recording. Aborting.")

    return awinda, mtwCallbacks, filenamesPCKL, devId, devIdUsed, nDevs, firmware_version, controlDev, Ports


def writeXsens(mtwCallbacks, filenamesPCKL):
    for callback, filenamePCKL in zip(mtwCallbacks, filenamesPCKL):
        if callback.packetAvailable():
            callback.writeData(filenamePCKL)
