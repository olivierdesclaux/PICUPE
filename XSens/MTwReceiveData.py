import os.path
import numpy as np
import xsensdeviceapi as xda
import time
import threading as th
import sys
import pickle
import argparse
from threading import Lock

# Local modules
from MTwFunctions import stopAll, checkConnectedSensors, pickle2txt


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
        assert (packet is not 0)
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
        assert (len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        with open(filename, "ab") as file_handle:
            pickle.dump((oldest_packet.packetCounter(), oldest_packet.calibratedAcceleration(),
                         oldest_packet.orientationMatrix(),
                         oldest_packet.timeOfArrival().utcToLocalTime().toXsString().__str__()),
                        file_handle)
        self.m_lock.release()


def key_capture_thread(awinda):
    """Thread that stops the recording loop for the MTw devices
    Parameters
    ----------
    awinda : class XsDevice object
        Object that stores all the functions of a XSens device
    Returns
    -------
    None.
    """
    global keep_going
    input("Press enter to stop recording... \n")
    keep_going = False
    print("Abort flushing operation...")
    if not awinda.abortFlushing():
        print("Failed to abort flushing operation.")
    print("Stopping recording...\n")
    if not awinda.stopRecording():
        print("Failed to stop recording. Aborting.")


## Main function ##
def main(updateRate, radioChannel):
    """MTwReceiveData.py obtains the data packets sent by the MTw to write them in a txt file.

    The script ask users to input an update rate of the system (40, 60, 80, 100, 120 Hz) and a radio channel to
    establish a wireless communication between the MTw and the Awinda station. The default values are 60 Hz for the
    update rate and radio channel 11.

    Parameters
    ----------
    updateRate : int
        Establishes the frequency of data transmitted from the MTw to the Awinda station (read manual MTw user
         manual for recommended values)
    radioChannel : int
        Channel to connect the IMU to the winda station wireless.
    Returns
    -------
    None.
    """
    savePath = "./Results"
    maxBufferSize = 5
    global keep_going
    try:
        # Extract the XsDeviceApi version used

        print("Creating a XsControl object...")
        controlDev = xda.XsControl_construct()
        assert (controlDev is not 0)

        xdaVersion = xda.XsVersion()
        xda.xdaVersion(xdaVersion)
        print("Using the following XDA version: %s \n" % xdaVersion.toXsString())

        print("Detecting ports...\n")
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
        print(" Found awinda station with: ")
        print(" Device ID: %s" % devId.toXsString())
        print(" Port name: %s" % Ports.portName())
        print(" Port baudrate: %s" % Ports.baudrate())

        print("Opening port...")
        if not controlDev.openPort(Ports.portName(), Ports.baudrate()):
            raise RuntimeError("Could not open port. Aborting")

        # Get the awinda station object
        awinda = controlDev.device(devId)
        firmware_version = awinda.firmwareVersion()

        assert (awinda is not 0)
        print("Disvice: %s, with ID: %s open. \n" % (awinda.productCode(), awinda.deviceId().toXsString()))

        # Put the device in configuration mode
        print("Putting device into configuraiton mode...\n")
        if not awinda.gotoConfig():
            raise RuntimeError("Could not put device into configuration mode. Aborting.")

        """
        XSO_Orientation : keep the orientation data of MTw
        
        XSO_Calibrate : compute calibrated data from raw data obtained by the MTw
        
        XSO_RetainLiveData: keep the currently streaming data from MTw
        """
        awinda.setOptions(xda.XSO_Orientation + xda.XSO_Calibrate + xda.XSO_RetainLiveData, 0)

        print("Creating a log file...")
        logFileName = "logfile.mtb"
        if awinda.createLogFile(logFileName) != xda.XRV_OK:
            raise RuntimeError("Failed to create a log file. Aborting.")
        else:
            print("Created a log file: %s" % logFileName)

        # Implementing the update rate selected by the user
        print("Update rate chosen: ", updateRate, "\n")

        if not awinda.setUpdateRate(updateRate):
            raise RuntimeError("Could not set up the update rate. Aborting.")

        # Implementing the chosen radio channel
        print("Radio channel chosen: ", radioChannel, "\n")

        try:
            awinda.enableRadio(radioChannel)
        except ValueError:
            print("The radio is still active, please undock the device from the computer and try again.")

        input(
            '\n Undock the MTw devices from the Awinda station and wait until the devices are connected (synced leds), then press enter... \n')

        # Attaching XsDevice objects to each MTw wireless connected to the awinda station
        MTws = awinda.children()

        devIdAll = []
        mtwCallbacks = []
        filenamesPCKL = []

        for i in range(len(MTws)):
            devIdAll.append(MTws[i].deviceId())
            mtwCallbacks.append(MTwCallback(maxBufferSize))
            MTws[i].addCallbackHandler(mtwCallbacks[i])  # add callback to handle data transmission to each MTw

        devicesUsed, devIdUsed, nDevs = checkConnectedSensors(devIdAll, MTws, controlDev, awinda, Ports)

        #### Flushing the devices
        controlDev.flushInputBuffers()

        MTw_pickle = os.path.join(savePath, "MTw Pickle")

        # Create pickle txt files for each MTw connected
        for n in range(nDevs):
            filePCKL = "PCKL_" + str(devId.toXsString()) + "_" + str(devIdUsed[n]) + ".txt"
            filenamesPCKL.append(os.path.join(MTw_pickle, filePCKL))
            open(filenamesPCKL[-1], 'wb')

        # Put devices to measurement mode
        print("Putting devices into measurement mode...\n")

        if not awinda.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        # Wait period to let the filters of the devices warm up before acquiring data
        print("Wait 10 seconds before starting data acquisition.\n")
        time.sleep(10)

        # Reset the yaw (XRM_Heading), pitch and roll (XRM_Alignment) angles to 0
        for n in range(nDevs):
            if not devicesUsed[n].resetOrientation(xda.XRM_Heading + xda.XRM_Alignment):
                print("Could not reset the header.")

        keep_going = True

        input("Press enter to start recording...\n")
        th.Thread(target=key_capture_thread, args=(awinda,), name='key_capture_thread', daemon=True).start()

        print("Starting recording...\n")
        if not awinda.startRecording():
            raise RuntimeError("Failed to start recording. Aborting.")
        startTime = xda.XsTimeStamp_nowMs()

        while keep_going:
            for i in range(len(mtwCallbacks)):
                callback = mtwCallbacks[i]
                filenamePCKL = filenamesPCKL[i]

                if callback.packetAvailable():
                    callback.writeData(filenamePCKL)

        startEnd = xda.XsTimeStamp_nowMs() - startTime
        print("Time of recording (s): ", startEnd / 1000, "\n")
        print("Number of packets that should be acquired by each MTw:", round(updateRate * startEnd / 1000))

        # Writing the data from a pickle txt file to a readable txt file
        pickle2txt(devId, devIdUsed, nDevs, firmware_version, filenamesPCKL, updateRate, savePath, maxBufferSize)

        print("Closing log file...")
        if not awinda.closeLogFile():
            raise RuntimeError("Failed to close log file. Aborting.")

        exit_program = int(input('\n Press 0 to exit program... \n'))
        if exit_program == 0:
            stopAll(awinda, controlDev, Ports)


    except RuntimeError as error:
        print(error)
        sys.exit(1)

    except Exception as e:
        print("An unknown fatal error has occurred. Aborting.")
        print(e)
        sys.exit(1)

    else:
        print("Successful exit.")


####

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Select update rate with -rate argument.
    parser.add_argument('-r', '--rate', type=int, dest='updateRate', default=40,
                        help='Select an update rate (40 - 120) Hz',
                        required=False)
    # Select radio channel with -radio argument
    parser.add_argument('--radio', type=int, dest='radioChannel', default=11,
                        help='Select a radio Channel between 11 to 25',
                        required=False)
    args = parser.parse_args().__dict__

    updateRate = args['updateRate']
    radioChannel = args['radioChannel']

    main(updateRate, radioChannel)
