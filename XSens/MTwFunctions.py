import datetime

import numpy as np
import xsensdeviceapi as xda
import sys
import re
import threading as th
import os
import time
import pickle
import datetime


class MTwIdentifier:
    def __init__(self, ID):
        self.ID = ID
        if ID == '00B48784':
            self.label = 'HAND_L'
        elif ID == '00B487B6':
            self.label = 'fARM_L'
        elif ID == '00B48760':
            self.label = 'SHOU'
        elif ID == '00B48761':
            self.label = 'FOOT'
        elif ID == '00B4876D':
            self.label = 'fARM_R'
        elif ID == '00B4875D':
            self.label = 'HAND_R'
        elif ID == '00B48772':
            self.label = 'PELV'
        elif ID == '00B487B7':
            self.label = 'PROP 1'
        elif ID == '00B4876F':
            self.label = 'uLEG_R'
        elif ID == '00B4875C':
            self.label = 'uARM_L'
        elif ID == '00B48770':
            self.label = 'uARM_R'
        elif ID == '00B4875B':
            self.label = 'STERN'
        elif ID == '00B48773':
            self.label = 'uLEG_L'
        elif ID == '00B48769':
            self.label = 'LLEG_L'
        elif ID == '00B4876C':
            self.label = 'LLEG_R'
        elif ID == '00B48768':
            self.label = 'FOOT_L'
        elif ID == '00B48765':
            self.label = 'HEAD'
        elif ID == '00B4876E':
            self.label = 'SHOU_L'


def stopAll(device, control, Ports):
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
    print("Putting device into configuration mode.")
    device.gotoConfig()
    device.disableRadio()

    print("Closing ports.")
    control.closePort(Ports)
    control.close()
    print("Destroying XsControl object.")
    xda.XsControl.destruct(control)



def checkConnectedSensors(devIdAll, children, control, device, Ports):
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

        print("\n Rejected devices: ")
        rejects = np.array(devIdAll)[[not elem for elem in childUsed]].tolist()
        for i in range(len(rejects)):
            index = devIdAll.index(rejects[i])
            children[index].requestBatteryLevel()
            time.sleep(0.1)
            print("%d - " % index + "%s" % rejects[i] + " || battery percentage: " + " %d" % children[index].batteryLevel())

        print("\n Accepted devices: ")
        accepted = np.array(devIdAll)[childUsed].tolist()
        for i in range(len(accepted)):
            index = devIdAll.index(accepted[i])
            children[index].requestBatteryLevel()
            time.sleep(0.1)
            print("%d - " % index + "%s" % accepted[i] + " || battery percentage: " + " %d" % children[index].batteryLevel())

        option = str(input('Keep current status?' + ' (y/n): ')).lower().strip()
        change = []
        if option[0] == 'n':
            op2 = input(
                "\n Type the numbers of the sensors (csv list, e.g. 1,2,3) from which status should be changed \n (if accepted than reject or the other way around):\n")
            change = [int(i) for i in re.split(",", op2)]
            for i in range(len(change)):
                if devIdAll[change[i]]:
                    device.rejectConnection(children[change[i]])
                    childUsed[change[i]] = False
                else:
                    device.acceptConnection(children[change[i]])
                    childUsed[change[i]] = True
        if sum(childUsed) == 0:
            stopAll(device, control, Ports)
            raise RuntimeError("No MTw devices found. Aborting.")
    devicesUsed = np.array(children)[childUsed].tolist()
    devIdUsed = np.array(devIdAll)[childUsed].tolist()
    nDevs = sum(childUsed)

    return devicesUsed, devIdUsed, nDevs

def pickle2txt(devId, devIdUsed, nDevs, firmware_version, filenames, updateRate, maxBuffer = 5):
    """Write readable txt files from pickle txt files and applies interpolation and quick fixes on missing packets
    Parameters
    ----------
    devId : string
        ID of the awinda station
    devIdUsed: list strings
        List of de connected MTw devices IDs
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
    f_path = os.path.dirname(os.path.realpath("__file__")) + "\\MTw data"
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
    nPacketFinalRef = int(np.median(nPacketFinal)) # Quick fix for all devices to start at the same data packet counter
    nPacketInitialRef = int(np.median(nPacketInitial)) # Quick fix for all devices to end at the same data packet counter

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
        timeOfArrival = [item[3] for item in dataPackets]
        numberPacketsRaw = len(packetCounter)

        # Quick fix to remove the packet counter wrapper
        packetCounter = removeResetCounter(packetCounter)
        ref = [x for x in range(packetCounter[0], packetCounter[-1] + 1)
               if x not in packetCounter]

        # Quick fix to interpolate missing data packets
        if ref:
            packetCounter, acceleration, orientationMatrix = interPolateData(packetCounter, acceleration, orientationMatrix)

        # Quick fix to start at the same packet count
        if packetCounter[0] > nPacketInitialRef:
            packetCounter.insert(0, packetCounter[0] - 1)
            acceleration.insert(0, acceleration[0])
            orientationMatrix.insert(0, orientationMatrix[0])
            timeOfArrival.insert(0, timeOfArrival[0])

        # Quick fix to end at the same packet count
        elif packetCounter[0] < nPacketInitialRef:
            packetCounter.pop(0)
            acceleration.pop(0)
            orientationMatrix.pop(0)
            timeOfArrival.pop(0)

        if packetCounter[-1] < nPacketFinalRef:
            packetCounter.append(packetCounter[-1]+1)
            acceleration.append(acceleration[-1])
            orientationMatrix.append(orientationMatrix[-1])
            timeOfArrival.append(timeOfArrival[-1])
        elif packetCounter[-1] > nPacketFinalRef:
            packetCounter.pop()
            acceleration.pop()
            orientationMatrix.pop()
            timeOfArrival.pop()

        numberPackets = len(packetCounter)
        packetsMissing = abs(numberPackets - numberPacketsRaw)

        # Quick fix to estimate the utc time of data packets from time of arrival
        timeMeasurement = timeOfArrival2timeMeasurement(timeOfArrival, updateRate, numberPackets)

        filepath = os.path.join(f_path, filename)
        file_txt = open(filepath, "w")
        file_txt.write("// Start Time: Unknown: \n")
        file_txt.write("// Update Rate: " + str(float(updateRate)) + "Hz \n")
        file_txt.write("// Filter Profile: human (46.1) \n")
        file_txt.write("// Firmware Version: " + str(firmware_version.toXsString()) + "\n")
        file_txt.write("// Option Flags: AHS Disabled ICC Disabled \n")
        file_txt.write(
            "PacketCounter\tSampleTimeFine\tYear\tMonth\tDay\tSecond\tUTC_Nano\tUTC_Year\tUTC_Month\tUTC_Day\tUTC_Hour\tUTC_Minute\tUTC_Second\tUTC_Valid\tAcc_X\tAcc_Y\tAcc_Z\tMat[1][1]\tMat[2][1]\tMat[3][1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3] \n")

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

        print(devIdUsed[n], "number of data packets: ", len(packetCounter), " with ", packetsMissing, " packets interpolated")

def timeOfArrival2timeMeasurement(timeOfArrival, updateRate, numberPackets):
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
    t0 = time.strptime(timeOfArrival[0][11:-1].split('.')[0],'%H:%M:%S')
    t0_s = datetime.timedelta(hours=t0.tm_hour, minutes=t0.tm_min, seconds=t0.tm_sec).total_seconds()
    t0_ms = int(timeOfArrival[0][11:].split('.')[1])/1000

    t = time.strptime(timeOfArrival[-1][11:-1].split('.')[0],'%H:%M:%S')
    t_s = datetime.timedelta(hours=t0.tm_hour, minutes=t0.tm_min, seconds=t0.tm_sec).total_seconds()
    t_ms = int(timeOfArrival[-1][11:].split('.')[1])/1000

    delta_t_dot = round((t_s + t_ms)-(t0_s + t0_ms), 3)
    delta = round(np.abs(delta_t_dot - numberPackets/updateRate), 3)

    timeMeasurement = []
    temps_init = t0_s + t0_ms - delta
    for n in range(numberPackets):
        seconds = round(temps_init + n/60, 3)
        m_s = round(seconds%1, 3)
        ty_res = time.gmtime(seconds)
        temps = time.strftime("%H:%M:%S", ty_res)
        temps = temps + str(m_s)[1:]
        timeMeasurement.append(timeOfArrival[0].split(' ')[0] + ' ' + temps)

    return timeMeasurement

def removeResetCounter(PacketCounter, maxWrap = 65535):
    """QUICKFIX to remove the reset counter at packet count # 65535
    Parameters
    ----------
    PacketCounter : list int
        List of packet count for the data packets
    maxWrap : int
        The number that the packet count reset to 0
    Returns
    -------
     PacketCounter : list int
        List of packet counter without any resets at count # 65535
    """
    index = []
    for i in range(len(PacketCounter)):
        if PacketCounter[i]//10 == 0 and PacketCounter[i-1]//10 != 0:
            index.append(i)

    if index:
        for n in index:
            for i in range(n, len(PacketCounter)):
                PacketCounter[i] = PacketCounter[i] + maxWrap + 1
    return PacketCounter

def interPolateData(PacketCounter, acceleration, orientationMatrix):
    """QUICKFIX to interpolate missing data packets
    Parameters
    ----------
    PacketCounter : list int
        List of packet count for the data packets
    acceleration : list numpy array [1,3]
        List of the calibrated XYZ accelerations of the data packets
    orientationMatrix : list numpy array [3,3]
        List of the rotation matrix of the data packets
    Returns
    -------
     PacketCounter :
        List of packet counter with the interpolated missing data packets
    acceleration : list numpy array [1,3]
        List of the calibrated XYZ accelerations with the interpolated accelerations
    orientationMatrix : list numpy array [3,3]
        List of the rotation matrix with the interpolated rotation matrix
    """
    i = 0
    while True:
        if PacketCounter[i+1] != PacketCounter[i] + 1:
            acc = np.mean([acceleration[i], acceleration[i+1]], 0)
            orientation = np.mean([orientationMatrix[i], orientationMatrix[i+1]], 0)
            PacketCounter.insert(i+1, PacketCounter[i]+1)
            acceleration.insert(i+1, acc)
            orientationMatrix.insert(i+1, orientation)
        i += 1

        if PacketCounter[-1] - PacketCounter[i] == 1:
            break

    return PacketCounter, acceleration, orientationMatrix
