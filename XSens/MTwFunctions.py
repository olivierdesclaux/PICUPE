import numpy as np
import os
import pickle
import datetime


def pickle2txt(mtwManager):
    """Write readable txt files from pickle txt files and applies interpolation and quick fixes on missing packets
    Parameters
    ----------
    mtwManager: mtwManager
    Returns
    -------
    None.
    """
    f_path = os.path.join(mtwManager.savePath, "MTw data")
    if not os.path.isdir(f_path):
        os.mkdir(f_path)

    # nIMUs = len(mtwManager)
    nPacketFinal = []
    nPacketInitial = []
    for file in mtwManager.filenamesPCKL:
        # for n in range(nDevs):
        dataPackets = []
        with (open(file, 'rb')) as openfile:
            while True:
                try:
                    dataPackets.append(pickle.load(openfile))
                except EOFError:
                    break
        packetCounter = [item[0] for item in dataPackets][mtwManager.maxBufferSize - 1:]
        nPacketFinal.append(packetCounter[-1])
        nPacketInitial.append(packetCounter[0])
    finalPacket = int(np.median(nPacketFinal))  # Quick fix for all devices to start at the same data packet counter
    initialPacket = int(np.median(nPacketInitial))  # Quick fix for all devices to end at the same data packet counter

    # for n in range(nDevs):
    for imu, pcklFile in zip(mtwManager.MTws, mtwManager.filenamesPCKL):
        dataPackets = []
        filename = "MT_" + str(mtwManager.deviceId.toXsString()) + "_" + str(imu.deviceId()) + ".txt"
        with (open(pcklFile, 'rb')) as openfile:
            while True:
                try:
                    dataPackets.append(pickle.load(openfile))
                except EOFError:
                    break
        packetCounter = [item[0] for item in dataPackets][mtwManager.maxBufferSize - 1:]
        acceleration = [item[1] for item in dataPackets][mtwManager.maxBufferSize - 1:]
        orientationMatrix = [item[2] for item in dataPackets][mtwManager.maxBufferSize - 1:]
        timeOfArrival = [item[3] for item in dataPackets[mtwManager.maxBufferSize - 1:]]

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
        file_txt.write("// Update Rate: " + str(float(mtwManager.updateRate)) + "Hz \n")
        file_txt.write("// Filter Profile: human (46.1) \n")
        file_txt.write("// Firmware Version: " + str(mtwManager.firmwareVersion.toXsString()) + "\n")
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

        mtwManager.logger.log(
            "{} Number of data packets: {} with {} packets interpolated, ({} %)".format(imu.deviceId(),
                                                                                        len(packetCounter),
                                                                                        packetsMissing, round(packetsMissing/len(packetCounter) * 100, 4)))


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
