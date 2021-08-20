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
    print("Arrêt de l'enregistrement, mise du dispositif en mode configuration.")
    device.gotoConfig()
    device.disableRadio()

    print("Fermeture du port.")
    control.closePort(Ports)
    control.close()
    xda.XsControl.destruct(control)



def checkConnectedSensors(devIdAll, children, control, device, Ports):
    childUsed = np.full(np.shape(children), False).tolist()
    if children.empty():
        print("Aucun dispositif trouvé.")
        stopAll(device, control, Ports)
        raise RuntimeError("Échec. Aucun capteur MTw trouvé.")
    else:
        for i in range(len(children)):
            if children[i].connectivityState() == xda.XCS_Wireless:
                childUsed[i] = True

        print("\n Dispositifs rejetés:")
        rejects = np.array(devIdAll)[[not elem for elem in childUsed]].tolist()
        for i in range(len(rejects)):
            index = devIdAll.index(rejects[i])
            # MTw = MTwIdentifier(rejects[i].toXsString().__str__())
            children[index].requestBatteryLevel()
            time.sleep(0.1)
            print("%d - " % index + "%s" % rejects[i] + " || battery percentage: " + " %d" % children[index].batteryLevel())

        print("\n Dispositifs acceptés:")
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
            raise RuntimeError("Échec. Aucun capteur MTw trouvé.")
    devicesUsed = np.array(children)[childUsed].tolist()
    devIdUsed = np.array(devIdAll)[childUsed].tolist()
    nDevs = sum(childUsed)

    return devicesUsed, devIdUsed, nDevs

def pickle2txt(devId, devIdAll, devIdUsed, nDevs, firmware_version, filenames, updateRate):
    f_path = os.path.dirname(os.path.realpath("__file__")) + "\\MTw data"
    for n in range(nDevs):
        dataPackets = []
        filename = "MT_" + str(devId.toXsString()) + "_" + str(devIdUsed[n]) + ".txt"
        with (open(filenames[n], 'rb')) as openfile:
            while True:
                try:
                    dataPackets.append(pickle.load(openfile))
                except EOFError:
                    break
        packetCounter = [item[0] for item in dataPackets][4:]
        acceleration = [item[1] for item in dataPackets][4:]
        orientationMatrix = [item[2] for item in dataPackets][4:]
        # timeOfArrival = [item[3] for item in dataPackets]
        # timeMeasurement = timeOfArrival2timeMeasurement(timeOfArrival, updateRate)
        numberPackets = len(packetCounter)
        packetCounter = removeResetCounter(packetCounter)
        ref = [x for x in range(packetCounter[0], packetCounter[-1] + 1)
               if x not in packetCounter]

        if ref:
            packetCounter, acceleration, orientationMatrix = interPolateData(packetCounter, acceleration, orientationMatrix)

        packetsMissing = len(packetCounter) - numberPackets
        filepath = os.path.join(f_path, filename)
        file_txt = open(filepath, "w")
        file_txt.write("// Start Time: Unknown: \n")
        file_txt.write("// Update Rate: " + str(float(updateRate)) + "Hz \n")
        file_txt.write("// Filter Profile: human (46.1) \n")
        file_txt.write("// Firmware Version: " + str(firmware_version.toXsString()) + "\n")
        file_txt.write(
            "PacketCounter\tSampleTimeFine\tYear\tMonth\tDay\tSecond\tUTC_Nano\tUTC_Year\tUTC_Month\tUTC_Day\tUTC_Hour\tUTC_Minute\tUTC_Second\tUTC_Valid\tAcc_X\tAcc_Y\tAcc_Z\tMat[1][1]\tMat[2][1]\tMat[3][1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3] \n")

        for i in range(len(packetCounter)):
            # file_txt.write(str(packetCounter[i]) + "\t\t\t\t\t\t\t\t")
            file_txt.write(str(packetCounter[i]) + "\t\t\t\t\t\t\t\t\t\t\t\t\t\t")
            # date = timeMeasurement[i].split(' ')[0]
            # temps = timeMeasurement[i].split(' ')[1]
            # file_txt.write(date.split('/')[0] + "\t")
            # file_txt.write(date.split('/')[1] + "\t")
            # file_txt.write(date.split('/')[2] + "\t")

            # file_txt.write(temps.split(':')[0] + "\t")
            # file_txt.write(temps.split(':')[1] + "\t")
            # file_txt.write(temps.split(':')[2] + "\t")

            matrix = orientationMatrix[i].reshape(9, )
            for k in range(3):
                file_txt.write('{:.6f}'.format(round(acceleration[i][k], 6)) + "\t")
            for j in range(9):
                file_txt.write('{:.6f}'.format(round(matrix[j], 6)) + "\t")
            file_txt.write("\n")
        file_txt.close()

        print(devIdAll[n], "number of data packets: ", len(packetCounter), " with ", packetsMissing, " packets interpolated")

def timeOfArrival2timeMeasurement(timeOfArrival, updateRate):
    t0 = time.strptime(timeOfArrival[4][11:-1].split('.')[0],'%H:%M:%S')
    t0_s = datetime.timedelta(hours=t0.tm_hour, minutes=t0.tm_min, seconds=t0.tm_sec).total_seconds()
    t0_ms = int(timeOfArrival[4][11:].split('.')[1])/1000

    t = time.strptime(timeOfArrival[-1][11:-1].split('.')[0],'%H:%M:%S')
    t_s = datetime.timedelta(hours=t0.tm_hour, minutes=t0.tm_min, seconds=t0.tm_sec).total_seconds()
    t_ms = int(timeOfArrival[-1][11:].split('.')[1])/1000

    delta_t_dot = round((t_s + t_ms)-(t0_s + t0_ms), 3)
    delta = round(np.abs(delta_t_dot - len(timeOfArrival[4:])/updateRate), 3)

    timeMeasurement = []
    temps_init = t0_s + t0_ms - delta
    for n in range(len(timeOfArrival[4:])):
        seconds = round(temps_init + n/60, 3)
        m_s = round(seconds%1, 3)
        ty_res = time.gmtime(seconds)
        temps = time.strftime("%H:%M:%S", ty_res)
        temps = temps + str(m_s)[1:]
        timeMeasurement.append(timeOfArrival[n].split(' ')[0] + ' ' + temps)

    return timeMeasurement

def removeResetCounter(PacketCounter, maxWrap = 65535):
    index = [i for i, element in enumerate(PacketCounter) if element == 0]
    if index:
        for n in index:
            for i in range(n, len(PacketCounter)):
                PacketCounter[i] = PacketCounter[i] + maxWrap + 1

    return PacketCounter

def interPolateData(PacketCounter, acceleration, orientationMatrix):
    i = 0
    while True:
        if PacketCounter[i+1] != PacketCounter[i] + 1:
            acc = np.mean([acceleration[i], acceleration[i+1]], 0)
            orientation = np.mean([orientationMatrix[i], orientationMatrix[i+1]], 0)
            PacketCounter.insert(i+1, PacketCounter[i]+1)
            acceleration.insert(i+1, acc)
            orientationMatrix.insert(i+1, orientation)
        i += 1

        if PacketCounter[-1] - PacketCounter[i] == 1
            break

    return PacketCounter, acceleration, orientationMatrix
