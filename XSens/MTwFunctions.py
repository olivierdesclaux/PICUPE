import numpy as np
import xsensdeviceapi as xda
import sys
import re
import threading as th
import os
import time
import pickle


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
        packetCounter = [item[0] for item in dataPackets]
        acceleration = [item[1] for item in dataPackets]
        orientationMatrix = [item[2] for item in dataPackets]
        filepath = os.path.join(f_path, filename)
        file_txt = open(filepath, "w")
        file_txt.write("// Start Time: Unknown: \n")
        file_txt.write("// Update Rate: " + str(float(updateRate)) + "Hz \n")
        file_txt.write("// Filter Profile: human (46.1) \n")
        file_txt.write("// Firmware Version: " + str(firmware_version.toXsString()) + "\n")
        file_txt.write(
            "PacketCounter\tSampleTimeFine\tYear\tMonth\tDay\tSecond\tUTC_Nano\tUTC_Year\tUTC_Month\tUTC_Day UTC_Hour\tUTC_Minute\tUTC_Second\tUTC_Valid\tAcc_X\tAcc_Y\tAcc_Z\tMat[1][1]\tMat[2][1]\tMat[3][1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3] \n")

        for i in range(4, len(dataPackets)):
            file_txt.write(str(packetCounter[i]) + "\t\t\t\t\t\t\t\t\t\t\t\t\t\t")
            matrix = orientationMatrix[i].reshape(9, )
            for k in range(3):
                file_txt.write('{:.6f}'.format(round(acceleration[i][k], 6)) + "\t")
            for j in range(9):
                file_txt.write('{:.6f}'.format(round(matrix[j], 6)) + "\t")
            file_txt.write("\n")
        file_txt.close()

        print(devIdAll[n], "number of data packets: ", len(packetCounter[4:]))
