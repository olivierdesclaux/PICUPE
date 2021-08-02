from threading import Lock

import numpy as np
import xsensdeviceapi as xda
import sys
import re
import threading as th
import os
import time


# Création d'un Handle pour la lecture des packets des MTw
class MTwCallback(xda.XsCallback):
    def __init__(self, max_buffer_size=1024):
        xda.XsCallback.__init__(self)
        self.m_maxNumberOfPacketsInBuffer = max_buffer_size
        self.m_packetBuffer = list()
        self.m_lock = Lock()

    def packetAvailable(self):
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def getNextPacket(self):
        self.m_lock.acquire()
        assert (len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        self.m_lock.release()
        return oldest_packet

    def onLiveDataAvailable(self, dev, packet):
        self.m_lock.acquire()
        assert (packet is not 0)
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()


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

        print("Dispositifs rejetés:")
        rejects = np.array(devIdAll)[[not elem for elem in childUsed]].tolist()
        for i in range(len(rejects)):
            index = devIdAll.index(rejects[i])
            print("%d - " % index + "%s" % rejects[i])

        print("Dispositifs acceptés:")
        accepted = np.array(devIdAll)[childUsed].tolist()
        for i in range(len(accepted)):
            index = devIdAll.index(accepted[i])
            print("%d - " % index + "%s" % accepted[i])

        option = str(input('Keep current status?' + ' (y/n): ')).lower().strip()
        change = []
        if option[0] == 'n':
            op2 = input(
                "\n Type the numbers of the sensors (csv list, e.g. 1,2,3) from which status should be changed \n (if accepted than reject or the other way around):\n")
            change = [int(i) for i in re.split(",", op2)]
            for i in range(len(change)):
                if a[change[i]]:
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


## Thread to stop recording loop ##
def key_capture_thread():
    global keep_going
    input("Press enter to stop recording.")
    keep_going = False


####################################


def export_txt(devId, devIdUsed, nDevs, firmware_version):
    global matrix_nDev, acc_nDev, counter_nDev
    f_path = os.path.dirname(os.path.realpath("__file__"))
    for n in range(nDevs):
        filename = "MT_" + str(devId.toXsString()) + "_" + str(devIdUsed[n]) + ".txt"
        path = os.path.join(f_path, filename)
        file_txt = open(path, "w")
        file_txt.write("// Start Time: Unknown " + "\n")
        file_txt.write("// Update Rate: " + str(float(fq_support[index_fq])) + "Hz \n")
        file_txt.write("// Filter Profile: human (46.1) \n")
        file_txt.write("// Firmware Version: " + str(firmware_version.toXsString()) + "\n")
        file_txt.write(
            "PacketCounter\tSampleTimeFine\tYear\tMonth\tDay\tSecond\tUTC_Nano\tUTC_Year\tUTC_Month\tUTC_Day UTC_Hour\tUTC_Minute\tUTC_Second\tUTC_Valid\tAcc_X\tAcc_Y\tAcc_Z\tMat[1][1]\tMat[2][1]\tMat[3][1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3] \n")

        for i in range(len(counter_nDev[n])):
            file_txt.write(str(counter_nDev[n][i]) + "\t\t\t\t\t\t\t\t\t\t\t\t\t\t")
            matrix = matrix_nDev[n][i].reshape(9, )
            for k in range(3):
                file_txt.write('{:.6f}'.format(round(acc_nDev[n][i][k], 6)) + "\t")
            for j in range(9):
                file_txt.write('{:.6f}'.format(round(matrix[j], 6)) + "\t")
            file_txt.write("\n")
        file_txt.close()


# Debut du script
if __name__ == '__main__':
    try:
        # Extraire la version du Xs Device Api employé

        print("Creation d'un objet XsControl...")
        control = xda.XsControl_construct()
        assert (control is not 0)

        xdaVersion = xda.XsVersion()
        xda.xdaVersion(xdaVersion)
        print("Utilisation de la version XDA: %s \n" % xdaVersion.toXsString())

        print("Détection des ports#...\n")
        ports_Scan = xda.XsScanner_scanPorts(0, 100, True, True)

        # Détection des dispositifs MTw ou Station Awinda
        Ports = xda.XsPortInfo()
        for i in range(ports_Scan.size()):
            if ports_Scan[i].deviceId().isWirelessMaster() or ports_Scan[i].deviceId().isAwindaXStation():
                Ports = ports_Scan[i]
                break

        if Ports.empty():
            raise RuntimeError("Échec. Aucun dispositif a été detecté.")

        devId = Ports.deviceId()
        print(" Dispositif trouvé avec les caractéristiques suivantes:")
        print(" Device ID: %s" % devId.toXsString())
        print(" Nom du port: %s" % Ports.portName())
        print(" Port baudrate: %s" % Ports.baudrate())

        print("Ouverture du port...")
        if not control.openPort(Ports.portName(), Ports.baudrate()):
            raise RuntimeError("Échec de l'ouverture du port.")

        device = control.device(devId)
        firmware_version = device.firmwareVersion()

        assert (device is not 0)
        print("Dispositif: %s, avec ID: %s ouvert. \n" % (device.productCode(), device.deviceId().toXsString()))

        # Mettre le dispositif en mode configuration
        print("Dispositif en mode configuration...\n")
        if not device.gotoConfig():
            raise RuntimeError("Échec. Le dispositif n'a as pu être mis en mode configuration.")

        device.setOptions(xda.XSO_Orientation + xda.XSO_Calibrate + xda.XSO_RetainLiveData, 0)
        firmware_version = device.firmwareVersion()

        # Acquisition des fréquences pour la mise à jour des données
        fq_support = device.supportedUpdateRates(xda.XDI_None)
        index_fq = []

        while index_fq == []:
            print("Les fréquences disponibles sont:")
            print('\n'.join('{}: {}'.format(*k) for k in enumerate(fq_support)))
            choix_fq = input("Veuillez choisir une fréquence d'acquisition.")
            if choix_fq == []:
                continue
            index_fq = fq_support.index(int(choix_fq))
        print("Fréquence d'acquisition choisi: ", fq_support[index_fq], "\n")

        # Mise à jour de la fréquence d'acquisition
        if not device.setUpdateRate(fq_support[index_fq]):
            raise RuntimeError("Échec de la mise à jour du update rate.")

        # Acquisition du canal radio
        canaux_radio = [11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25]
        index_radio = []

        while index_radio == []:
            print("Les canaux disponibles sont: ")
            print('\n'.join('{}: {}'.format(*k) for k in enumerate(canaux_radio)))
            choix_radio = input("Veuillez choisir un canal de radio.")
            if choix_radio == []:
                continue
            index_radio = canaux_radio.index(int(choix_radio))
        print("Canal de radio choisi: ", canaux_radio[index_radio], "\n")

        # Mise à jour du canal radio
        try:
            device.enableRadio(canaux_radio[index_radio])
        except ValueError:
            print("La radio est encore activée, veuillez débrancher l'appareil de l'ordinateur et reessayer.")

        input(
            '\n Undock the MTw devices from the Awinda station and wait until the devices are connected (synced leds), then press enter... \n')
        # Vérifier les dispositifs détectées (nombre de capteurs MTw)

        children = device.children()

        devIdAll = []
        mtwCallbacks = []
        for i in range(len(children)):
            devIdAll.append(children[i].deviceId())
            mtwCallbacks.append(MTwCallback())
            children[i].addCallbackHandler(mtwCallbacks[i])

        devicesUsed, devIdUsed, nDevs = checkConnectedSensors(devIdAll, children, control, device, Ports)

        ##################

        # Mis les capteurs MTw en mode acquisition des données
        print("Putting devices into measurement mode...\n")

        if not device.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")

        ### Initialisation of list containing datapackets of MTw ##
        matrix_nDev = list(range(nDevs))
        acc_nDev = list(range(nDevs))
        pack_nDev = list(range(nDevs))
        counter_nDev = list(range(nDevs))

        for i in range(nDevs):
            pack_nDev[i] = []
            counter_nDev[i] = []
            acc_nDev[i] = []
            matrix_nDev[i] = []
        ############################################################

        print("Wait 10 seconds before starting data acquisition.\n")
        time.sleep(10)

        input("Press enter to start recording...\n")
        print("Starting acquisition...\n")

        startTime = xda.XsTimeStamp_nowMs()

        keep_going = True
        th.Thread(target=key_capture_thread, args=(), name='key_capture_thread', daemon=True).start()

        while keep_going:
            for i in range(len(mtwCallbacks)):
                callback = mtwCallbacks[i]
                if callback.packetAvailable():
                    packet = callback.getNextPacket()
                    pack_nDev[i].append(packet)
                    acc_nDev[i].append(packet.calibratedAcceleration())
                    counter_nDev[i].append(packet.packetCounter())
                    matrix_nDev[i].append(packet.orientationMatrix())

        startEnd = xda.XsTimeStamp_nowMs() - startTime
        print("Time end (s): ", startEnd / 1000, "\n")
        #####
        for i in range(nDevs):
            print(devIdAll[i], "number of data packets: ", len(counter_nDev[i]))

        ##################

        exit = int(input('\n Appuyez sur la touche 0 pour quitter le mode acquisition... \n'))
        if exit == 0:
            stopAll(device, control, Ports)

        export_txt(devId, devIdUsed, nDevs, firmware_version)
    except RuntimeError as error:
        print(error)
        sys.exit(1)

    except:
        print("An unknown fatal error has occured. Aborting.")
        sys.exit(1)

    else:
        print("Successful exit.")
