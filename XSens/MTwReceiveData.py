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


# Création d'un Handle pour la lecture des packets des MTw
class MTwCallback(xda.XsCallback):
    def __init__(self, max_buffer_size = 5):
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

    def writeData(self, filename):
        self.m_lock.acquire()
        assert (len(self.m_packetBuffer) > 0)
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        with open(filename, "ab") as file_handle:
            pickle.dump((oldest_packet.packetCounter(), oldest_packet.calibratedAcceleration(),
                         oldest_packet.orientationMatrix(),
                         oldest_packet.timeOfArrival().utcToLocalTime().toXsString().__str__()),
                        (file_handle))
        self.m_lock.release()


## Thread to stop recording loop ##
def key_capture_thread(awinda):
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
    global keep_going
    try:
        # Extraire la version du Xs Device API employé

        print("Creation d'un objet XsControl...")
        controlDev = xda.XsControl_construct()
        assert (controlDev is not 0)

        xdaVersion = xda.XsVersion()
        xda.xdaVersion(xdaVersion)
        print("Utilisation de la version XDA: %s \n" % xdaVersion.toXsString())

        print("Détection des ports...\n")
        ports_Scan = xda.XsScanner_scanPorts(0, 100, True, True)

        # Détection d'une Station Awinda
        Ports = xda.XsPortInfo()
        for i in range(ports_Scan.size()):
            if ports_Scan[i].deviceId().isWirelessMaster() or ports_Scan[i].deviceId().isAwindaXStation():
                Ports = ports_Scan[i]
                break

        if Ports.empty():
            raise RuntimeError("Échec. Aucun dispositif a été detecté.")

        devId = Ports.deviceId()
        print(" Dispositif trouvé avec les caractéristiques suivantes: ")
        print(" Device ID: %s" % devId.toXsString())
        print(" Nom du port: %s" % Ports.portName())
        print(" Port baudrate: %s" % Ports.baudrate())

        print("Ouverture du port...")
        if not controlDev.openPort(Ports.portName(), Ports.baudrate()):
            raise RuntimeError("Échec de l'ouverture du port.")

        awinda = controlDev.device(devId)
        firmware_version = awinda.firmwareVersion()

        assert (awinda is not 0)
        print("Dispositif: %s, avec ID: %s ouvert. \n" % (awinda.productCode(), awinda.deviceId().toXsString()))


        # Mettre le dispositif en mode configuration
        print("Dispositif en mode configuration...\n")
        if not awinda.gotoConfig():
            raise RuntimeError("Échec. Le dispositif n'a as pu être mis en mode configuration.")

        awinda.setOptions(xda.XSO_Orientation + xda.XSO_Calibrate + xda.XSO_RetainLiveData, 0)

        print("Creating a log file...")
        logFileName = "logfile.mtb"
        if awinda.createLogFile(logFileName) != xda.XRV_OK:
            raise RuntimeError("Failed to create a log file. Aborting.")
        else:
            print("Created a log file: %s" % logFileName)

        # Acquisition des fréquences pour la mise à jour des données
        print("Fréquence d'acquisition choisi: ", updateRate, "\n")

        # Mise à jour de la fréquence d'acquisition
        if not awinda.setUpdateRate(updateRate):
            raise RuntimeError("Échec de la mise à jour du update rate.")

        # Acquisition du canal radio
        print("Canal de radio choisi: ", radioChannel, "\n")

        # Mise à jour du canal radio
        try:
            awinda.enableRadio(radioChannel)
        except ValueError:
            print("La radio est encore activée, veuillez débrancher l'appareil de l'ordinateur et reessayer.")

        input(
            '\n Undock the MTw devices from the Awinda station and wait until the devices are connected (synced leds), then press enter... \n')
        # Vérifier les dispositifs détectées (nombre de capteurs MTw)

        MTws = awinda.children()

        devIdAll = []
        mtwCallbacks = []
        filenamesPCKL = []
        for i in range(len(MTws)):
            devIdAll.append(MTws[i].deviceId())
            mtwCallbacks.append(MTwCallback())
            MTws[i].addCallbackHandler(mtwCallbacks[i])

        devicesUsed, devIdUsed, nDevs = checkConnectedSensors(devIdAll, MTws, controlDev, awinda, Ports)


        MTw_pickle = os.path.dirname(os.path.realpath("__file__")) + "\\MTw Pickle"

        # Set UTC on MTWs
        for n in range(nDevs):
            filePCKL = "PCKL_" + str(devId.toXsString()) + "_" + str(devIdUsed[n]) + ".txt"
            filenamesPCKL.append(os.path.join(MTw_pickle, filePCKL))
            open(filenamesPCKL[-1], 'wb')
            devicesUsed[n].setUtcTime(xda.XsTimeInfo().currentLocalTime())

        ##################

        # Mis les capteurs MTw en mode acquisition des données
        print("Putting devices into measurement mode...\n")

        if not awinda.gotoMeasurement():
            raise RuntimeError("Could not put device into measurement mode. Aborting.")
#

        print("Wait 10 seconds before starting data acquisition.\n")
        time.sleep(10)

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

        for n in range(nDevs):
            devicesUsed[n].flushInputBuffers()
        while keep_going:
            for i in range(len(mtwCallbacks)):
                callback = mtwCallbacks[i]
                filenamePCKL = filenamesPCKL[i]

                if callback.packetAvailable():
                    callback.writeData(filenamePCKL)

        startEnd = xda.XsTimeStamp_nowMs() - startTime
        print("Time of recording (s): ", startEnd / 1000, "\n")
        print("Number of packets that should be acquired by each MTw:", round(updateRate*startEnd / 1000))
        #####

        pickle2txt(devId, devIdAll, devIdUsed, nDevs, firmware_version, filenamesPCKL, updateRate)

        print("Closing log file...")
        if not awinda.closeLogFile():
            raise RuntimeError("Failed to close log file. Aborting.")

        exit_program = int(input('\n Appuyez sur la touche 0 pour quitter le mode acquisition... \n'))
        if exit_program == 0:
            stopAll(awinda, controlDev, Ports)


    except RuntimeError as error:
        print(error)
        sys.exit(1)

    except:
        print("An unknown fatal error has occured. Aborting.")
        sys.exit(1)

    else:
        print("Successful exit.")

####

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-rate', type=int, dest='updateRate', default=40, help='Select an update rate (40 - 120) Hz',
                        required=True)
    parser.add_argument('-radio', type=int, dest='radioChannel', default=11, help='Select a radio Channel between 11 to 25',
                        required=True)
    args = parser.parse_args().__dict__

    updateRate = args['updateRate']
    radioChannel = args['radioChannel']

    main(updateRate, radioChannel)