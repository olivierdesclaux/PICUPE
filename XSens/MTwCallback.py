import xsensdeviceapi as xda
from threading import Lock
import pickle

class MTwCallback(xda.XsCallback):
    """
    XsCallback super class that manages data packets transmission between the MTw devices and the awinda station
    while on recording mode. Also, we can add events and flags to enhance data transmission of packets.

    Parameters
    ----------

    Returns
    -------
    None.
    """

    def __init__(self, maxBufferSize):
        """
        Initialise the MTwCallback
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
        """
        Event that verifies that a data packet was receive by the device before transmission to the master device
        Parameters
        ----------
        None.
        Returns
        -------
        res : bool
            True if the data packet is available on the buffer of the device
        """
        self.m_lock.acquire()
        res = len(self.m_packetBuffer) > 0
        self.m_lock.release()
        return res

    def onLiveDataAvailable(self, dev, packet):
        """

        Parameters
        ----------
        dev
        packet

        Returns
        -------

        """
        self.m_lock.acquire()
        while len(self.m_packetBuffer) >= self.m_maxNumberOfPacketsInBuffer:
            self.m_packetBuffer.pop()
        self.m_packetBuffer.append(xda.XsDataPacket(packet))
        self.m_lock.release()

    def writeData(self, filename):
        """
        Event that writes the data packet receive by the device to a pickle txt file
        Parameters
        ----------
        filename : string
            directory of the pickle txt file in MTw Pickle folder
        Returns
        -------
        None.
        """
        self.m_lock.acquire()
        oldest_packet = xda.XsDataPacket(self.m_packetBuffer.pop(0))
        with open(filename, "ab") as file_handle:
            pickle.dump((oldest_packet.packetCounter(), oldest_packet.calibratedAcceleration(),
                         oldest_packet.orientationMatrix(),
                         oldest_packet.timeOfArrival().utcToLocalTime().toXsString().__str__()),
                        file_handle)
        self.m_lock.release()