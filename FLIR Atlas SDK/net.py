# Import modules
import clr
import sys
import time
sys.path.append(r"C:\Program Files (x86)\FLIR Systems\FLIR Atlas SDK 6\bin\x64")  # path of dll
clr.AddReference("Flir.Atlas.Live")
clr.AddReference("System")
clr.AddReference("System.Threading")
clr.AddReference("System.Windows.Forms")
import System
from Flir.Atlas.Live import Device, Recorder, Discovery


def main():
    # camera = Device.CameraBase()
    # camera.Connect()
    # camera = Device.ThermalCamera(0)
    # print(camera)
    # if camera is None:
    #
    #     raise Exception("Couldn't open Camera")
    # print(camera.CameraDeviceInfo())
    # rec = Recorder.VideoRecorder.Start('./Results')
    # print(rec.toString())
    # # rec.Start('./Results/')
    # time.sleep(5)
    # rec.Stop()

    disc = Discovery.Discovery()
    disc.Start()
    print(disc.Dispose())
    event = disc.DeviceFound
    print(event)
    # print(event.ErrorMessage)
    # ShowError(event)
    # System.ShowError(event)
    return True

if __name__ == '__main__':
    main()

