# -*- coding: utf-8 -*-
"""
pythonnet test
hopefully useful for Atlas SDK
"""

import clr  # pythonnet module
import time
from matplotlib import pyplot as plt
from PIL import Image
import sys

sys.path.append(r"C:\Program Files (x86)\FLIR Systems\FLIR Atlas SDK 6\bin\x64")  # path of dll
clr.AddReference("Flir.Atlas.Live")
clr.AddReference("FLir.Atlas.Image")
# clr.AddReference(r"C:\Users\Administrator\Documents\Code\Atlas pythonnet\Flir.Atlas.Live.dll")
# clr.AddReference(r"C:\Users\Administrator\Documents\Code\Atlas pythonnet\Flir.Atlas.Image.dll")


from Flir.Atlas.Live import Device, Discovery
from Flir.Atlas.Image import ThermalImage, Palettes

# %% discovering cameras

disc = Discovery.Discovery()

disc_cameras = []


def device_found(sender, event):
    camera_info = event.CameraDevice
    disc_cameras.append(camera_info)
    print(camera_info.Name + " " + camera_info.SerialNumber)


disc.DeviceFound += device_found

# disc.Start(Discovery.Interface.Network)
disc.Start()
disc.Dispose()

for d in disc_cameras:
    try:
        print(b)
        b = d.IsFlirCamera
    except:
        device = d
        break
print(device)
device = disc_cameras[1]

# %% connecting a camera

# cam = Device.ThermalCamera()  # Or VideoOverlay
# device.SelectedStreamingFormat = Discovery.ImageFormat.FlirFileFormat  # device.StreamingFormats[0]
# print(device.SelectedStreamingFormat)  # Enumeration 0 or 1. 0 is FLirFileFOrmat (thermal matrix) 1 is RGB
# print("SUpported formatss", device.StreamingFormats)

cam = Device.VideoOverlayCamera()
cam.Connect(device)
cam.StartGrabbing()
# %% grab a single image

print("Connecting...")
while not cam.IsGrabbing:
    time.sleep(.1)
print("Starting Acquisition...")

# print("Bla")
# print("Is grabbing", cam.IsGrabbing)
im = cam.GetImage()
print(im)
print(im.Size)
#%% edit the image

# im.Palette = Palettes.PaletteManager.Iron
x = im.ImageArray
print(type(x))
print(x[0])
# plt.imshow(im.Bitmap)
# save the image
# print(im.Bitmap)
#
# path2im = r"C:\Users\Olivier Desclaux\OneDrive\Recherche\PICUPE\FLIR Atlas SDK\image.jpg"
# im.SaveSnapshot(path2im)
# #
# # # show the image
# #
# #
# img = Image.open(path2im)
# plt.imshow(img)

# %% disconnect the camera
cam.StopGrabbing()
cam.Disconnect()
