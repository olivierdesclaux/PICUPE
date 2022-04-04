import opensim as osim
import os
from osimUtils import IMUDataConverter, IMUPlacer, IMUTracker
from IMUMappingsXMLGenerator import makeIMUPlacer_xml
from time import sleep
import numpy as np
from distutils.dir_util import copy_tree
from osim23D import computeMarker3D
import sys

sys.path.append("..")
from utils.timestampMappers import mapMot2Frame
from utils.viz3D import viz3D


def main(experimentDir):
    # Initialise variables
    imuDataFolder = os.path.join(experimentDir, "XSens", "MTw data")
    imuDataFolder += "/"

    osimFolder = os.path.join(experimentDir, "openSim")
    imuMappings = os.path.join(osimFolder, "IMUMappings.xml")

    if not os.path.isfile(imuMappings):
        print("Generating IMUMappings.xml ...")
        imuMappings = makeIMUPlacer_xml(experimentDir)

    print(imuMappings)
    model = r'C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\openSim\OpenSense\Rajagopal_2015_calibrationPose.osim'

    if not os.path.isdir(osimFolder):
        os.mkdir(osimFolder)
    if not os.path.isdir(os.path.join(osimFolder, "Geometry")):
        copy_tree(r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\openSim\config\Geometry", os.path.join(osimFolder,
                                                                                                           "Geometry"))
    osimFolder += "/"

    # Main architecture for integrating IMU data
    # 1. Convert IMU txt data to .sto
    print("Converting IMU txt data to .sto ...")
    imuDC = IMUDataConverter(imuDataFolder, imuMappings, osimFolder)
    path2orientations = imuDC.convertData()
    print(path2orientations)

    while not os.path.isfile(path2orientations):
        sleep(0.1)

    print("Scaling model ...")
    # 2. Scale our model
    # 2.1. Annotate RGB Image and save 3D keypoints to json
    # 2.2. Convert json to .trc
    # 2.3. Call IK for scaling
    # Here we will suppose that our model was scaled
    scaledModel = model

    print("Placing IMUs ...")
    # 3. Place IMUs on the skeleton i.e. IMU Calibration
    # for x in [-np.pi/2, 0, np.pi/2]:
    #     for y in [-np.pi / 2, 0, np.pi / 2]:
    #         for z in [-np.pi / 2, 0, np.pi / 2]:
    #             sensor2osim = osim.Vec3(x, y, z)
    #             baseIMUHeading = "z"
    #             calibrator = modelCalibrator(scaledModel, path2orientations, osimFolder, sensor2osim=sensor2osim,
    #                                          baseIMUHeading=baseIMUHeading)
    #             calibrator.visualizeCalibration = False
    #             calibrated = calibrator.run()
    # sensor2osim = osim.Vec3(0, np.pi, np.pi/2)
    # baseIMUHeading = "z"
    sensor2osim = osim.Vec3(-np.pi / 2, 0, 0)
    baseIMUHeading = "z"
    placer = IMUPlacer(scaledModel, path2orientations, osimFolder, sensor2osim=sensor2osim,
                       baseIMUHeading=baseIMUHeading)
    placer.visualizeCalibration = True
    calibratedModel = placer.run()

    print("Integrating data...")
    # 4. IMU Integration
    startTime = 0
    endTime = 50
    tracker = IMUTracker(calibratedModel, path2orientations, startTime, endTime, osimFolder, sensor2osim=sensor2osim)
    tracker.visualizeTracking = True
    tracker.run()

    # 5. Extract 3D marker movement from osim orientations
    computeMarker3D(model, experimentDir)

    # 6. Map the .mot timestamps to frames
    # timestampMappers.mapMot2Frame(experimentDir)
    mapMot2Frame(experimentDir)

    # 7. Visualize results
    viz3D(experimentDir)
    return True


if __name__ == '__main__':
    # path2Exp = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Results\Acquisitions\OlivierNoHeadReset"
    path2Exp = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results\Oliv mille - 22 mars"
    main(path2Exp)
