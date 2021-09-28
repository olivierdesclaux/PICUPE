import opensim as osim
import os
from utils import IMUDataConverter
from time import sleep


def main():
    # Initialise variables
    imuDataFolder = '../OpenSense/IMUData/'
    imuMappings = '../OpenSense/myIMUMappings.xml'
    model = '../OpenSense/Rajagopal_2015.osim'
    # Main architecture for integrating IMU data

    # 1. Convert IMU txt data to .sto
    imuDC = IMUDataConverter(imuDataFolder, imuMappings)
    imuDC.convertData()

    while not os.path.isfile('../config/MT_012005D6_009-001_angularVelocities.sto'):
        sleep(0.1)

    # 2. Scale our model
    # 2.1. Annotate RGB Image and save 3D keypoints to json
    # 2.2. Convert json to .trc
    # 2.3. Call IK for scaling


    # 3. Place IMUs on the skeleton

    # 4. Ca
    # We first need to scale our model
    # Then integrate it
    return 0


if __name__ == '__main__':
    main()
