import opensim as osim
import os
import argparse


class IMUDataConverter:
    """
    Class for converting IMU data from the XSens devices to .Sto files for OpenSenses integration
    """

    def __init__(self, imuDataFolder, imuMappings, outputDir='../config/'):
        """
        :param imuDataFolder: path to folder containing the .txt imu files. ex: 'IMUdata/'
        :param imuMappings: path to xml file containing containing IMU setup ex:'IMUMappings.xml'
        """
        self.imuDataFolder = imuDataFolder
        self.imuMappings = imuMappings
        self.outputDir = outputDir
        self.xsensSettings = osim.XsensDataReaderSettings(self.imuMappings)
        self.xsens = osim.XsensDataReader(self.xsensSettings)

    def convertData(self):
        # Read in seprate tables of data from the specified IMU file(s)
        tables = self.xsens.read(self.imuDataFolder)
        # get the trial name from the settings
        trial = self.xsensSettings.get_trial_prefix()
        trial = os.path.join(self.outputDir, trial)

        # Get Orientation Data as quaternions
        quatTable = self.xsens.getOrientationsTable(tables)
        # Write to file
        osim.STOFileAdapterQuaternion.write(quatTable, trial + '_orientations.sto')
        # Get Acceleration Data
        accelTable = self.xsens.getLinearAccelerationsTable(tables)
        # Write to file
        osim.STOFileAdapterVec3.write(accelTable, trial + '_linearAccelerations.sto')
        # Get Magnetic (North) Heading Data
        magTable = self.xsens.getMagneticHeadingTable(tables)
        # Write to file
        osim.STOFileAdapterVec3.write(magTable, trial + '_magneticNorthHeadings.sto')
        # Get Angular Velocity Data
        angVelTable = self.xsens.getAngularVelocityTable(tables)
        # Write to file
        osim.STOFileAdapterVec3.write(angVelTable, trial + '_angularVelocities.sto')



class CalibrateModel:
    def __init__(self):
        self.IMUPlacer = osim.IMUPlacer()




def main(imuDataFolder, imuMappings):
    imuDC = IMUDataConverter(imuDataFolder, imuMappings)
    imuDC.convertData()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, dest='imuDataFolder', help='Path to directory containing imu .txt files',
                        required=True)
    parser.add_argument('--mappings', type=str, dest='imuMappings', help='.xml file ith IMU Mappings', required=True)
    args = parser.parse_args().__dict__
    imuDataFolder, imuMappings = args["imuDataFolder"], args["imuMappings"]
    main(imuDataFolder, imuMappings)
