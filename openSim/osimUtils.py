import opensim as osim
import os
import argparse
import shutil
import numpy as np


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
        """
        Converts data from txt files in a directory to various time series tables (as .sto files).
        Returns
        -------
        res: str, path to the orientations.sto file
        """
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

        res = os.path.join(self.outputDir, trial + '_orientations.sto')
        return res


class IMUPlacer:
    """
    Class for placing IMUs on an osim model, using the osim imuPlacer method.
    """
    def __init__(self, scaledModel, orientations, outputDir, sensor2osim=osim.Vec3(-1.570796, 0, 0),
                 baseIMUName='pelvis_imu',
                 baseIMUHeading='z'):
        self.scaledModel = scaledModel  # Path to .osim scaled model
        self.scaledModelName = os.path.split(self.scaledModel)[-1]
        self.orientations = orientations  # Path to orientations.sto file
        self.outputDir = outputDir  # Path to directory for saving results
        self.sensor2osim = sensor2osim  # The rotation of IMU data to the OpenSim world frame
        self.baseIMUName = baseIMUName  # The base IMU is the IMU on the base body of the model that dictates the heading (
        # forward) direction of the model.
        self.baseIMUHeading = baseIMUHeading  # The Coordinate Axis of the base IMU that points in the heading direction.
        self.visualizeCalibration = False  # Boolean to Visualize the Output model
        self.imuPlacer = osim.IMUPlacer()

    def run(self):
        """
        Places imu on a model (with or without visualisation of the results) and saves it
        Returns
        -------
        savePath: str, path to the calibrated .osim model.
        """
        self.imuPlacer.set_model_file(self.scaledModel)
        self.imuPlacer.set_orientation_file_for_calibration(self.orientations)
        self.imuPlacer.set_sensor_to_opensim_rotations(self.sensor2osim)
        self.imuPlacer.set_base_imu_label(self.baseIMUName)
        self.imuPlacer.set_base_heading_axis(self.baseIMUHeading)

        self.imuPlacer.run(self.visualizeCalibration)

        # Get the model with the calibrated IMU
        model = self.imuPlacer.getCalibratedModel()
        model.setUseVisualizer(self.visualizeCalibration)

        # Print the calibrated model to file.
        # We want to save the heading direction and the vec3 used.
        saveParameters = "_".join([str(int(x)) for x in np.degrees(self.sensor2osim.to_numpy())] + [
            self.baseIMUHeading])
        saveName = 'calibrated_' + saveParameters + "_" + self.scaledModelName
        model.printToXML(saveName)  # Save the model. But saving is made in local directory by the .osim function.
        # So we move the file to the desired openSim folder of our experiment.
        savePath = os.path.join(self.outputDir, saveName)
        shutil.move(saveName, savePath)

        return savePath

class IMUTracker:
    """
    Implements the opensim IMUInverseKinematics function.
    """
    def __init__(self, calibratedModel, orientations, startTime, endTime, resultsDir, sensor2osim=osim.Vec3(-1.570796, 0, 0)):
        self.calibratedModel = calibratedModel
        self.orientations = orientations
        self.startTime = startTime
        self.endTime = endTime
        self.resultsDir = resultsDir
        self.sensor2osim = sensor2osim
        self.visualizeTracking = True
        self.IKTool = osim.IMUInverseKinematicsTool()

    def run(self):
        """
        Computes inverse kinematics on the imu data stored in the .sto file between timestamps self.startTime and
        self.endTime. Saves results in a .mot file.
        Returns
        -------
        True when finished. 
        """
        # Set tool properties
        self.IKTool.set_model_file(self.calibratedModel)
        self.IKTool.set_orientations_file(self.orientations)
        self.IKTool.set_sensor_to_opensim_rotations(self.sensor2osim)
        self.IKTool.set_results_directory(self.resultsDir)

        # Set time range in seconds
        self.IKTool.set_time_range(0, self.startTime)
        self.IKTool.set_time_range(1, self.endTime)

        # Run IK
        self.IKTool.run(self.visualizeTracking)

        return True


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
