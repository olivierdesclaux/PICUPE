import os
import lxml.etree as ET

IMUMappings = {
    "head_imu": "00B48765",
    "torso_imu": "00B4875B",
    "pelvis_imu": "00B48772",
    "humerus_l_imu": "00B4875C",
    "humerus_r_imu": "00B48770",
    "ulna_l_imu": "00B487B6",
    "ulna_r_imu": "00B4876D",
    "hand_l_imu": '00B48784',
    "hand_r_imu": "00B4875D",
    "femur_l_imu": "00B48773",
    "femur_r_imu": "00B4876F",
    "tibia_l_imu": "00B48769",
    "tibia_r_imu": "00B4876C",
    "calcn_l_imu": "00B48768",
    "calcn_r_imu": "00B48761",
    "Right Shoulder": "00B48760",
    "Left Shoulder": "00B4876E"
}


def makeIMUPlacer_xml(path2Exp):
    """
    Function for generating an .xml file with the mappings of imu addresses to imu names. Reads the data in MTw data
    dir of our experiment, and writes as many mappings as there are IMU .txt files.
    Template can be find in openSim/xml files/IMUMappings.xml
    Parameters
    ----------
    path2Exp: str, path to the experiment.

    Returns
    -------
    res: str, path to the generated .xml
    """
    path2MTw = os.path.join(path2Exp, "XSens", "MTw data")
    if not os.path.isdir(path2MTw):
        raise ValueError("No MTw data found.")

    save_path = os.path.join(path2Exp, "openSim")
    if not os.path.isdir(save_path):
        os.mkdir(save_path)

    imuList = os.listdir(path2MTw)  # Extract all imu files.
    # We have to split the IMU names from the station name (i.e. the trial prefix)
    trial_prefix = imuList[0].split("_")
    trial_prefix = "_".join(trial_prefix[:-1])

    imuList = [x.split("_")[-1] for x in imuList]
    imuList = [x.split(".")[0] for x in imuList]  # typical output: ['00B4875B', '00B4875C', ...]

    ROOT = ET.Element("OpenSimDocument", Version="40000")
    SETTINGS = ET.Element("XsensDataReaderSettings")
    TRIAL_PREFIX = ET.Element("trial_prefix")
    TRIAL_PREFIX.text = trial_prefix
    EXPERIMENTAL_SENSORS = ET.Element("ExperimentalSensors")

    for imu in imuList:
        EXPERIMENTAL_SENSOR = ET.Element("ExperimentalSensor", name="_" + imu)
        # Have to add an underscore in front of each imu name (xml format for opensim).
        NAME_IN_MODEL = ET.Element("name_in_model")
        NAME_IN_MODEL.text = list(IMUMappings.keys())[list(IMUMappings.values()).index(imu)]
        EXPERIMENTAL_SENSOR.append(NAME_IN_MODEL)
        EXPERIMENTAL_SENSORS.append(EXPERIMENTAL_SENSOR)

    SETTINGS.append(TRIAL_PREFIX)
    SETTINGS.append(EXPERIMENTAL_SENSORS)
    ROOT.append(SETTINGS)

    xml_object = ET.tostring(ROOT,
                             pretty_print=True,
                             xml_declaration=True,
                             encoding='UTF-8')

    with open(os.path.join(save_path, "IMUMappings.xml"), "wb") as writer:
        writer.write(xml_object)

    res = os.path.join(save_path, "IMUMappings.xml")
    return res


if __name__ == "__main__":
    data = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results\Oliv mille - 22 mars"
    makeIMUPlacer_xml(data)
