import numpy as np
import json
import os
import cv2.cv2 as cv2
import tkinter
import tkinter.ttk as ttk
import argparse
import sys
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

sys.path.append("../../")
from Recalage.stereocalibration import undistort, undistortPoints

JOINTS = ['SpineBase', 'SpineMid', 'Neck', 'Head',
          'ShoulderLeft', 'ElbowLeft', 'WristLeft', 'HandLeft',
          'ShoulderRight', 'ElbowRight', 'WristRight', 'HandRight',
          'HipLeft', 'KneeLeft', 'AnkleLeft', 'FootLeft',
          'HipRight', 'KneeRight', 'AnkleRight', 'FootRight',
          'SpineShoulder']
links = {
    'SpineBase': ['SpineMid', 'HipLeft', 'HipRight'],
    'SpineMid': ['SpineBase', 'SpineShoulder'],
    'SpineShoulder': ['Neck', 'SpineMid', 'ShoulderLeft', 'ShoulderRight'],
    'Neck': ['Head', 'SpineShoulder'],
    'Head': ['Neck'],
    'ShoulderLeft': ['SpineShoulder', 'ElbowLeft'],
    'ElbowLeft': ['WristLeft', 'ShoulderLeft'],
    'WristLeft': ['ElbowLeft', 'HandLeft'],
    'HandLeft': ['WristLeft'],
    'ShoulderRight': ['SpineShoulder', 'ElbowRight'],
    'ElbowRight': ['WristRight', 'ShoulderRight'],
    'WristRight': ['ElbowRight', 'HandRight'],
    'HandRight': ['WristRight'],
    'HipLeft': ['SpineBase', 'KneeLeft'],
    'KneeLeft': ['HipLeft', 'AnkleLeft'],
    'AnkleLeft': ['KneeLeft', 'FootLeft'],
    'FootLeft': ['AnkleLeft'],
    'HipRight': ['SpineBase', 'KneeRight'],
    'KneeRight': ['HipRight', 'AnkleRight'],
    'AnkleRight': ['KneeRight', 'FootRight'],
    'FootRight': ['AnkleRight']
}

DEFAULT_KPT = [[[258.5, 179.5], [258.5, 135.], [255.5, 87.5], [255.5, 71.5],
                [282.5, 100.], [287., 131.5], [286., 161.5], [286., 167.],
                [229., 101.], [227.5, 136.], [230., 168.], [231.5, 173.],
                [278., 181.], [280.5, 231.], [280., 273.5], [282.5, 281.5],
                [239., 182.], [242., 232.5], [244., 275.5], [240., 286.],
                [255.5, 95.5], [322.56, 134.68], [326.76, 131.6], [324.24, 133.], [318.64, 128.8]]]

# Color selection for visualization
JOINTS_COLOR = [(255, 0, 0), (244, 41, 0), (234, 78, 0), (223, 112, 0),
                (213, 142, 0), (202, 168, 0), (192, 191, 0), (151, 181, 0),
                (213, 142, 0), (202, 168, 0), (192, 191, 0), (151, 181, 0),
                (114, 170, 0), (80, 160, 0), (50, 149, 0), (23, 139, 0),
                (114, 170, 0), (80, 160, 0), (50, 149, 0), (23, 139, 0),
                (244, 41, 0)]


class Noter:
    def __init__(self, data_dir, imageNum, scale, radius):
        self.data_dir = data_dir
        if type(imageNum) is not str:
            imageNum = str(imageNum)
            zeroPadding = (5 - len(imageNum)) * '0'  # Number of zeros to add in front of image
            self.imageNum = zeroPadding + imageNum

        # Paths for saving keypoints in different views.
        self.annotations_path = os.path.join(data_dir, "annotations")
        self.annotations_path_rgb = os.path.join(self.annotations_path, "rgb")
        self.annotations_path_flir = os.path.join(self.annotations_path, "flir")
        self.annotations_path_3D = os.path.join(self.annotations_path, "3D")

        # Visualization variable, radius for circle dimension and scale for image dimension
        self.radius = radius
        self.scale = scale

        # Init json dictionaries
        self.json_dict_2D = {}
        self.json_dict_3D = {}
        self.json_dict_2D_IR = {}

        if not os.path.isdir(self.annotations_path):
            os.mkdir(self.annotations_path)
            os.mkdir(self.annotations_path_rgb)
            os.mkdir(self.annotations_path_flir)
            os.mkdir(self.annotations_path_3D)

        # Load existing annotations or set default
        initial_annotations = os.path.join(self.annotations_path_rgb, str(self.imageNum) + ".json")
        if os.path.isfile(initial_annotations):
            with open(initial_annotations, 'r') as f:
                jsonFile = json.load(f)
                self.json_dict_2D = jsonFile
        else:
            for i, joint in enumerate(JOINTS):
                self.json_dict_2D[joint] = DEFAULT_KPT[0][i]

        # Init 2D and 3D keypoint lists.
        self.kpts = np.array([list(self.json_dict_2D.values())])
        self.kpts_backup = self.kpts.copy()
        self.kpts3D = []
        self.kpts_IR = []
        self.segmentLengths = None

        # Init rgb and depth images
        self.depth = None
        self.rgb = None
        self.flir = None

        # Event variable, for click, modify and add events
        self.is_clicked = False
        self.is_modifying = False

        # Point position and index variable
        self.selectedPoint = [-1, -1]
        self.kpt_idx = -1
        self.obj_idx = -1

        # Tkinter attribute for windows management
        self.master = tkinter.Tk()
        self.info = tkinter.StringVar()
        self.error = tkinter.StringVar()
        jointFrame = ttk.LabelFrame(master=self.master, text="Selected Joint")
        jointFrame.grid(column=0, row=0)
        tkinter.Label(master=jointFrame, text="Name").grid(row=0, column=0)
        tkinter.Label(master=jointFrame, textvariable=self.info).grid(row=0, column=1)
        tkinter.Label(master=jointFrame, text="Status").grid(row=1, column=0)
        tkinter.Label(master=jointFrame, textvariable=self.error, width=25).grid(row=1, column=1)

        # Tkinter initialization
        self.info.set("....")
        self.error.set("....")

        # Select format for different system
        if os.name == 'nt':
            self.slash = '\\'
        elif os.name == 'posix':
            self.slash = '/'
        else:
            raise NotImplementedError(f"Wrong system, implement different path management: {os.name}")

        self.master.update()

    def reset(self):
        self.is_modifying = False
        self.is_clicked = False
        self.obj_idx = -1
        self.kpt_idx = -1
        self.selectedPoint = [-1, -1]

    @staticmethod
    def draw_kpts(img, kpts, radius):
        for obj in range(kpts.shape[0]):
            for i, el in enumerate(kpts[obj]):
                if i > 20:
                    break
                if el[0] >= 0 and el[1] >= 0:
                    cv2.circle(img, (int(el[0]), int(el[1])), radius, JOINTS_COLOR[i], -1)

    def annotate(self):
        rgbPath = os.path.join(self.data_dir, "rgb")
        depthPath = os.path.join(self.data_dir, "depth")
        flirPath = os.path.join(self.data_dir, "flir")

        rgb = np.load(os.path.join(rgbPath, self.imageNum))
        depth = np.load(os.path.join(depthPath, self.imageNum), cv2.IMREAD_GRAYSCALE)
        self.flir = np.load(os.path.join(flirPath, self.imageNum))
        self.rgb = rgb
        self.depth = depth

        h, w, c = self.rgb.shape
        if c == 4:
            self.rgb = self.rgb[:, :, :3]

        # Upscale image for better readability
        # This was in the initial code. Instead of changing everything, I just upscale (and downscale later on) with
        # a scale factor of 1
        self.rgb, self.kpts = self.upscale(self.rgb, self.kpts)

        # Create copy of image and keypoints on which we're going to work
        tmp = self.rgb.astype(np.uint8).copy()

        # Start GUI
        name = str(self.imageNum)
        self.draw_kpts(tmp, self.kpts, self.radius)
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, w, h)
        cv2.setMouseCallback(name, self.click_left, [name, tmp, self.kpts])
        cv2.imshow(name, tmp)

        while True:
            cv2.imshow(name, tmp)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('\n') or key == ord('\r'):
                self.write2DJoints()
                self.convertTo3D()
                self.write3DJoints()
                self.annotateIR()
                self.write_IR()
                self.reset()
                self.info.set("....")
                self.error.set("....")
                self.master.update()
                break

            if self.is_modifying is False:
                if key == 27 and not self.is_clicked:
                    print("Exiting without saving.")
                    break
                elif key == 27 and self.is_clicked:
                    self.selectedPoint = [-1, -1]
                    tmp[:, :, :] = self.rgb.astype(np.uint8).copy()[:, :, :]
                    np.copyto(self.kpts_backup, self.kpts)
                    self.draw_kpts(tmp, self.kpts, self.radius)
                    self.is_clicked = False
                    self.is_modifying = False
            else:
                if key == 27:  # User pressed the esc button while modifying a kpt
                    self.selectedPoint = [-1, -1]
                    tmp[:, :, :] = self.rgb.astype(np.uint8).copy()[:, :, :]
                    np.copyto(self.kpts, self.kpts_backup)
                    self.draw_kpts(tmp, self.kpts, self.radius)
                    self.is_clicked = False
                    self.is_modifying = False

                elif key == ord('y'):
                    self.error.set('....')
                    tmp[:, :, :] = self.rgb.astype(np.uint8).copy()[:, :, :]
                    np.copyto(self.kpts_backup, self.kpts)
                    self.draw_kpts(tmp, self.kpts, self.radius)
                    self.reset()

        cv2.destroyAllWindows()
        self.master.destroy()
        return True

    def write2DJoints(self):
        _, d_k = self.downscale(kpts=self.kpts_backup)  # Most recently saved kpts
        d_k = d_k[0]
        for i, marker in enumerate(JOINTS):
            self.json_dict_2D[marker] = list(d_k[i])
        filePath = os.path.join(self.annotations_path_rgb, str(self.imageNum) + ".json")
        with open(filePath, 'w') as f:
            json.dump(self.json_dict_2D, f)

    def convertTo3D(self):
        kInt = np.array([[599.19, 0.0, 633.4154], [0.0, 599.5903, 369.7261], [0.0, 0.0, 1.0]])
        fx = kInt[0, 0]
        fy = kInt[1, 1]
        cx = kInt[0, 2]
        cy = kInt[1, 2]
        kDist = np.array([0.06640906755828462, -0.02978842968405564, 0.002389513299378855, -0.002113879410271612, 0.0])

        _, d_k = self.downscale(kpts=self.kpts_backup)  # Most recently saved kpts
        for k in d_k[0]:
            x, y = k[0], k[1]
            # z = self.depth[int(y), int(x), :]
            z = self.depth[int(y), int(x)]
            z = int(z)  # Convert from uint16 to signed int.

            [[undistorted_x, undistorted_y]] = undistort([np.array([x, y])], kInt, kDist)
            x_3D = ((undistorted_x - cx) / fx * z)
            y_3D = ((undistorted_y - cy) / fy * z)
            self.kpts3D.append([x_3D, y_3D, z])

    def write3DJoints(self):
        for i, marker in enumerate(JOINTS):
            self.json_dict_3D[marker] = self.kpts3D[i]
        filePath = os.path.join(self.annotations_path_3D, str(self.imageNum) + ".json")
        with open(filePath, 'w') as f:
            json.dump(self.json_dict_3D, f)

    def viz3D(self):
        if self.segmentLengths is None:
            print("Computing segment lengths...")
            self.computeSegmentLengths()
            print("Done.")

        fig1 = plt.figure(figsize=(15, 8))
        axRGB = fig1.add_subplot(1,2,1)
        self.draw_kpts(self.rgb, self.kpts, self.radius)
        # rgb = cv2.resize(self.rgb, None, fx=1.5, fy=1.5)
        axRGB.imshow(cv2.cvtColor(self.rgb, cv2.COLOR_BGR2RGB))

        axIR = fig1.add_subplot(1, 2, 2)
        # flir = cv2.resize(self.flir, None, fx=1.5, fy=1.5)
        axIR.imshow(cv2.cvtColor(self.flir, cv2.COLOR_BGR2RGB))

        fig2 = plt.figure(figsize=(15, 8))
        # 2D plot with distances
        ax2D = fig2.add_subplot(1, 2, 1)

        # 3D plot
        ax3D = fig2.add_subplot(1, 2, 2, projection='3d')
        d = self.json_dict_3D
        xdata = [d[x][0] for x in d.keys()]
        ydata = [-d[x][1] for x in d.keys()]  # Because openCV has 0 starting at the top
        zdata = [d[x][2] for x in d.keys()]

        ax3D.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')

        for jointName in links.keys():
            jointCoords = d[jointName]
            connectedJoints = links[jointName]
            for joint2Name in connectedJoints:
                joint2Coords = d[joint2Name]
                x1, y1, z1 = jointCoords
                y1 = - y1
                x2, y2, z2 = joint2Coords
                y2 = -y2
                # ax3D.plot([jointCoords[0], joint2Coords[0]], [jointCoords[1], joint2Coords[1]], [jointCoords[2],
                #                                                                                joint2Coords[2]])
                ax3D.plot([x1, x2], [y1, y2], [z1, z2])

                ax2D.plot([x1, x2], [y1, y2])
                if (jointName + '-' + joint2Name) in self.segmentLengths.keys():
                    length = self.segmentLengths[jointName + '-' + joint2Name]
                elif (joint2Name + '-' + jointName) in self.segmentLengths.keys():
                    length = self.segmentLengths[joint2Name + '-' + jointName]
                else:
                    raise ValueError("Oups")
                ax2D.annotate(str(round(length/10, 1)), xy=((x1 + x2) / 2, (y1 + y2) / 2), xytext=(-1, 1),
                              textcoords='offset pixels')
        plt.tight_layout()
        plt.show()

    def computeSegmentLengths(self):
        distances = {}
        for joint in links.keys():
            jointCoords = np.array(self.json_dict_3D[joint])
            for joint2 in links[joint]:
                if (joint + '-' + joint2) in distances.keys() or (joint2 + '-' + joint) in distances.keys():
                    continue
                else:
                    joint2Coords = np.array(self.json_dict_3D[joint2])
                    distances[joint + '-' + joint2] = np.linalg.norm(joint2Coords - jointCoords)
        self.segmentLengths = distances
        return distances

    # def annotateIR(self):
    #     calibFiles = os.path.join(self.data_dir, "calib")
    #     with open(os.path.join(calibFiles, "stereo.json")) as f:
    #         stereo = json.load(f)
    #
    #     M1 = np.array(stereo["Matrix1"])  # Left Matrix
    #     D1 = np.array(stereo["Dist1"])  # Left Dist
    #     M2 = np.array(stereo["Matrix2"])  # Right Matrix
    #     D2 = np.array(stereo["Dist2"])  # Right Dist
    #     R = np.array(stereo["R"])
    #     T = np.array(stereo["T"])
    #     # STEREO RECTIFICATION
    #     R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(M1, D1, M2, D2, (1024, 768), R, T, alpha=1,
    #                                                       flags=cv2.CALIB_ZERO_DISPARITY)
    #
    #     new_M1, _ = cv2.getOptimalNewCameraMatrix(M1, D1, (1024, 768), 1, (1024, 768))
    #     new_M2, _ = cv2.getOptimalNewCameraMatrix(M2, D2, (1280, 720), 1, (1280, 720))
    #     print(new_M1)
    #     print(new_M2)
    #     # COMPUTE UNDISTORTION+RECTIFICATION MAPS
    #     leftMap1, leftMap2 = cv2.initUndistortRectifyMap(new_M1, D1, R1, P1, (1024, 768),
    #                                                      cv2.CV_32FC1)
    #     rightMap1, rightMap2 = cv2.initUndistortRectifyMap(new_M2, D2, R2, P2, (1024, 768),
    #                                                        cv2.CV_32FC1)
    #     flir_rectified = cv2.remap(self.flir, leftMap1, leftMap2, interpolation=cv2.INTER_LINEAR)
    #     rgb_rectified = cv2.remap(self.rgb, rightMap1, rightMap2, interpolation=cv2.INTER_LINEAR)
    #
    #     flir_undistorted = cv2.undistort(self.flir, M1, D1)
    #     rgb_undistorted = cv2.undistort(self.rgb, M2, D2)
    #
    #     # Undistort checkerboard centers
    #     kpts_2D_undistorted = undistortPoints(self.kpts[0], M2, D2)
    #     kpts_rectified = []
    #     for k in self.kpts[0]:
    #     # for k in kpts_2D_undistorted.squeeze():
    #         y, x = k
    #         y, x = int(y), int(x)
    #         new_x = rightMap1[y, x]
    #         new_y = rightMap2[y, x]
    #         kpts_rectified.append([new_y, new_x])
    #     kpts_rectified = np.array([kpts_rectified])
    #     self.draw_kpts(rgb_rectified, kpts_rectified, self.radius)
    #     while True:
    #         cv2.imshow("RGB rectified", rgb_rectified)
    #         cv2.imshow("FLir rectified", flir_rectified)
    #
    #         if cv2.waitKey(10) == ord("q"):
    #             break
    #     cv2.destroyAllWindows()
    #     return True

    def annotateIR(self):
        calibFiles = os.path.join(self.data_dir, "calib")
        with open(os.path.join(calibFiles, "stereo.json")) as f:
            stereo = json.load(f)

        M1 = np.array(stereo["Matrix1"])  # Flir Matrix
        D1 = np.array(stereo["Dist1"])  # Flir Dist

        R = np.array(stereo["R"])
        T = np.array(stereo["T"])
        k1, k2, p1, p2, k3 = D1
        kpts_3D_IRView = np.array(self.kpts3D)

        # Reformat T and R matrix
        Ts = np.tile(T, (1, len(kpts_3D_IRView)))
        Ts = Ts.T
        R_inv = np.linalg.inv(R)

        # Convert 3D skel. in Kinect view to IR view.
        kpts_3D_IRView = kpts_3D_IRView - Ts
        kpts_3D_IRView = np.matmul(R_inv, kpts_3D_IRView.T)

        # Scale by Z, i.e. (X, Y, Z) becomes (X/Z, Y/Z, 1)
        Zs = kpts_3D_IRView[2, :]
        for i in range(kpts_3D_IRView.shape[0]):
            kpts_3D_IRView[i, :] = kpts_3D_IRView[i,:]/Zs
        kpts_2D_IRView_basic = np.matmul(M1, kpts_3D_IRView)

        # Equations taken from OReilly learn openCV p376 And implementation can also be found in openCV source code
        # for undistort:
        # https://github.com/opencv/opencv_attic/blob/a6078cc8477ff055427b67048a95547b3efe92a5/opencv/modules/imgproc/src/undistort.cpp
        # Here the difference is that we "distort" artificially. We do the same thing as in openCV source code,
        # but the other way round.

        Xs = kpts_3D_IRView[0, :]
        Ys = kpts_3D_IRView[1, :]
        X0 = Xs
        Y0 = Ys
        for j in range(5):
            r2 = Xs**2 + Ys**2
            scale = (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3)
            delta_x = 2 * p1 * Xs * Ys + p2 * (r2 + 2 * Xs ** 2)
            delta_y = p1 * (r2 + 2 * Ys ** 2) + 2 * p2 * Xs * Ys

            Xs = X0 * scale + delta_x
            Ys = Y0 * scale + delta_y
        kpts_3D_IRView[0, :] = Xs
        kpts_3D_IRView[1, :] = Ys
        # Project points back in 2D with the flir matrix
        kpts_2D_IRView = np.matmul(M1, kpts_3D_IRView)
        kpts_2D_IRView = kpts_2D_IRView[:2, :]  # Go from homogeneous back to normal

        kpts_2D_IRView = kpts_2D_IRView.T
        kpts_2D_IRView = np.expand_dims(kpts_2D_IRView, axis=0)

        self.draw_kpts(self.flir, kpts_2D_IRView, self.radius)
        self.kpts_IR = kpts_2D_IRView

    def write_IR(self):
        kpts_IR = self.kpts_IR[0]
        for i, marker in enumerate(JOINTS):
            self.json_dict_2D_IR[marker] = list(kpts_IR[i])
        filePath = os.path.join(self.annotations_path_rgb, str(self.imageNum) + ".json")
        with open(filePath, 'w') as f:
            json.dump(self.json_dict_2D, f)

    def search_near(self, x, y, kpts):
        for obj in range(kpts.shape[0]):
            for i, el in enumerate(kpts[obj]):
                range_x = [el[0] - self.radius + 2, el[0] + self.radius + 2]
                range_y = [el[1] - self.radius + 2, el[1] + self.radius + 2]
                if range_x[0] <= x <= range_x[1] and range_y[0] <= y <= range_y[1]:
                    self.info.set(JOINTS[i])
                    self.master.update()
                    return int(el[0]), int(el[1])
        return -1, -1

    def click_left(self, event, x, y, flags, param):
        name = param[0]
        rgb = param[1]
        kpts = param[2]
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.is_modifying is False:
                if self.is_clicked is True:
                    # If a point is already selected
                    self.is_clicked = False
                    self.is_modifying = True
                    old_x, old_y = self.selectedPoint
                    self.selectedPoint = [-1, -1]
                    for obj in range(kpts.shape[0]):
                        for el in kpts[obj]:
                            if int(el[0]) == old_x and int(el[1]) == old_y:
                                el[0] = x
                                el[1] = y
                                break
                    cv2.circle(rgb, (x, y), self.radius, (0, 0, 255), -1)
                    cv2.imshow(name, rgb)
                else:
                    # If a point was never selected. We color it in red and save it in self.point
                    if tuple(rgb[y, x]) in JOINTS_COLOR:
                        new_x, new_y = self.search_near(x, y, kpts)
                        if new_y > 0 and new_x > 0:
                            cv2.circle(rgb, (new_x, new_y), self.radius, (0, 0, 255), -1)
                            self.is_clicked = True
                            self.selectedPoint = [new_x, new_y]
                            cv2.imshow(name, rgb)
            else:
                self.error.set("Confirm modfying?")
                self.master.update()

    @staticmethod
    def __resize(img, kpts, scale):
        up = up_k = None
        if img is not None:
            up = cv2.resize(img.copy(), None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        if kpts is not None:
            up_k = kpts.copy()
            for obj in range(up_k.shape[0]):
                for el in up_k[obj]:
                    if el[0] > 0 and el[1] > 0:
                        el[0] *= scale
                        el[1] *= scale
        return up, up_k

    def upscale(self, img=None, kpts=None):
        return self.__resize(img, kpts, self.scale)

    def downscale(self, img=None, kpts=None):
        return self.__resize(img, kpts, 1 / self.scale)

def json_to_trc(json_path, template_trc_path, target_trc_path):
    with open(json_path, 'r') as f:
        json_dict = json.load(f)
    markers = list(json_dict.keys())

    with open(template_trc_path, 'r') as f:
        lines = f.readlines()
        target_file_content = lines.copy()

    new_header = "Frame#" + "\t" + "Time" + "\t"
    new_coordinates = "\t\t"
    new_values = "1" + "\t" + "0.000000" + "\t"

    for i, marker in enumerate(markers):
        # Update header
        new_header += marker
        new_header += "\t\t\t"

        # Update coordinate names
        index = str(i + 1)
        x = "X" + index
        y = "Y" + index
        z = "Z" + index
        new_coordinates = new_coordinates + x + "\t" + y + "\t" + z + "\t"

        # Update coordinate names
        x_val, y_val, z_val = json_dict[marker]
        new_values += str(x_val) + "\t" + str(y_val) + "\t" + str(z_val) + "\t"

    # Add new line to all the end of lines
    new_header += "\n"
    new_coordinates += "\n"
    new_values += "\n"

    target_file_content[3] = new_header
    target_file_content[4] = new_coordinates
    target_file_content[6] = new_values

    if os.path.exists(target_trc_path):
        os.remove(target_trc_path)
    with open(target_trc_path, 'a') as w:
        w.writelines(target_file_content)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir', type=str, dest='data_dir', help='Data directory.', required=False)
    parser.add_argument('--scale', type=float, default=1, dest='scale', help='Depth image scale.')
    parser.add_argument('--radius', type=int, default=6, dest='radius', help='Joint annotation radius.')

    args = parser.parse_args().__dict__

    data_dir = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\sandbox\results\Oliv mille - 22 mars"
    noter = Noter(data_dir, 200, args["scale"], args["radius"])
    noter.annotate()

    # template_trc = "../../openSim/scaling/template_static_scale.trc"
    # target_trc = "../output/custom.trc"
    # json_to_trc(args['kpts2D_outputPath'], template_trc, target_trc)
    noter.viz3D()

