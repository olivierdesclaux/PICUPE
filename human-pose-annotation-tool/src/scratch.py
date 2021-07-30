import numpy as np
import json
import os
import cv2
import tkinter
import tkinter.ttk as ttk
from tkinter import simpledialog
import argparse

JOINTS = ['SpineBase', 'SpineMid', 'Neck', 'Head',
          'ShoulderLeft', 'ElbowLeft', 'WristLeft', 'HandLeft',
          'ShoulderRight', 'ElbowRight', 'WristRight', 'HandRight',
          'HipLeft', 'KneeLeft', 'AnkleLeft', 'FootLeft',
          'HipRight', 'KneeRight', 'AnkleRight', 'FootRight',
          'SpineShoulder', 'HandTipLeft', 'ThumbLeft', 'HandTipRight',
          'ThumbRight']

# Markers = ['pelvis', 'torso',
#            'humerus_r', 'radius_r',
#            'femur_r', 'tibia_r', 'patella_r', 'talus_r', 'toes_r',
#            'femur_l', 'tibia_l', 'patella_l', 'talus_l', 'toes_l',]

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


class Noter():

    def __init__(self, kpts_out, scale, radius, kpts_in):
        # Visualization variable, radius for circle dimension and scale for image dimension
        self.radius = radius
        self.scale = scale

        # Loading annotation if we're resuming old noting sessions
        self.kpts_in = kpts_in
        self.kpts_out = kpts_out
        self.json_dict = dict()
        if (self.kpts_in is not None) and (os.path.isfile(self.kpts_in)):
            with open(self.kpts_in, 'r') as f:
                self.json_dict = json.load(f)
        else:
            for i, joint in enumerate(JOINTS):
                self.json_dict[joint] = DEFAULT_KPT[0][i]

        # Event variable, for click, modify and add events
        self.is_clicked = False
        self.is_modifying = False
        self.is_adding_joint = False

        # Point position and index variable
        self.point = [-1, -1]
        self.kpt_idx = -1
        self.obj_idx = -1

        # Tkinter attribute for windows management
        self.master = tkinter.Tk()
        self.info = tkinter.StringVar()
        self.error = tkinter.StringVar()
        self.status = tkinter.StringVar()
        self.sequences = tkinter.StringVar()
        self.progressbar = ttk.Progressbar(self.master, orient="horizontal", length=100, mode="determinate")
        self.progressbar.pack(side=tkinter.BOTTOM)
        tkinter.Label(master=self.master, textvariable=self.info).pack()
        tkinter.Label(master=self.master, textvariable=self.error, width=25).pack()
        tkinter.Label(master=self.master, textvariable=self.status).pack()
        tkinter.Label(master=self.master, textvariable=self.sequences).pack()

        # Tkinter initialization
        self.info.set("....")
        self.error.set("....")
        self.status.set("....")
        self.sequences.set("....")

        # Select format for different system
        if os.name == 'nt':
            self.slash = '\\'
        elif os.name == 'posix':
            self.slash = '/'
        else:
            raise NotImplementedError(f"Wrong system, implement different path management: {os.name}")

    def reset(self):
        self.is_modifying = False
        self.is_clicked = False
        self.is_adding_joint = False
        self.obj_idx = -1
        self.kpt_idx = -1
        self.point = [-1, -1]

    @staticmethod
    def draw_kpts(img, kpts, radius):
        for obj in range(kpts.shape[0]):
            for i, el in enumerate(kpts[obj]):
                if i > 20:
                    break
                if el[0] >= 0 and el[1] >= 0:
                    cv2.circle(img, (int(el[0]), int(el[1])), radius, JOINTS_COLOR[i], -1)

    def annotate(self, data_dir):
        img = cv2.imread(os.path.join(data_dir, "RGB_image.png"))
        h, w, _ = img.shape
        depth = cv2.imread(os.path.join(data_dir, "depth_image.png"), cv2.IMREAD_GRAYSCALE)
        name = "keypoints"


        # #Initialise keypoints
        # if self.kpts_in is not None:
        #     kpts = np.array(self.json_dict[name])
        # else:
        #     kpts2 = np.array(DEFAULT_KPT).copy()
        # print(kpts2)
        kpts = np.array([list(self.json_dict.values())])

        # Upscale image for better readability
        # This was in the initial code. Instead of changing everything, I just upscale (and downscale later on) with a scale factor of 1
        img, kpts = self.upscale(img, kpts)

        # Create copy of image and keypoints on which we're going to work
        tmp = img.astype(np.uint8).copy()
        kpts_backup = kpts.copy()

        # Start GUI
        self.draw_kpts(tmp, kpts, self.radius)
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, w, h)
        # cv2.moveWindow(name, 15, 150)
        cv2.setMouseCallback(name, self.click_left, [name, tmp, kpts])
        cv2.imshow(name, tmp)

        while True:
            cv2.imshow(name, tmp)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('\n') or key == ord('\r'):
                _, d_k = self.downscale(kpts=kpts)
                kpts3D = []
                for k in d_k[0]:
                    y, x = k[0], k[1]
                    z = float(depth[int(y), int(x)])
                    kpts3D.append([x,y,z])

                for i, marker in enumerate(JOINTS):
                    self.json_dict[marker] = kpts3D[i]#.tolist()

                cv2.destroyAllWindows()
                with open(self.kpts_out, 'w') as f:
                    json.dump(self.json_dict, f)
                self.reset()
                self.info.set("....")
                self.error.set("....")
                self.master.update()
                break

            # elif key == ord('r'):
            #     tmp = img.astype(np.uint8).copy()
            #     kpts_backup = kpts.copy()
            #     self.draw_kpts(tmp, kpts_backup, self.radius)
            #     self.reset()
            #     cv2.setMouseCallback(name, self.click_left, [name, tmp, kpts])


            # elif key == ord('c'):
            #     with open(self.kpts_out, 'w') as f:
            #         json.dump(self.json_dict, f)
            #     exit(1)

            # elif key == ord('p'):
            #     self.error.set("Changing sequence.")
            #     self.master.update()
            #     next_name = name.split(self.slash)[-3]
            #     self.reset()
            #     break

            # elif key == ord('n'):
            #     tmp[:, :, :] = img.astype(np.uint8).copy()[:, :, :]
            #     np.copyto(kpts, kpts_backup)
            #     self.draw_kpts(tmp, kpts, self.radius)
            #     self.reset()

            if self.is_modifying is not True:
                if key == ord('n'):
                    tmp[:, :, :] = img.astype(np.uint8).copy()[:, :, :]
                    np.copyto(kpts, kpts_backup)
                    self.draw_kpts(tmp, kpts, self.radius)
                    self.reset()

                if key == 27 and self.is_clicked is True:
                    for obj in range(kpts.shape[0]):
                        for el in kpts[obj]:
                            if int(el[0]) == self.point[0] and int(el[1]) == self.point[1]:
                                el[0] = -1
                                el[1] = -1
                                break
                    tmp[:, :, :] = img.astype(np.uint8).copy()[:, :, :]
                    self.draw_kpts(tmp, kpts, self.radius)
                    self.is_clicked = False
                    self.is_modifying = True

                elif key == ord('a') and self.is_clicked is not True:
                    obj = 0
                    val = 0
                    lgnd = ""
                    self.info.set('....')
                    self.error.set('Adding joint.')
                    self.master.update()
                    while self.kpt_idx < 0 and val is not None and obj is not None:
                        for key, v in enumerate(JOINTS):
                            if key > 20:
                                break
                            lgnd += "{} for {}".format(key, v)
                            if key != 20:
                                lgnd += ", "
                                if (key + 1) % 3 == 0:
                                    lgnd += '\n'
                        val = simpledialog.askinteger("Input", "Insert Joint Number\n{}".format(lgnd),
                                                      parent=self.master,
                                                      minvalue=0, maxvalue=25)
                        self.master.update()
                        if val:
                            if kpts.shape[0] > 1:
                                obj = simpledialog.askinteger("Input", "Insert obj number",
                                                              parent=self.master,
                                                              minvalue=0, maxvalue=kpts.shape[0])
                            if obj is not None:
                                self.kpt_idx = val
                                self.obj_idx = obj
                                if self.kpt_idx < 0 or self.kpt_idx > 20:
                                    self.error.set("Insert only value between 0 and 20.")
                                    self.master.update()
                                    self.kpt_idx = -1
                                    continue
                                self.is_adding_joint = True

                elif key == 27 and self.is_clicked is not True:
                    break
            else:
                if key == ord('y') and self.is_modifying is True:
                    self.error.set('....')
                    tmp[:, :, :] = img.astype(np.uint8).copy()[:, :, :]
                    np.copyto(kpts_backup, kpts)
                    self.draw_kpts(tmp, kpts, self.radius)
                    self.reset()

        cv2.destroyAllWindows()
        exit(1)

    def search_near(self, x, y, kpts):
        for obj in range(kpts.shape[0]):
            for i, el in enumerate(kpts[obj]):
                if i > 20:
                    break
                range_x = [el[0] - self.radius + 2, el[0] + self.radius + 2]
                range_y = [el[1] - self.radius + 2, el[1] + self.radius + 2]
                if range_x[0] <= x <= range_x[1] and range_y[0] <= y <= range_y[1]:
                    self.info.set(JOINTS[i])
                    self.master.update()
                    return int(el[0]), int(el[1])
        return -1, -1

    def click_left(self, event, x, y, flags, param):
        name = param[0]
        img = param[1]
        kpts = param[2]
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.is_modifying is False:
                if self.is_clicked is True:
                    self.is_clicked = False
                    self.is_modifying = True
                    old_x, old_y = self.point
                    self.point = [-1, -1]
                    for obj in range(kpts.shape[0]):
                        for el in kpts[obj]:
                            if int(el[0]) == old_x and int(el[1]) == old_y:
                                el[0] = x
                                el[1] = y
                                break
                    cv2.circle(img, (x, y), self.radius, (0, 0, 255), -1)
                    cv2.imshow(name, img)
                elif self.is_adding_joint is True:
                    kpts[self.obj_idx][self.kpt_idx][0] = x
                    kpts[self.obj_idx][self.kpt_idx][1] = y
                    cv2.circle(img, (x, y), self.radius, (0, 255, 255), -1)
                    cv2.imshow(name, img)
                    self.error.set("{} added.".format(JOINTS[self.kpt_idx]))
                    self.master.update()
                    self.is_modifying = True
                    self.is_adding_joint = False
                    self.kpt_idx = -1
                    self.obj_idx = -1
                else:
                    if tuple(img[y, x]) in JOINTS_COLOR:
                        new_x, new_y = self.search_near(x, y, kpts)
                        if new_y > 0 and new_x > 0:
                            cv2.circle(img, (new_x, new_y), self.radius, (0, 0, 255), -1)
                            self.is_clicked = True
                            self.point = [new_x, new_y]
                            cv2.imshow(name, img)
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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--data_dir', type=str, dest='data_dir', help='Data directory.', required=True)
    parser.add_argument('--kpts_in', default=None, help="Pre-saved keypoints")
    parser.add_argument('--kpts_out', type=str, default='../annotations3D.json', dest='kpts_out', help='Output file path')
    parser.add_argument('--scale', type=float, default=1, dest='scale', help='Depth image scale.')
    parser.add_argument('--radius', type=int, default=6, dest='radius', help='Joint annotation radius.')

    args = parser.parse_args().__dict__

    noter = Noter(args['kpts_out'], args["scale"], args["radius"], args["kpts_in"])
    noter.annotate(args['data_dir'])
