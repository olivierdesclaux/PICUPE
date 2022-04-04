import numpy as np
import cv2.cv2 as cv2
import os
import matplotlib.pyplot as plt
from moviepy.editor import ImageSequenceClip
from circledetector import CircleDetector
from cameraUtils import openCalibrationFile


def main():
    threshold = 3
    boardSize = (9, 15)
    circleDetector = CircleDetector()
    dataPath = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\multiSetStereoImages2"
    positions3D = np.load(os.path.join(dataPath, "positions3D.npy"))
    leftMatrix, leftDist = openCalibrationFile(os.path.join(dataPath, "flirCalib.json"))
    rightMatrix, rightDist = openCalibrationFile(r"C:\Users\Recherche\OneDrive - "
                                                 r"polymtl.ca\PICUPE\Results\Calibration "
                                                 r"Directories\test\kinectCalib.json")
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    rets = []
    Ts = []
    epipolarConstraintErrors = []
    sets = range(1, 6)

    for setNum in sets:
        SUCCESS = False

        print("Set: {}".format(setNum))
        leftSet = "leftSet" + str(setNum) + ".npy"
        rightSet = "rightSet" + str(setNum) + ".npy"
        leftIms = np.load(os.path.join(dataPath, leftSet))
        rightIms = np.load(os.path.join(dataPath, rightSet))

        left2D = []
        right2D = []
        indices2remove = []
        for i, (leftFrame, rightFrame) in enumerate(zip(leftIms, rightIms)):
            grayLeft = grayify(leftFrame)
            retLeft, posLeft = cv2.findCirclesGrid(grayLeft, boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                   blobDetector=circleDetector.get())

            grayRight = grayify(rightFrame)
            retRight, posRight = cv2.findCirclesGrid(grayRight, boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                     blobDetector=circleDetector.get())
            if retLeft and retRight:
                left2D.append(posLeft)
                right2D.append(posRight)
            else:
                indices2remove.append(i)
                left2D.append(None)
                right2D.append(None)

        leftIms = [leftIms[i] for i in range(len(leftIms)) if i not in indices2remove]
        rightIms = [rightIms[i] for i in range(len(rightIms)) if i not in indices2remove]
        left2D = [left2D[i] for i in range(len(left2D)) if i not in indices2remove]
        right2D = [right2D[i] for i in range(len(right2D)) if i not in indices2remove]
        newPositions3D = positions3D[: len(left2D)]
        cnt = 0
        while not SUCCESS:
            SUCCESS = True
            # STEREO CALIBRATION
            retError, leftMatrix, leftDist, rightMatrix, rightDist, R, T, E, F = cv2.stereoCalibrate(
                newPositions3D, left2D, right2D, leftMatrix, leftDist, rightMatrix, rightDist,
                None, flags=flags, criteria=criteria)
            indices2remove = []
            errors = []
            for i, (leftFrame, rightFrame) in enumerate(zip(leftIms, rightIms)):
                posLeft = left2D[i]
                posRight = right2D[i]
                error = epipolarConstraintMetric(F, posLeft, posRight,
                                                 leftMatrix, leftDist,
                                                 rightMatrix, rightDist,
                                                 False)
                errors.append(error)
                if error > threshold:
                    indices2remove.append(i)
                    SUCCESS = False
            print("After stereo calibration # {} we remove images: {}".format(cnt, indices2remove))
            leftIms = [leftIms[i] for i in range(len(leftIms)) if i not in indices2remove]
            rightIms = [rightIms[i] for i in range(len(rightIms)) if i not in indices2remove]
            left2D = [left2D[i] for i in range(len(left2D)) if i not in indices2remove]
            right2D = [right2D[i] for i in range(len(right2D)) if i not in indices2remove]
            newPositions3D = positions3D[: len(left2D)]
            cnt += 1
            if not SUCCESS:
                print("Redoing calibration with {} images".format(len(leftIms)))
        print("Final result: ")
        print(retError)
        print(T)
        rets.append(retError)
        Ts.append(T)
        epipolarConstraintErrors.append(errors)
        # break

    Ts = np.array(Ts)
    TsX = Ts[:, 0]
    TsY = Ts[:, 1]
    TsZ = Ts[:, 2]

    fig = plt.figure(num=0, figsize=(14, 6))
    ax1 = fig.add_subplot(111)
    ax1.set_ylabel("Translation of X (mm)")
    xs, ys = zip(*sorted(zip(rets, TsX)))
    lns1 = ax1.plot(xs, ys, label="X", color="green")
    ax1.axhline(y=-233, color='green', linestyle='--')

    ####
    ax2 = ax1.twinx()
    xs, ys = zip(*sorted(zip(rets, TsY)))
    lns2 = ax2.plot(xs, ys, label="Y", color="orange")
    ax2.axhline(y=np.mean(TsY), color='orange', linestyle='--')

    xs, ys = zip(*sorted(zip(rets, TsZ)))
    lns3 = ax2.plot(xs, ys, label="Z", color="blue")
    ax2.axhline(y=np.mean(TsZ), color='blue', linestyle='--')

    lns = lns1 + lns2 + lns3
    labs = [l.get_label() for l in lns]
    ax2.set_ylabel("Translation of Y and Z (mm)")
    ax1.set_xlabel("Retroprojection error (px)")
    ax2.set_title("Values of Tx, Ty and Tz as a function of the retroprojection error")
    ax2.grid()
    ax1.legend(lns, labs, loc=0)

    fig2 = plt.figure(num=1, figsize=(14, 6))
    ax = fig2.add_subplot(111)
    for setNum, error in zip(sets, epipolarConstraintErrors):
        ax.plot(error, label="Set {}".format(setNum))
    ax.set_ylabel("Error (px)")
    ax.set_xlabel("Image Number")
    ax.set_title("Epipolar constraint error for each image of each set (in px)")
    ax.grid()
    plt.show()
    return True


# draw the provided lines on the image
def drawLines(img, lines, colors, width):
    _, c, _ = img.shape
    # color = (0, 255, 0)
    for r, color in zip(lines, colors):
        color = (int(color[0]), int(color[1]), int(color[2]))
        r = r[0]
        x0, y0 = map(int, [0, -r[2] / r[1]])
        x1, y1 = map(int, [c, -(r[2] + r[0] * c) / r[1]])
        cv2.line(img, (x0, y0), (x1, y1), tuple(color), width)


def grayify(frame):
    if frame.shape == (768, 1024, 3):
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    elif frame.shape == (720, 1280, 3):
        return cv2.subtract(frame[:, :, 2], frame[:, :, 0])
    else:
        return False


def epipolarConstraintMetric(F, leftPts, rightPts, leftMatrix, leftDist, rightMatrix,
                             rightDist, VIZ):
    # Undistort checkerboard centers
    posLeftUndistorted = undistortPoints(leftPts, leftMatrix, leftDist)
    posRightUndistorted = undistortPoints(rightPts, rightMatrix, rightDist)

    # Compute the epipolar lines.
    # leftLines are the product of F.T * x for x in left Frame
    # rightLines are the product of F * x for x in right Frame
    leftLines = cv2.computeCorrespondEpilines(posLeftUndistorted, 1, F)
    rightLines = cv2.computeCorrespondEpilines(posRightUndistorted, 2, F)

    # Convert 2 homogenous
    posLeftUndistorted_homogeneous = np.ones_like(leftLines)
    posLeftUndistorted_homogeneous[:, :, :2] = posLeftUndistorted

    posRightUndistorted_homogeneous = np.ones_like(rightLines)
    posRightUndistorted_homogeneous[:, :, :2] = posRightUndistorted

    # Compute distances from each point to its associated line.
    # Line equation is of type (a, b, c) so distance of point (x, y) to line is simply:
    #  ax + by + c
    # According to epipolar geometry, this distance should be as close as possible to 0

    leftDists = np.mean(np.abs(np.sum(posLeftUndistorted_homogeneous * rightLines, axis=-1)))
    # leftDists = posLeftUndistorted_homogeneous * rightLines
    rightDists = np.mean(np.abs(np.sum(posRightUndistorted_homogeneous * leftLines, axis=-1)))
    error = np.mean([leftDists, rightDists])

    return error


def undistort(xy, k, distortion, iter_num=3):
    if type(distortion) is not list:
        distortion = list(distortion)
    if len(distortion) == 1:
        distortion = distortion[0]

    xy = xy[0]
    [k1, k2, p1, p2, k3] = distortion
    fx, fy = k[0, 0], k[1, 1]
    cx, cy = k[:2, 2]
    [x, y] = xy.astype(float)
    x = (x - cx) / fx
    x0 = x
    y = (y - cy) / fy
    y0 = y
    for _ in range(iter_num):
        r2 = x ** 2 + y ** 2
        k_inv = 1 / (1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3)
        delta_x = 2 * p1 * x * y + p2 * (r2 + 2 * x ** 2)
        delta_y = p1 * (r2 + 2 * y ** 2) + 2 * p2 * x * y
        x = (x0 - delta_x) * k_inv
        y = (y0 - delta_y) * k_inv
    # return np.array((x * fx + cx, y * fy + cy))
    return [[x * fx + cx, y * fy + cy]]


def undistortPoints(points, k, distortion):
    pointsUndistorted = []

    for point in points:
        pointsUndistorted.append(undistort(point, k, distortion))
    pointsUndistorted = np.array(pointsUndistorted).astype("float32")

    return pointsUndistorted


if __name__ == "__main__":
    main()
