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
    dataPath = r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\multiSetStereo\multiSetStereoImages2"
    positions3D = np.load(os.path.join(dataPath, "positions3D.npy"))
    leftMatrix, leftDist = openCalibrationFile(os.path.join(dataPath, "flirCalib.json"))
    rightMatrix, rightDist = openCalibrationFile(r"C:\Users\Recherche\OneDrive - polymtl.ca\PICUPE\Recalage\Results\kinect Calib\kinectCalib.json")
    flags = cv2.CALIB_FIX_INTRINSIC
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)
    rets = []
    Ts = []
    epipolarConstraintErrors = []
    sets = range(1, 7)

    for setNum in sets:
        if setNum == 2:
            continue
        # if setNum == 6:
        #     setNum = 1
        print("Set: {}".format(setNum))
        leftSet = "leftSet" + str(setNum) + ".npy"
        rightSet = "rightSet" + str(setNum) + ".npy"
        leftIms = np.load(os.path.join(dataPath, leftSet))
        rightIms = np.load(os.path.join(dataPath, rightSet))

        left2D = []
        right2D = []


        for i, (leftFrame, rightFrame) in enumerate(zip(leftIms, rightIms)):
            # cv2.putText(leftFrame, 'Image Number: ' + str(i + 1), (10, 50),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)
            # cv2.putText(rightFrame, 'Image Number: ' + str(i + 1), (10, 50),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1, (250, 150, 0), 2)

            grayLeft = grayify(leftFrame)
            retLeft, posLeft = cv2.findCirclesGrid(grayLeft, boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                   blobDetector=circleDetector.get())

            grayRight = grayify(rightFrame)
            retRight, posRight = cv2.findCirclesGrid(grayRight, boardSize, flags=cv2.CALIB_CB_ASYMMETRIC_GRID,
                                                     blobDetector=circleDetector.get())
            if retLeft and retRight:
                left2D.append(posLeft)
                right2D.append(posRight)

            UNDISTORT = False
            if UNDISTORT:
                leftFrameUndistorted = cv2.undistort(leftFrame, leftMatrix, leftDist)
                posLeftUndistorted = []
                for point in posLeft:
                    posLeftUndistorted.append(undistort(point, leftMatrix, leftDist))
                posLeftUndistorted = np.array(posLeftUndistorted).astype("float32")
                cv2.drawChessboardCorners(leftFrameUndistorted, boardSize, posLeftUndistorted, True)
                cv2.imshow("undistorted", leftFrameUndistorted)
                # cv2.imshow("Original", leftFrame)
                while True:
                    k = cv2.waitKey(100)
                    if k == ord("n"):
                        break
                    elif k == ord("q"):
                        return
                    else:
                        continue
            VIZ = False
            if VIZ:
                cv2.drawChessboardCorners(leftFrame, boardSize, posLeft, True)
                cv2.namedWindow("Left")  # Create a named window
                cv2.moveWindow("Left", 10, 10)
                cv2.namedWindow("Right")  # Create a named window
                cv2.moveWindow("Right", 500, 10)
                cv2.imshow("Left", cv2.resize(leftFrame, None, fx=0.5, fy=0.5))

                cv2.drawChessboardCorners(rightFrame, boardSize, posRight, True)
                cv2.imshow("Right", cv2.resize(rightFrame, None, fx=0.5, fy=0.5))
                while True:
                    k = cv2.waitKey(100)
                    if k == ord("n"):
                        break
                    elif k == ord("q"):
                        return
                    else:
                        continue

        newPositions3D = positions3D[: len(left2D)]

        ###
        # np.save(os.path.join(dataPath, "positions", "left" + str(setNum) + ".npy"), left2D)
        # np.save(os.path.join(dataPath, "positions", "right" + str(setNum) + ".npy"), right2D)
        # continue
        ###
        # STEREO CALIBRATION
        retError, leftMatrix, leftDist, rightMatrix, rightDist, R, T, E, F = cv2.stereoCalibrate(
            newPositions3D, left2D, right2D, leftMatrix, leftDist, rightMatrix, rightDist,
            None, flags=flags, criteria=criteria)
        rets.append(retError)
        Ts.append(T)

        # STEREO RECTIFICATION
        RLeft, RRight, PLeft, PRight, Q, roiLeft, roiRight = cv2.stereoRectify(
            leftMatrix, leftDist, rightMatrix, rightDist, (1024, 768),
            R, T, alpha=0.5, flags=cv2.CALIB_ZERO_DISPARITY)

        # COMPUTE UNDISTORTION+RECTIFICATION MAPS
        leftMap1, leftMap2 = cv2.initUndistortRectifyMap(leftMatrix, leftDist, RLeft, PLeft, (1024, 768), cv2.CV_32FC1)
        rightMap1, rightMap2 = cv2.initUndistortRectifyMap(rightMatrix, rightDist, RRight, PRight, (1024, 768),
                                                           cv2.CV_32FC1)
        indices2remove = []
        errors = []
        for i, (leftFrame, rightFrame) in enumerate(zip(leftIms, rightIms)):
            # print(i)
            posLeft = left2D[i]
            posRight = right2D[i]

            UNDISTORT = True
            if UNDISTORT:
                VIZ = True
                undistorted_l, undistorted_r, error = epipolarConstraintMetric(leftFrame, rightFrame,
                                                                               F, posLeft, posRight,
                                                                               leftMatrix, leftDist,
                                                                               rightMatrix, rightDist,
                                                                               VIZ)
                errors.append(error)
                if error > threshold:
                    indices2remove.append(i)

                if VIZ:
                    cv2.imshow("Left Undistorted", undistorted_l)
                    cv2.imshow("Right Undistorted", undistorted_r)
                    while True:
                        k = cv2.waitKey(100)
                        if k == ord("n"):
                            break
                        elif k == ord("q"):
                            return
                        else:
                            continue

            REMAP = False
            if REMAP:
                leftFrameRectified = cv2.remap(leftFrame, leftMap1, leftMap2, interpolation=cv2.INTER_LINEAR)
                rightFrameRectified = cv2.remap(rightFrame, rightMap1, rightMap2, interpolation=cv2.INTER_LINEAR)
                rectifiedPointsLeft = []
                for point in posLeft:
                    point = point[0]
                    y = int(point[1])
                    x = int(point[0])
                    rectifiedPointsLeft.append([leftMap1[y, x], leftMap2[y, x]])

                # Compute distance from points

                rectifiedPointsRight = []
                for point in posRight:
                    point = point[0]
                    y = int(point[1])
                    x = int(point[0])
                    rectifiedPointsRight.append([rightMap1[y, x], rightMap2[y, x]])
                rectifiedPointsRight = np.array(rectifiedPointsRight).astype("float32")

                cv2.drawChessboardCorners(leftFrameRectified, boardSize, rectifiedPointsLeft, True)
                cv2.drawChessboardCorners(rightFrameRectified, boardSize, rectifiedPointsRight, True)

                stereo = cv2.StereoSGBM_create(
                    numDisparities=16,
                    blockSize=16,
                )
                disparity = stereo.compute(leftFrameRectified, rightFrameRectified).astype(np.float32) / 16.0
                points3D = cv2.reprojectImageTo3D(disparity, Q)
                points3D = cv2.normalize(points3D, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX,
                                         dtype=cv2.CV_8UC3)

                cv2.imshow("Disparity Map", cv2.resize(disparity, dsize=None, fx=0.4, fy=0.4))
                cv2.imshow("Rectified Left", cv2.resize(leftFrameRectified, dsize=None, fx=0.4, fy=0.4))
                cv2.imshow("Rectified Right", cv2.resize(rightFrameRectified, dsize=None, fx=0.4, fy=0.4))
                while True:
                    k = cv2.waitKey(100)
                    if k == ord("n"):
                        break
                    elif k == ord("q"):
                        return
                    else:
                        continue

        print(retError)
        print(T)
        print()
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
    ax.set_yscale('log')
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


def epipolarConstraintMetric(leftFrame, rightFrame, F, leftPts, rightPts, leftMatrix, leftDist, rightMatrix,
                             rightDist, VIZ):
    error = 0
    leftFrameUndistorted = cv2.undistort(leftFrame, leftMatrix, leftDist)
    rightFrameUndistorted = cv2.undistort(rightFrame, rightMatrix, rightDist)

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

    if VIZ:
        width = 2
        colors = [np.random.randint(0, 255, 3) for _ in range(10)]
        # subset = np.random.default_rng().choice(135, 9, replace=False)
        drawLines(rightFrameUndistorted, leftLines[18: 27], colors, width)
        drawLines(leftFrameUndistorted, rightLines[18: 27], colors, width)

    return leftFrameUndistorted, rightFrameUndistorted, error


def undistort(xy, k, distortion, iter_num=3):
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
