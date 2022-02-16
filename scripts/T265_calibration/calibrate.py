import cv2
import numpy as np
import glob

CHESSBOARD = (5, 7)

_img_shape = None
obj_points = []  # 3d point in real world space
img_points = []  # 2d points in image plane.

sub_pix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
objp = np.zeros((1, CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
objp[0, :, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)

images = glob.glob('images/*.png')
for image_name in images:
    img = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)
    if _img_shape is None:
        _img_shape = img.shape[:2]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."

    # find the chessboard corners
    ret, corners = \
        cv2.findChessboardCorners(img, CHESSBOARD,
                                  cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    # add object points and image points (after refining them)
    if ret:
        obj_points.append(objp)
        cv2.cornerSubPix(img, corners, (3, 3), (-1, -1), sub_pix_criteria)
        img_points.append(corners)
    else:
        print(f"WW: {image_name} does not seem to have a chessboard inside!")

N_OK = len(obj_points)
if N_OK == 0:
    print("EE: no chessboard found on any camera!")
    exit(1)

K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
rms, _, _, _, _ = \
    cv2.fisheye.calibrate(obj_points, img_points, _img_shape[::-1],
                          K, D, rvecs, tvecs, calibration_flags, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))

print("Found " + str(N_OK) + " valid images for calibration.")
print("Image dimensions = " + str(_img_shape[::-1]))
print("K = " + str(K.flatten().tolist()))
print("D = " + str(D.flatten().tolist()))
