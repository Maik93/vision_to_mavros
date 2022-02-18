import cv2
import numpy as np
import glob

K = np.array([[289.22431161906024, 0.0, 415.35740667946783],
              [0.0, 288.9799586082012, 404.7018605999265],
              [0.0, 0.0, 1.0]])
D = np.array([-0.02124170439107891, 0.07311021781479196, -0.06446836883745513, 0.013266403411735362])

K1, _ = cv2.getOptimalNewCameraMatrix(K, D, (800, 800), 1.)
map_x, map_y = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K1, (800, 800), cv2.CV_32FC1)

images = glob.glob('images/*.png')
for image_name in images:
    img = cv2.imread(image_name, cv2.IMREAD_GRAYSCALE)

    rect_img = cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    cv2.imshow("rectified", np.hstack((img, rect_img)))
    key = cv2.waitKey()
    if key == ord('q'):
        break

cv2.destroyWindow("rectified")
