import cv2
import numpy as np
import glob

# Prepare object points (3D points in real world space)
# For a 10x8 grid, we have 9 internal corners along the x-axis and 7 internal corners along the y-axis.
objp = np.zeros((9*7, 3), np.float32)  # 9 corners in width, 7 corners in height
objp[:, :2] = np.mgrid[0:9, 0:7].T.reshape(-1, 2)  # Assign 2D grid coordinates to the 3D points

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load images
images = glob.glob('calibration_images/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, (9, 7), None)

    if ret:
        objpoints.append(objp)  # Add object points
        imgpoints.append(corners)  # Add image points

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (9, 7), corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Calibrate the camera
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the camera matrix and distortion coefficients
print("Camera matrix:", mtx)
print("Distortion coefficients:", dist)
