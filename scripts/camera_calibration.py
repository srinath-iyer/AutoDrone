import cv2
import numpy as np
import glob

# === Settings ===
chessboard_size = (7, 10)       # Inner corners per row/column
square_size = 31.75             # mm
image_folder = "scripts/camera_calibration/*.jpg"

# === Prepare object points ===
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points in world
imgpoints = []  # 2D points in image

images = glob.glob(image_folder)

print(f"Found {len(images)} images...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Optional: refine and draw
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        img = cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Corners', img)
        cv2.waitKey(100)
    else:
        print(f"❌ Corners not found in {fname}")

cv2.destroyAllWindows()

# === Calibrate ===
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("\n✅ Calibration Complete")
print("Camera matrix (K):\n", K)
print("Distortion coefficients:\n", dist.ravel())

# === Save to file ===
np.savez("camera_calibration.npz", K=K, dist=dist)

print("\nSaved to 'camera_calibration.npz'")
