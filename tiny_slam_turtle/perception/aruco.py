import cv2
import numpy as np

def detect_aruco_pose(rgb, camera_matrix=None, dist_coeffs=None, marker_length=0.1):
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco
    dict_ = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, dict_, parameters=params)
    detections = []
    if ids is not None and camera_matrix is not None and dist_coeffs is not None:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
        for i, mid in enumerate(ids.flatten()):
            detections.append((int(mid), rvecs[i], tvecs[i]))
    return detections
