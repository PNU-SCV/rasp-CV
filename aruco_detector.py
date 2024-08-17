import cv2
import numpy as np

class ArucoDetector:
    def __init__(self, camera_matrix, dist_coeffs):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()

    def detect_marker_angle(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None and ids[0][0] == 0: 
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
            rmat, _ = cv2.Rodrigues(rvec[0])
            marker_y_axis = rmat[:, 1]
            angle = np.degrees(np.arctan2(marker_y_axis[0], marker_y_axis[2]))
            return (angle + 360) % 360
        
        return None
