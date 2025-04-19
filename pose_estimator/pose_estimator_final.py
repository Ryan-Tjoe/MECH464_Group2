# To use Inference Engine backend, specify location of plugins:
# export LD_LIBRARY_PATH=/opt/intel/deeplearning_deploymenttoolkit/deployment_tools/external/mklml_lnx/lib:$LD_LIBRARY_PATH
import cv2 as cv
import numpy as np
import argparse
import time
from realsense import RealSenseCamera 
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[0]
print(PROJECT_ROOT)

class PoseEstimator:
    def __init__(self):
        self.network = cv.dnn.readNetFromTensorflow(f"{PROJECT_ROOT}/graph_opt.pb")
        self.BODY_PARTS = { "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
                           "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9, 
                            "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
                            "LEye": 15, "REar": 16, "LEar": 17, "Background": 18 }
        self.POSE_PAIRS = [ ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
               ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
               ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
               ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
               ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"] ]
        parser = argparse.ArgumentParser()
        parser.add_argument('--input', help='Path to image or video. Skip to capture frames from camera')
        parser.add_argument('--thr', default=0.2, type=float, help='Threshold value for pose parts heat map')
        parser.add_argument('--width', default=368, type=int, help='Resize input to specific width.')
        parser.add_argument('--height', default=368, type=int, help='Resize input to specific height.')

        self.args = parser.parse_args()
        self.frame = None
        self.frameWidth = 0
        self.frameHeight = 0

    def get_pose(self, frame):
        """Takes a cv2 frame and returns the detected body parts and their coordinates."""
        inWidth = self.args.width
        inHeight = self.args.height
        self.frame = frame

        self.frameWidth = self.frame.shape[1]
        self.frameHeight = self.frame.shape[0]

        self.network.setInput(cv.dnn.blobFromImage(self.frame, 1.0, (inWidth, inHeight), (127.5, 127.5, 127.5), swapRB=True, crop=False))
        out = self.network.forward()
        out = out[:, :19, :, :]
        assert(len(self.BODY_print("HELLO")
PARTS) == out.shape[1])
    
    def get_2d_points(self, out, frameWidth, frameHeight):
        """Extracts 2D points from the output of the network."""
        self.points = []
        for i in range(len(self.BODY_PARTS)):
            heatMap = out[0, i, :, :]
            _, conf, _, point = cv.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            self.points.append((int(x), int(y)) if conf > self.args.thr else None)
        return self.points
    
    def get_angle_2d(self, pointA, pointB, pointC):
        """Calculates the angle between three 2D points."""
        a = np.array(pointA)
        b = np.array(pointB)
        c = np.array(pointC)

        ba = a - b
        bc = c - b

        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
        angle_rad = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

        return np.degrees(angle_rad)
    
    def get_elbow_angle(self, right_side =True):
        """Calculates the elbow angle using 2D points."""
        if right_side:
            wrist = self.points[self.BODY_PARTS["RWrist"]]
            elbow = self.points[self.BODY_PARTS["RElbow"]]
            shoulder = self.points[self.BODY_PARTS["RShoulder"]]
            neck = self.points[self.BODY_PARTS["Neck"]]
        else:
            wrist = self.points[self.BODY_PARTS["LWrist"]]
            elbow = self.points[self.BODY_PARTS["LElbow"]]
            shoulder = self.points[self.BODY_PARTS["LShoulder"]]
            neck = self.points[self.BODY_PARTS["Neck"]]
        if wrist and elbow and shoulder:
            # elbow_angle = self.get_angle_2d(wrist, elbow, shoulder)
            elbow_angle = self.angle_with_axis(wrist, elbow, neck, shoulder)
            return elbow_angle
        return None
    
    def angle_with_axis(self, p1, vertex, axis_start, axis_end):
        """Calculates the angle between the line (p1-vertex) and the axis (axis_start-axis_end)."""
        v1 = np.array(p1) - np.array(vertex)
        v2 = np.array(axis_end) - np.array(axis_start)
        cosine = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
        
    def get_shoulder_angle(self, right_side =True):
        """Calculates the shoulder angle using 2D points."""
        if right_side:
            elbow = self.points[self.BODY_PARTS["RElbow"]]
            shoulder = self.points[self.BODY_PARTS["RShoulder"]]
            neck = self.points[self.BODY_PARTS["Neck"]]
        else:
            elbow = self.points[self.BODY_PARTS["LElbow"]]
            shoulder = self.points[self.BODY_PARTS["LShoulder"]]
            neck = self.points[self.BODY_PARTS["Neck"]]
        if elbow and shoulder and neck:
            # shoulder_angle = self.get_angle_2d(elbow, shoulder, neck)
            shoulder_angle = self.angle_with_axis(elbow, shoulder, neck, shoulder)
            return shoulder_angle
        return None
    
    def draw_pose(self, text: str, position):
        """Draws pose text on the frame """
        for pair in self.POSE_PAIRS:
            partFrom = pair[0]
            partTo = pair[1]
            assert(partFrom in self.BODY_PARTS)
            assert(partTo in self.BODY_PARTS)

            idFrom = self.BODY_PARTS[partFrom]
            idTo = self.BODY_PARTS[partTo]

            if self.points[idFrom] and self.points[idTo]:
                cv.line(self.frame, self.points[idFrom], self.points[idTo], (0, 255, 0), 3)
                cv.ellipse(self.frame, self.points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
                cv.ellipse(self.frame, self.points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
        t, _ = self.network.getPerfProfile()
        freq = cv.getTickFrequency() / 1000
        latency = '%.2fms' % (t / freq)
        text += " " + latency
        cv.putText(self.frame, text, position, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        cv.imshow('OpenPose using OpenCV', self.frame)
        cv.waitKey(1)

if __name__ == '__main__':
    pose_estimator = PoseEstimator()
    camera = RealSenseCamera()


    while True:
        frame = camera.get_color_image()


        pose_estimator.get_pose(frame)
        out = pose_estimator.network.forward()
        points = pose_estimator.get_2d_points(out, pose_estimator.frameWidth, pose_estimator.frameHeight)

        # Draw the pose on the frame
        pose_estimator.draw_pose("Pose Estimation", (10, 30))
