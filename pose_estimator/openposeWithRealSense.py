# To use Inference Engine backend, specify location of plugins:
# export LD_LIBRARY_PATH=/opt/intel/deeplearning_deploymenttoolkit/deployment_tools/external/mklml_lnx/lib:$LD_LIBRARY_PATH
import cv2 as cv
import numpy as np
import argparse
import time
from realsense import RealSenseCamera 
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]


class PoseEstimator:
    def __init__(self):
        self.network = cv.dnn.readNetFromTensorflow(f"{PROJECT_ROOT}/pose_estimator/graph_opt.pb")
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
        assert(len(self.BODY_PARTS) == out.shape[1])
    
    def get_2d_points(self, out, frameWidth, frameHeight):
        """Extracts 2D points from the output of the network."""
        self.points = []
        for i in range(len(self.BODY_PARTS)):
            heatMap = out[0, i, :, :]
            _, conf, _, point = cv.minMaxLoc(heatMap)
            x = (frameWidth * point[0]) / out.shape[3]
            y = (frameHeight * point[1]) / out.shape[2]
            self.points.append((int(x), int(y)) if conf > self.args.thr else None)
        return points
    
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
        else:
            wrist = self.points[self.BODY_PARTS["LWrist"]]
            elbow = self.points[self.BODY_PARTS["LElbow"]]
            shoulder = self.points[self.BODY_PARTS["LShoulder"]]
        if wrist and elbow and shoulder:
            elbow_angle = self.get_angle_2d(wrist, elbow, shoulder)
            return elbow_angle
        return None
    
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
            shoulder_angle = self.get_angle_2d(elbow, shoulder, neck)
            return shoulder_angle
        return None
    
    def draw_pose(self, text, position):
        """Draws pose text on the frame """
        for pair in self.POSE_PAIRS:
            partFrom = pair[0]
            partTo = pair[1]
            assert(partFrom in self.BODY_PARTS)
            assert(partTo in self.BODY_PARTS)

            idFrom = self.BODY_PARTS[partFrom]
            idTo = self.BODY_PARTS[partTo]

            if points[idFrom] and points[idTo]:
                cv.line(self.frame, points[idFrom], points[idTo], (0, 255, 0), 3)
                cv.ellipse(self.frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
                cv.ellipse(self.frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
        cv.putText(self.frame, text, position, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
        cv.imshow('OpenPose using OpenCV', self.frame)

# def get_angle_2d(pointA, pointB, pointC):
#     a = np.array(pointA)
#     b = np.array(pointB)
#     c = np.array(pointC)

#     ba = a - b
#     bc = c - b

#     cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
#     angle_rad = np.arccos(np.clip(cosine_angle, -1.0, 1.0))

#     return np.degrees(angle_rad)


    
# '''
# def get_angle_3d(pointA, pointB, pointC):
#     # Convert points to numpy arrays
#     a = np.array(pointA)
#     b = np.array(pointB)
#     c = np.array(pointC)
    
#     # Vectors
#     ba = a - b
#     bc = c - b
    
#     # Cosine angle calculation
#     cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
#     angle_rad = np.arccos(np.clip(cosine_angle, -1.0, 1.0))  # Clipping to avoid numerical errors
    
#     angle_deg = np.degrees(angle_rad)
    
#     return angle_deg
# '''



# BODY_PARTS = { "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
#                "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
#                "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
#                "LEye": 15, "REar": 16, "LEar": 17, "Background": 18 }

# POSE_PAIRS = [ ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
#                ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
#                ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
#                ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
#                ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"] ]

# inWidth = args.width
# inHeight = args.height

# net = cv.dnn.readNetFromTensorflow("graph_opt.pb")

# #UPDATES:
# # -> image comes from the RealSenseCamera
# # -> depth comes from the RealSenseCamera

# camera = RealSenseCamera()

# while cv.waitKey(1) < 0:
#     #Get frame directly from the RealSenseCamera
#     frame = camera.get_color_image()
#     if frame is None:
#         continue

#     frameWidth = frame.shape[1]
#     frameHeight = frame.shape[0]
    
#     net.setInput(cv.dnn.blobFromImage(frame, 1.0, (inWidth, inHeight), (127.5, 127.5, 127.5), swapRB=True, crop=False))
#     out = net.forward()
#     out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements

#     assert(len(BODY_PARTS) == out.shape[1])

#     points = []
#     for i in range(len(BODY_PARTS)):
#         # Slice heatmap of corresponging body's part.
#         heatMap = out[0, i, :, :]

#         # Originally, we try to find all the local maximums. To simplify a sample
#         # we just find a global one. However only a single pose at the same time
#         # could be detected this way.
#         _, conf, _, point = cv.minMaxLoc(heatMap)
#         x = (frameWidth * point[0]) / out.shape[3]
#         y = (frameHeight * point[1]) / out.shape[2]
#         # Add a point if it's confidence is higher than threshold.
#         points.append((int(x), int(y)) if conf > args.thr else None)

    
#     # Define the body parts of interest (Left/Right Wrist, Elbow, Shoulder)
#     body_parts_of_interest = ['RShoulder', 'RElbow', 'RWrist', 'LShoulder', 'LElbow', 'LWrist']

#     for part in body_parts_of_interest:
#         part_id = BODY_PARTS[part] 
#         point = points[part_id]
#         if point:  # If point detected
#             depth= camera.get_depth_at(point[0], point[1])

#             # Display the coordinates on the frame
#             cv.putText(frame, f"{part}: ({point[0]}, {point[1]}, {depth})", 
#                 (point[0] + 10, point[1] + 10),  # Position of the text
#                 cv.FONT_HERSHEY_SIMPLEX, 0.4,    # Font size (smaller for readability)
#                 (255, 255, 255), 1,              # White text color and thickness 2
#                 lineType=cv.LINE_AA)             # Anti-aliased for smoothness
    
#     # Helper function to get a 2D point if it exists
#     def get_2d_point(part):
#         part_id = BODY_PARTS[part]
#         point = points[part_id]
#         return point if point else None

#     # Right Elbow Angle Calculation (Wrist-Elbow-Shoulder)
#     r_wrist = get_2d_point("RWrist")
#     r_elbow = get_2d_point("RElbow")
#     r_shoulder = get_2d_point("RShoulder")

#     if r_wrist and r_elbow and r_shoulder:
#         elbow_angle = get_angle_2d(r_wrist, r_elbow, r_shoulder)
#         cv.putText(frame, f"Elbow: {elbow_angle:.1f} deg", 
#                (r_elbow[0]+15, r_elbow[1]-15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
#         print(f"Right Elbow Angle: {elbow_angle:.2f} degrees")

#     # Right Shoulder Angle Calculation (Elbow-Shoulder-Neck)
#     neck = get_2d_point("Neck")

#     if r_elbow and r_shoulder and neck:
#         shoulder_angle = get_angle_2d(r_elbow, r_shoulder, neck)
#         cv.putText(frame, f"Shoulder: {shoulder_angle:.1f} deg", 
#                (r_shoulder[0]+15, r_shoulder[1]-15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)
#         print(f"Right Shoulder Angle: {shoulder_angle:.2f} degrees")

#     # Assuming depth is already available from your existing script
    
#     # 3D Angle
# # def get_3d_point(part):
# #     part_id = BODY_PARTS[part]
# #     point = points[part_id]
# #     if point:
# #         depth = camera.get_depth_at(point[0], point[1])
# #         return (point[0], point[1], depth)
# #     return None

# # # Right Elbow Angle Calculation (Wrist-Elbow-Shoulder)
# # r_wrist = get_3d_point("RWrist")
# # r_elbow = get_3d_point("RElbow")
# # r_shoulder = get_3d_point("RShoulder")

# # if r_wrist and r_elbow and r_shoulder:
# #     elbow_angle = get_angle_3d(r_wrist, r_elbow, r_shoulder)
# #     print(f"Right Elbow Angle: {elbow_angle:.2f} degrees")

# # # Right Shoulder Angle Calculation (Elbow-Shoulder-Neck)
# # neck = get_3d_point("Neck")

# # if r_elbow and r_shoulder and neck:
# #     shoulder_angle = get_angle_3d(r_elbow, r_shoulder, neck)
# #     print(f"Right Shoulder Angle: {shoulder_angle:.2f} degrees")

#     # Optionally, print the coordinates in the terminal using the following code:
    
#     #for part in body_parts_of_interest:
#     #    part_id = BODY_PARTS[part]
#     #    point = points[part_id]
#     #    if point:
#     #        depth= camera.get_depth_at(point[0], point[1])
#     #        #time.sleep(0.05)
#     #        print(f"{part} coordinates: {point}, depth:{depth}")
    

#     for pair in POSE_PAIRS:
#         partFrom = pair[0]
#         partTo = pair[1]
#         assert(partFrom in BODY_PARTS)
#         assert(partTo in BODY_PARTS)

#         idFrom = BODY_PARTS[partFrom]
#         idTo = BODY_PARTS[partTo]

#         if points[idFrom] and points[idTo]:
#             cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
#             cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
#             cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)

#     t, _ = net.getPerfProfile()
#     freq = cv.getTickFrequency() / 1000
#     cv.putText(frame, '%.2fms' % (t / freq), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

#     cv.imshow('OpenPose using OpenCV', frame)

if __name__ == '__main__':
    pose_estimator = PoseEstimator()
    camera = cv.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Camera not opened.")
        exit()
    cv.waitKey(1)
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Frame not captured.")
            break

        pose_estimator.get_pose(frame)
        out = pose_estimator.network.forward()
        points = pose_estimator.get_2d_points(out, pose_estimator.frameWidth, pose_estimator.frameHeight)

        # Draw the pose on the frame
        pose_estimator.draw_pose("Pose Estimation", (10, 30))

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    camera.release()


    cv.destroyAllWindows()