# To use Inference Engine backend, specify location of plugins:
# export LD_LIBRARY_PATH=/opt/intel/deeplearning_deploymenttoolkit/deployment_tools/external/mklml_lnx/lib:$LD_LIBRARY_PATH
import cv2 as cv
import numpy as np
import argparse
import time
from pathlib import Path

PROJECT_ROOT = str(Path(__file__).resolve().parents[1])
print(PROJECT_ROOT)


parser = argparse.ArgumentParser()
parser.add_argument('--input', help='Path to image or video. Skip to capture frames from camera')
parser.add_argument('--thr', default=0.2, type=float, help='Threshold value for pose parts heat map')
parser.add_argument('--width', default=368, type=int, help='Resize input to specific width.')
parser.add_argument('--height', default=368, type=int, help='Resize input to specific height.')

args = parser.parse_args()

BODY_PARTS = { "Nose": 0, "Neck": 1, "RShoulder": 2, "RElbow": 3, "RWrist": 4,
               "LShoulder": 5, "LElbow": 6, "LWrist": 7, "RHip": 8, "RKnee": 9,
               "RAnkle": 10, "LHip": 11, "LKnee": 12, "LAnkle": 13, "REye": 14,
               "LEye": 15, "REar": 16, "LEar": 17, "Background": 18 }

POSE_PAIRS = [ ["Neck", "RShoulder"], ["Neck", "LShoulder"], ["RShoulder", "RElbow"],
               ["RElbow", "RWrist"], ["LShoulder", "LElbow"], ["LElbow", "LWrist"],
               ["Neck", "RHip"], ["RHip", "RKnee"], ["RKnee", "RAnkle"], ["Neck", "LHip"],
               ["LHip", "LKnee"], ["LKnee", "LAnkle"], ["Neck", "Nose"], ["Nose", "REye"],
               ["REye", "REar"], ["Nose", "LEye"], ["LEye", "LEar"] ]

inWidth = args.width
inHeight = args.height

net = cv.dnn.readNetFromTensorflow(f"{PROJECT_ROOT}/pose_estimator/graph_opt.pb")

cap = cv.VideoCapture(args.input if args.input else 0)

def angle_with_axis(p1, vertex, axis_start, axis_end):
    """Calculates the angle between the line (p1-vertex) and the axis (axis_start-axis_end)."""
    v1 = np.array(p1) - np.array(vertex)
    v2 = np.array(axis_end) - np.array(axis_start)
    cosine = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    return np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))

while cv.waitKey(1) < 0:
    hasFrame, frame = cap.read()
    if not hasFrame:
        cv.waitKey()
        break

    frameWidth = frame.shape[1]
    frameHeight = frame.shape[0]
    
    net.setInput(cv.dnn.blobFromImage(frame, 1.0, (inWidth, inHeight), (127.5, 127.5, 127.5), swapRB=True, crop=False))
    out = net.forward()
    out = out[:, :19, :, :]  # MobileNet output [1, 57, -1, -1], we only need the first 19 elements

    assert(len(BODY_PARTS) == out.shape[1])

    points = []
    for i in range(len(BODY_PARTS)):
        # Slice heatmap of corresponging body's part.
        heatMap = out[0, i, :, :]

        # Originally, we try to find all the local maximums. To simplify a sample
        # we just find a global one. However only a single pose at the same time
        # could be detected this way.
        _, conf, _, point = cv.minMaxLoc(heatMap)
        x = (frameWidth * point[0]) / out.shape[3]
        y = (frameHeight * point[1]) / out.shape[2]
        # Add a point if it's confidence is higher than threshold.
        points.append((int(x), int(y)) if conf > args.thr else None)

    
    # Define the body parts of interest (Left/Right Wrist, Elbow, Shoulder)
    body_parts_of_interest = ['RShoulder', 'RElbow', 'RWrist', 'LShoulder', 'LElbow', 'LWrist']

    for part in body_parts_of_interest:
        part_id = BODY_PARTS[part] 
        point = points[part_id]
        if point:  # If point detected
            # Display the coordinates on the frame
            cv.putText(frame, f"{part}: ({point[0]}, {point[1]})", 
                (point[0] + 10, point[1] + 10),  # Position of the text
                cv.FONT_HERSHEY_SIMPLEX, 0.4,    # Font size (smaller for readability)
                (255, 255, 255), 1,              # White text color and thickness 2
                lineType=cv.LINE_AA)             # Anti-aliased for smoothness

    # Get required points
    neck = points[BODY_PARTS["Neck"]]
    r_shoulder = points[BODY_PARTS["RShoulder"]]
    r_elbow = points[BODY_PARTS["RElbow"]]
    r_wrist = points[BODY_PARTS["RWrist"]]

    if neck and r_shoulder and r_elbow and r_wrist:
        # Axis: Neck → Right Shoulder
        axis_start = neck
        axis_end = r_shoulder

        # Elbow angle w.r.t. axis
        elbow_angle = angle_with_axis(r_wrist, r_elbow, axis_start, axis_end)
        cv.putText(frame, f"Elbow: {elbow_angle:.1f}", (r_elbow[0]+10, r_elbow[1]-10),
                cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Shoulder angle w.r.t. axis
        shoulder_angle = angle_with_axis(r_elbow, r_shoulder, axis_start, axis_end)
        cv.putText(frame, f"Shoulder: {shoulder_angle:.1f}", (r_shoulder[0]+10, r_shoulder[1]-10),
                cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        print(f"Elbow Angle wrt Neck-RShoulder axis: {elbow_angle:.2f}°")
        print(f"Shoulder Angle wrt Neck-RShoulder axis: {shoulder_angle:.2f}°")

    # Optionally, print the coordinates in the terminal using the following code:
    
    #for part in body_parts_of_interest:
    #    part_id = BODY_PARTS[part]
    #    point = points[part_id]
    #    if point:
    #        #time.sleep(0.05)
    #        print(f"{part} coordinates: {point}")

    for pair in POSE_PAIRS:
        partFrom = pair[0]
        partTo = pair[1]
        assert(partFrom in BODY_PARTS)
        assert(partTo in BODY_PARTS)

        idFrom = BODY_PARTS[partFrom]
        idTo = BODY_PARTS[partTo]

        if points[idFrom] and points[idTo]:
            cv.line(frame, points[idFrom], points[idTo], (0, 255, 0), 3)
            cv.ellipse(frame, points[idFrom], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)
            cv.ellipse(frame, points[idTo], (3, 3), 0, 0, 360, (0, 0, 255), cv.FILLED)

    t, _ = net.getPerfProfile()
    freq = cv.getTickFrequency() / 1000
    cv.putText(frame, '%.2fms' % (t / freq), (10, 20), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

    cv.imshow('OpenPose using OpenCV', frame)