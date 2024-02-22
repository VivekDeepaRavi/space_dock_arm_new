import cv2
import cv2.aruco as aruco
import numpy as np
import math

# Define constants
KNOWN_WIDTH = 11.5  # The actual width of the object in cm
FOCAL_LENGTH = 1500  # Approximate focal length of your camera in pixels
POSITION_LIMIT = 10.5  # The maximum allowable position in cm

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Define aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Initialize parameters for aruco detector
parameters = aruco.DetectorParameters_create()

while True:
    # Capture each frame
    _, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect aruco markers in the frame
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if np.all(ids is not None):
        if 0 in ids and 1 in ids:
            marker_0 = corners[np.where(ids==0)[0][0]]
            marker_1 = corners[np.where(ids==1)[0][0]]

            # Compute the geometric mean point of each marker
            marker_0_center = np.mean(marker_0[0], axis=0)
            marker_1_center = np.mean(marker_1[0], axis=0)


            # Calculate orientation angle with respect to markers (in degrees)
            angle_degrees = math.degrees(math.atan2(marker_1_center[1] - marker_0_center[1],
                                                    marker_1_center[0] - marker_0_center[0]))

            # Draw rectangle around object
            aruco.drawDetectedMarkers(frame, [marker_0, marker_1])
            cv2.putText(frame, f'Orientation (degrees): {angle_degrees:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show the frame
    cv2.imshow("Frame", frame)

    # Exit loop on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all windows
cap.release()
cv2.destroyAllWindows()
