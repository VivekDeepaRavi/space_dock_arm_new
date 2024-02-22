import cv2
import cv2.aruco as aruco
import numpy as np

# Define constants
KNOWN_WIDTH = 11.5  # The actual width of the object in cm
FOCAL_LENGTH = 1500  # Approximate focal length of your camera in pixels
POSITION_LIMIT = 10.5  # The maximum allowable position in cm
TILT_ANGLE = -3.8  # The tilt angle of the camera in degrees

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Define aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Initialize parameters for aruco detector
parameters = aruco.DetectorParameters_create()

while True:
    # Capture each frame
    _, frame = cap.read()

    # Calculate the rotation matrix for the specified tilt angle
    height, width = frame.shape[:2]
    rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), TILT_ANGLE, 1)

    # Rotate the frame to correct for camera tilt
    rotated_frame = cv2.warpAffine(frame, rotation_matrix, (width, height))

    # Calculate the top and bottom portion heights
    top_portion_height = int(height * 0.6)  # Adjust the percentage as needed
    bottom_portion_height = int(height * 0.8)  # Adjust the percentage as needed

    # Replace the top and bottom portions with white
    rotated_frame[:top_portion_height, :] = (255, 255, 255)  # White color
    rotated_frame[bottom_portion_height:, :] = (255, 255, 255)  # White color

    # Convert the rotated frame to grayscale
    gray = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2GRAY)

    # Detect aruco markers in the rotated frame
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if np.all(ids is not None):
        if 0 in ids and 1 in ids:
            marker_0 = corners[np.where(ids==0)[0][0]]
            marker_1 = corners[np.where(ids==1)[0][0]]

            # Compute the geometric mean point of each marker
            marker_0_center = np.mean(marker_0[0], axis=0)
            marker_1_center = np.mean(marker_1[0], axis=0)

            # Compute the pixel width between the markers' centers
            marker_width = np.linalg.norm(marker_0_center - marker_1_center)

            # Calculate distance of object from camera
            distance = (KNOWN_WIDTH * FOCAL_LENGTH) / marker_width

            # Calculate position of object relative to center of camera's view
            center_x = (marker_0_center[0] + marker_1_center[0]) / 2
            position = (center_x - rotated_frame.shape[1] / 2) * distance / FOCAL_LENGTH

            # Draw rectangle around object
            aruco.drawDetectedMarkers(rotated_frame, [marker_0, marker_1])

            if abs(position) <= POSITION_LIMIT:
                # Display position and distance of object
                cv2.putText(rotated_frame, f'Position (cm): {position:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                #cv2.putText(rotated_frame, f'Distance (cm): {distance:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(rotated_frame, 'Object out of view', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Show the rotated frame
    cv2.imshow("Frame", rotated_frame)

    # Exit loop on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all windows
cap.release()
cv2.destroyAllWindows()
