import cv2
import cv2.aruco as aruco
import numpy as np

# Define constants
KNOWN_WIDTH = 11.5  # The actual width of the object in cm
FOCAL_LENGTH = 1500  # Approximate focal length of your camera in pixels
POSITION_LIMIT = 10.5  # The maximum allowable position in cm

# Initialize the webcam
cap = cv2.VideoCapture(1)

# Define aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Initialize parameters for aruco detector
parameters = aruco.DetectorParameters_create()

# Define the tilt angle in degrees (positive for upward tilt, negative for downward tilt)
tilt_angle = 20  # Example: 10 degrees upward tilt

while True:
    # Capture each frame
    _, frame = cap.read()

    # Get the height and width of the frame
    height, width = frame.shape[:2]

    # Define the percentage of the upper part to cut (e.g., 30%)
    cut_percentage = 0.5

    # Calculate the number of rows to cut
    cut_rows = int(height * cut_percentage)

    # Create a white image of the same size as the frame
    white_part = np.full((cut_rows, width, 3), (255, 255, 255), dtype=np.uint8)

    # Replace the upper part of the frame with white
    frame[:cut_rows, :] = white_part

    # Adjust the camera intrinsic matrix for tiltation
    tilt_radians = np.radians(tilt_angle)
    new_focal_length_y = FOCAL_LENGTH / np.cos(tilt_radians)
    new_camera_matrix = np.array([[FOCAL_LENGTH, 0, width / 2],
                                  [0, new_focal_length_y, height / 2],
                                  [0, 0, 1]])

    # Undistort the frame using the new camera matrix (if you have distortion coefficients)
    # frame = cv2.undistort(frame, camera_matrix, distortion_coefficients, new_camera_matrix)

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect aruco markers in the frame
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    cv2.imshow("Frame", frame)

    # Exit loop on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all windows
cap.release()
cv2.destroyAllWindows()