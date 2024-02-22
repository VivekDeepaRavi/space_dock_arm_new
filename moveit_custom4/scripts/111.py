import cv2
import numpy as np

# Initialize the webcam
cap = cv2.VideoCapture(0)

# Define the rotation angle in degrees (e.g., 10 degrees clockwise)
angle = -4

while True:
    # Capture each frame
    _, frame = cap.read()

    # Get the height and width of the frame
    height, width = frame.shape[:2]

    # Calculate the rotation matrix
    rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), angle, 1)

    # Rotate the frame by the specified angle
    rotated_frame = cv2.warpAffine(frame, rotation_matrix, (width, height))

    # Convert the rotated frame to grayscale
    gray = cv2.cvtColor(rotated_frame, cv2.COLOR_BGR2GRAY)

    # Show the rotated frame
    cv2.imshow("Frame", rotated_frame)

    # Exit loop on 'q' press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and destroy all windows
cap.release()
cv2.destroyAllWindows()
