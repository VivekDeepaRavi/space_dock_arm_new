import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco

# Define constants
KNOWN_WIDTH = 11.5  # The actual width of the object in cm
FOCAL_LENGTH = 1500  # Approximate focal length of your camera in pixels
POSITION_LIMIT = 11.5  # The maximum allowable position in cm

# Initialize ROS node
rospy.init_node('aruco_position_publisher')

# Initialize the CvBridge for image conversion
bridge = CvBridge()

# Define aruc o dictionary
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

# Initialize parameters for aruco detector
parameters = cv2.aruco.DetectorParameters_create()

tilt_obtained = False
once = False
TILT_ANGLE =0
angle_degrees = 180
incr =0

# Create a ROS publisher for the position data
position_publisher = rospy.Publisher('object_position', Float32, queue_size=1)

# Image callback function
def image_callback(data):
    global once,TILT_ANGLE,angle_degrees,incr
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
        return

    future_frame = cv_image

    # Calculate the rotation matrix for the specified tilt angle
    height, width = future_frame.shape[:2]

    # Calculate the top and bottom portion heights
    top_portion_height = int(height * 0.5)  # Adjust the percentage as needed
    bottom_portion_height = int(height * 0.9)  # Adjust the percentage as needed

    # Replace the top and bottom portions with white
    cv_image[:top_portion_height, :] = (255, 255, 255)  # White color
    cv_image[bottom_portion_height:, :] = (255, 255, 255)  # White color

    # Convert the frame to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

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
            aruco.drawDetectedMarkers(cv_image, [marker_0, marker_1])

            cv2.putText(cv_image, f'Orientation (degrees): {angle_degrees:.2f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            

            tilt_obtained = True
    #cv2.imshow("Frame", cv_image)

    if once == False:
         TILT_ANGLE = angle_degrees -180
         incr = incr+1
         print(TILT_ANGLE)
         if incr == 20:
             once = True
         else:
             pass
    #angle_degrees = 1000000
    #TILT_ANGLE = -0.8669569744922967
    #print (TILT_ANGLE)

    height, width = future_frame.shape[:2]
    rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), TILT_ANGLE, 1)

    # Rotate the frame to correct for camera tilt
    rotated_future_frame = cv2.warpAffine(future_frame, rotation_matrix, (width, height))

    # Calculate the top and bottom portion heights
    top_portion_height = int(height * 0.56)  # Adjust the percentage as needed
    bottom_portion_height = int(height * 0.8)  # Adjust the percentage as needed

    # Replace the top and bottom portions with white
    rotated_future_frame[:top_portion_height, :] = (255, 255, 255)  # White color
    rotated_future_frame[bottom_portion_height:, :] = (255, 255, 255)  # White color

    # Convert the rotated frame to grayscale
    gray = cv2.cvtColor(rotated_future_frame, cv2.COLOR_BGR2GRAY)

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
            position = (center_x - rotated_future_frame.shape[1] / 2) * distance / FOCAL_LENGTH

            # Draw rectangle around object
            aruco.drawDetectedMarkers(rotated_future_frame, [marker_0, marker_1])

            print(f'Position (cm): {position:.4f}')

            


            if abs(position) <= POSITION_LIMIT:
                position_msg = Float32()
                
                # if position >= 10.00:
                #     position = 10
                # elif position <= -10.00:
                #     position = -10 
                # else:
                #     pass
                position_msg.data = position
                position_publisher.publish(position_msg)
                # Display position and distance of object
                cv2.putText(rotated_future_frame, f'Position (cm): {position:.4f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(rotated_future_frame, f'Orientation (degrees): {angle_degrees:.4f}', (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                #cv2.putText(rotated_frame, f'Distance (cm): {distance:.2f}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                cv2.putText(rotated_future_frame, 'Object out of view', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Show the rotated frame
    cv2.imshow("final_Frame", rotated_future_frame)
    cv2.waitKey(1)


# Create a subscriber for the camera topic
image_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

# Start the ROS loop
rospy.spin()
