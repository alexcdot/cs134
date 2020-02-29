#!/usr/bin/env python
#
#   pydetectobject.py
#
#   Run a Cascade Object Detector in OpenCV.
#
#   Subscribers:    /image              Source image, to be remapped
#   Publishers:     /detector/image     Destination image
#                   /detector/???       Coordinates??
#
#   Services:       none
#

# ROS Imports
import rospy
import sensor_msgs.msg
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from boogaloo.msg import Detection
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo
from camera_calibration.calibrator import ChessboardInfo
from camera_calibration.calibrator import Calibrator
from cv2 import aruco


import rospkg
import os
import errno

def intrinsic_params_from_file():
    # Grab a camera_info message, change topic as needed.
    msg = rospy.wait_for_message('/cam_feed/camera_info', CameraInfo)

    # Check/grab the camera matrix.
    if (msg.K[0] == 0 or msg.K[1] != 0 or msg.K[2] == 0 or
        msg.K[3] != 0 or msg.K[4] == 0 or msg.K[5] == 0 or
        msg.K[6] != 0 or msg.K[7] != 0 or msg.K[8] != 1):
        rospy.logerr("Camera Intrinsic Parameters strangely formatted!")
        rospy.signal_shutdown("Camera incorrectly calibrated")
        return
    K = np.float64(msg.K).reshape(3,3)

    # Check/grab the distortion model.
    D = np.float64(msg.D)
    return K, D


class CheckerboardCalibrator:
    def __init__(self):
        # Define the Checkerboard.  Note the OpenCV detector
        # apparently assumes more columns than rows.
        self.board = ChessboardInfo()
        self.board.n_cols = 4
        self.board.n_rows = 3
        self.board.dim = 0.0254 * (1 + 7 / 8.0)
        self.K, self.D = intrinsic_params_from_file()
        self.bridge = cv_bridge.CvBridge()

        # Instantiate a Calibrator, to extract corners etc.
        self.calibrator = Calibrator([self.board])
        self.R_cam_wrt_world = None
        self.x_cam_wrt_world = None
        self.R_world_wrt_cam = None
        self.x_world_wrt_cam = None

    def calibrate_checkboard(self, image):
        # Test for the presense of a checkerboard and pull out the
        # corners as a list of (u,v) data.
        gray = self.calibrator.mkgray(image)
        (ok, corners, board) = self.calibrator.get_corners(gray)
        if not ok:
            print("No matching checkboard...")
            return
        corners = corners.reshape(-1,2)

        # Set the (X,Y,Z) data for each corner.  This presumes the
        # checkerboard is on the Z=0 plane.
        # x = 82.5, y = -28.9
        X_CENTER = 0.834
        Y_CENTER = -0.331
        xyz = np.zeros((len(corners), 3))
        for r in range(board.n_rows):
            for c in range(board.n_cols):
                i = r*board.n_cols + c
                xyz[i][0] = board.dim * (c - (board.n_cols-1)/2.0) + X_CENTER
                xyz[i][1] = board.dim * ((board.n_rows-1)/2.0 - r) + Y_CENTER
                xyz[i][2] = 0

        # Really these are lists of (u,v) and (x,y,z)
        self.locate_camera(xyz, corners)

    def calibrate_charucoboard(self, ros_image):
        # Convert into OpenCV image.
        image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # ChAruco board variables
        CHARUCOBOARD_ROWCOUNT = 5
        CHARUCOBOARD_COLCOUNT = 5
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_5X5_50)
        SQUARE_LENGTH = (1+5.0/8) * 0.0254
        MARKER_LENGTH = (1+9.0/32) * 0.0254
        # Create constants to be passed into OpenCV and Aruco methods
        print(
            CHARUCOBOARD_COLCOUNT > 1,
            CHARUCOBOARD_ROWCOUNT > 1,
            SQUARE_LENGTH > MARKER_LENGTH
        )
        CHARUCO_BOARD = aruco.CharucoBoard_create(
                squaresX=CHARUCOBOARD_COLCOUNT,
                squaresY=CHARUCOBOARD_ROWCOUNT,
                squareLength=SQUARE_LENGTH,
                markerLength=MARKER_LENGTH,
                dictionary=ARUCO_DICT)

        # Create the arrays and variables we'll use to store info like corners and IDs from images processed
        corners_all = [] # Corners discovered in all images processed
        ids_all = [] # Aruco ids corresponding to corners discovered
        image_size = None # Determined at runtime

        # Grayscale the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find aruco markers in the query image
        corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=ARUCO_DICT)

        # Outline the aruco markers found in our query image
        image = aruco.drawDetectedMarkers(
                image=image,
                corners=corners)

        # Get charuco corners and ids from detected aruco markers
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                markerCorners=corners,
                markerIds=ids,
                image=gray,
                board=CHARUCO_BOARD)

        # If a Charuco board was found, let's collect image/corner points
        # Requiring at least 20 squares
        if response > 5:
            # Add these corners and ids to our calibration arrays
            corners_all.append(charuco_corners)
            ids_all.append(charuco_ids)

            # Draw the Charuco board we've detected to show our calibrator the board was properly detected
            image = aruco.drawDetectedCornersCharuco(
                    image=image,
                    charucoCorners=charuco_corners,
                    charucoIds=charuco_ids)

            # If our image size is unknown, set it now
            if not image_size:
                image_size = gray.shape[::-1]

            # Reproportion the image, maxing width or height at 1000
            # proportion = max(img.shape) / 1000.0
            # img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))
            # # Pause to display each image, waiting for key press
            #cv2.imshow('Charuco board', img)
            #cv2.waitKey(0)
            return image

        else:
            print("Not able to detect a charuco board in image")
            return None


    #
    #   Determine the camera position/orientation
    #
    #   Note in the vision world, folks are interested in where objects
    #   are relative to the camera.  So we will need to invert the
    #   orientation/position to get the camera w.r.t. world (which is our
    #   object).
    #
    def locate_camera(self, xyz, corners):        
        # Compute the world frame w.r.t. camera.
        ok, rvec, tvec = cv2.solvePnP(xyz, corners, self.K, self.D)
        if not ok:
            print("Problem locating the camera!")
            return
        (self.R_world_wrt_cam, _) = cv2.Rodrigues(rvec)
        self.x_world_wrt_cam      = tvec

        # Convert into the camera frame w.r.t. world.
        self.R_cam_wrt_world = self.R_world_wrt_cam.transpose()
        self.x_cam_wrt_world = - np.matmul(self.R_cam_wrt_world, self.x_world_wrt_cam)

        # Report.
        print("r cam wrt world", self.R_cam_wrt_world)
        print("Cam loc (relative to center of board): %6.3f, %6.3f, %6.3f" 
              % tuple(self.x_cam_wrt_world.reshape(3)))

        self.check_calibration(corners)

    #
    #   Undistort
    #
    #   Compute the normalized (image) coordinates from the pixels
    #
    def undistort(self, uv):
        # Map to the normalized (image) coordinates.  As above, the API
        # assume a set of lists of points, so reshape accordingly.
        #print(uv.reshape(1,-1,2), uv.reshape(1,-1,2).shape)
        #raise Exception
        xybar = cv2.undistortPoints(uv.reshape(1,-1,2).astype(float), self.K, self.D).reshape(2)
        #print('xbar, ybar:', xybar)

        # Now map into the world.  Here I am assuming zw = 0...
        Rc = self.R_cam_wrt_world
        xc = self.x_cam_wrt_world.reshape(3)

        lam = -xc[2] / (Rc[2][0]*xybar[0] + Rc[2][1]*xybar[1] + Rc[2][2])
        xw = lam*(Rc[0][0]*xybar[0] + Rc[0][1]*xybar[1] + Rc[0][2]) + xc[0]
        yw = lam*(Rc[1][0]*xybar[0] + Rc[1][1]*xybar[1] + Rc[1][2]) + xc[1]

        BOTTLE_HEIGHT = 0.2
        cam_height = self.x_cam_wrt_world[2]
        factor = BOTTLE_HEIGHT / cam_height
        bottle_to_cam_xy = (
            self.x_cam_wrt_world[0] - xw,
            self.x_cam_wrt_world[1] - yw
        )
        xw += bottle_to_cam_xy[0] * factor
        yw += bottle_to_cam_xy[1] * factor

        return (xw, yw)


    def check_calibration(self, corners):
        # Pick a (u,v) pair.  I used the top-left corner for
        # testing, which is (-3.5, 2.5) * 0.0254 * 29 / 32
        uv = corners[0]
        xw, yw = self.undistort(uv)
        # Check the location in number of squares...
        n_x = xw / self.board.dim
        n_y = yw / self.board.dim
        print('location of top left corner in number of squares', [n_x, n_y])


#
#  Detector Node Class
#
class Detector:


    def __init__(self):
        self.cap_height = 0.16
        self.band_height = 0.075
        # Instantiate a cascade detector.
        self.detector = None#cv2.CascadeClassifier(XMLfile)

        # Instantiate a calibrator
        self.checkCalibrator = CheckerboardCalibrator()

        # Set up the OpenCV Bridge.
        self.bridge = cv_bridge.CvBridge()

        # Pick the topic names.  The source image topic can be
        # remapped in the command line.  The '~' places the output
        # image topic will be under the node name.
        source_topic = rospy.resolve_name("/cam_feed/image_rect_color")
        output_topic = rospy.resolve_name("~image")
        calibration_topic = rospy.resolve_name("~calibration_image")
        cap_output_topic = rospy.resolve_name("/bottle_cap_dets")
        band_output_topic = rospy.resolve_name("/bottle_band_det")

        first_image = rospy.wait_for_message(source_topic, Image)
        self.checkCalibrator.calibrate_checkboard(first_image)

        # Subscribe to the source topic.  Using a queue size of one
        # means only the most recent message is stored for the next
        # subscriber callback.
        rospy.Subscriber(source_topic,
                         sensor_msgs.msg.Image,
                         self.process,
                         queue_size=1)

        # Publish to the output topic.
        self.publisher = rospy.Publisher(output_topic,
                                         sensor_msgs.msg.Image,
                                         queue_size=1)
        # Publish to the cap output topic.
        self.cap_publisher = rospy.Publisher(cap_output_topic,
                                                Detection,
                                                queue_size=1)

        # Publish to the band output topic:
        self.band_publisher = rospy.Publisher(band_output_topic,Detection,queue_size=1)

        # Publish to the calibration topic.
        self.calibration_publisher = rospy.Publisher(calibration_topic,
                                         sensor_msgs.msg.Image,
                                         queue_size=1)

        # Report.
        rospy.loginfo("Detector configured with:")
        rospy.loginfo("Image source topic: " + source_topic)
        rospy.loginfo("Image output topic: " + output_topic)
        rospy.loginfo("Tplink Rect output topic: " + cap_output_topic)


    def test_calibration(self, ros_image):
        calibration_image = self.checkCalibrator.calibrate_charucoboard(ros_image)
        print(type(calibration_image))
        self.calibration_publisher.publish(
            self.bridge.cv2_to_imgmsg(calibration_image, "bgr8"))


    def process(self, ros_image):
        #self.test_calibration(ros_image)
        # Convert into OpenCV image.
        cv_img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # Convert to rgb scale.
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # convert from 360, 100, 100 to 180, 255, 255)
        '''tols = np.array([2, 30, 15]) * 2

        cap_picked_color = (200.0 / 2, 75 * 2.55, 55 * 2.55)
        cap_lower = (picked_color[0] - tols[0], picked_color[1] - tols[1], picked_color[2] - tols[2])
        cap_upper = (picked_color[0] + tols[0], picked_color[1] + tols[1], picked_color[2] + tols[2])
        cap_mask = cv2.inRange(hsv_img,lower,upper)
        ms = cap_mask.shape'''
        # mask[:, ms[1]/2] = 255
        # mask[ms[0]/2,:] = 255
        # # print("starting")
        # print(hsv_img[ms[0]/2 - 5:ms[0]/2+5, ms[1]/2 - 5:ms[1]/2+5])
        
        #exit(1)
        #print(np.max(mask), np.min(mask))
        #mask = mask[:ms[0]/2, :ms[1]/2]

        cap_tols = np.array([5, 30, 15]) * 2
        #cap_picked_color = (200.0 / 2, 75 * 2.55, 55 * 2.55) # old values
        cap_picked_color = (185.0 / 2, 75 * 2.55, 55 * 2.55)
        cap_lower = (cap_picked_color[0] - cap_tols[0], cap_picked_color[1] - cap_tols[1], cap_picked_color[2] - cap_tols[2])
        cap_upper = (cap_picked_color[0] + cap_tols[0], cap_picked_color[1] + cap_tols[1], cap_picked_color[2] + cap_tols[2])
        cap_mask = cv2.inRange(hsv_img,cap_lower,cap_upper)
        cap_ms = cap_mask.shape

        band_picked_color = (210 / 2, 85 * 2.55, 50 * 2.55)
        band_tols = np.array([5,20,20]) * 2
        band_lower = (band_picked_color[0] - band_tols[0], band_picked_color[1] - band_tols[1], band_picked_color[2] - band_tols[2])
        band_upper = (band_picked_color[0] + band_tols[0], band_picked_color[1] + band_tols[1], band_picked_color[2] + band_tols[2])
        band_mask = cv2.inRange(hsv_img,band_lower,band_upper)
        band_ms = band_mask.shape

        #self.calibration_publisher.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))
        cap_contours, _ = cv2.findContours(cap_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        band_contours, _ = cv2.findContours(band_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #print("contours:", contours)
        found = False
        try:
            band_blob = max(band_contours, key=lambda el: cv2.contourArea(el))
            print("largest band blob: " + str(cv2.contourArea(band_blob)))
            if cv2.contourArea(band_blob) > 100: # magic number for the size of the band
                M = cv2.moments(band_blob)
                detection_type = "band"
                found = True
        except ValueError:
            pass

        try:
            cap_blob = max(cap_contours, key=lambda el: cv2.contourArea(el))
            print("largest cap blob: " + str(cv2.contourArea(cap_blob)))
            if cv2.contourArea(cap_blob) > 50: # magic number for the size of the cap
                M = cv2.moments(cap_blob)
                detection_type = "cap"
                found = True
        except ValueError:
            pass

        #self.calibration_publisher.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))
        #cap_contours, _ = cv2.findContours(cap_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #band_contours, _ = cv2.findContours(band_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        #print("contours:", contours)
        #if len(cap_contours) == 0:
        #    print("no cap contours found")
        #    return
        #blob = max(cap_contours, key=lambda el: cv2.contourArea(el))
        #M = cv2.moments(blob)
        #center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if found and M is not None:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            cv2.circle(cv_img, center, 2, (0,0,255), -1)
            uv = np.array((center))
            xw, yw = self.checkCalibrator.undistort(uv)
            if detection_type == "cap":
                zw = self.cap_height
            elif detection_type == "band":
                zw = self.band_height
            detection_msg = Detection()
            detection_msg.position = Vector3()
            detection_msg.position.x = xw
            detection_msg.position.y = yw
            detection_msg.position.z = zw
            print('x, y, z:', xw[0], yw[0], zw)
            if detection_type == "cap":
                self.cap_publisher.publish(detection_msg)
            elif detection_type == "band":
                self.band_publisher.publish(detection_msg)

        '''
        # Run the detector.
        objects = []#self.detector...

        # For the fun of it.  This should also be published!
        if len(objects) > 0:
            print(objects)

        # Indicate the objects in the image.
        for (x,y,w,h) in objects:
            cv2.rectangle(cv_img,(x,y),(x+w,y+h),(255,0,0),2)
            detection_msg = Rect()
            detection_msg.x = x + w/2
            detection_msg.y = y + h/2
            detection_msg.width = w
            detection_msg.height = h
            self.cap_publisher.publish(detection_msg)
        '''

        # Convert back into a ROS image and republish (for debugging).
        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.  You can override the name using the
    # 'rosrun .... __name:=something' convention.
    rospy.init_node('detector')

    # Instantiate the Detector object.
    detector = Detector()

    # Continually process until shutdown.
    rospy.loginfo("Continually processing latest pending images...")
    rospy.spin()

    # Report completion.
    rospy.loginfo("Done!")
