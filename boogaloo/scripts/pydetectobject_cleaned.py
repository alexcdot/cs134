#!/usr/bin/env python
#
#   pydetectobject_cleaned.py
#
#   Run a Vision-based Object Detector in OpenCV to detect parts of the bottle
#   to be flipped for our bottle flipping robot
#

# ROS Imports
import rospy
import sensor_msgs.msg
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from boogaloo.msg import Detection, Activation
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo
from camera_calibration.calibrator import ChessboardInfo
from camera_calibration.calibrator import Calibrator
from cv2 import aruco
from joblib import load
from sklearn.tree import DecisionTreeClassifier

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

CAP_HEIGHT = 0.20
BAND_HEIGHT = 0.0254 * 3
RIM_HEIGHT = 0.0254 * 2.25


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


    def calibrate_checkerboard(self, image, display_image):
        # Test for the presense of a checkerboard and pull out the
        # corners as a list of (u,v) data.
        # Grayscale the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        (ok, corners, board) = self.calibrator.get_corners(gray)
        if not ok:
            print("No matching checkerboard...")
            return [], [], display_image
        corners = corners.reshape(-1,2)

        # Set the (X,Y,Z) data for each corner.  This presumes the
        # checkerboard is on the Z=0 plane.
        ## 02/28 measurements
        ## x = 82.5, y = -28.9
        ## X_CENTER = 0.834
        ## Y_CENTER = -0.331

        # 03/04 measurements
        X_CENTER = 0.8285
        Y_CENTER = -0.3165
        """
        Robot thinks
        checkerboard 0.838 x, -0.311 y
        aruco large square, width 6 inchs / 15.2 cm
        18 aruco top right corner 0.791 x, -0.080 y
        12 aruco top right 0.294 x, 0.194
        42 topleft corner of far 0.363  x, -0.310
        42 topright corner of far 0.363  - 0.152 x, -0.310
        """

        """
        Hand measured
        checkerboard, 0.7735 + 0.055 = 0.8285, 0.3165
        18, 0.727 + 0.055 = 0.7914, -0.085
        12, 0.2395 + 0.055 = 0.2945, 0.192
        42, 0.148 + 0.055 = 0.203,-0.318
        """

        xyz = np.zeros((len(corners), 3))
        for r in range(board.n_rows):
            for c in range(board.n_cols):
                i = r*board.n_cols + c
                xyz[i][0] = board.dim * (c - (board.n_cols-1)/2.0) + X_CENTER
                xyz[i][1] = board.dim * ((board.n_rows-1)/2.0 - r) + Y_CENTER
                xyz[i][2] = 0

        display_image = cv2.drawChessboardCorners(
            image,
            (self.board.n_cols, self.board.n_rows),
            corners,
            patternWasFound=True
        )

        # Really these are lists of (u,v) and (x,y,z)
        return xyz, corners, display_image


    def calibrate_aruco_cv(self, image, display_image):
        # aruco variables
        ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_50)
        MARKER_LENGTH = 0.152
        TOP_RIGHT_CORNER_IND = 1
        """
        Hand measured
        18, 0.727 + 0.055 = 0.7814, -0.085
        12, 0.2395 + 0.055 = 0.2945, 0.192
        42, 0.148 + 0.055 = 0.203,-0.318
        """
        #  Location of top right corner
        XYZ_BY_ID = {
            18: (0.791, -0.085, 0),
            12: (0.2945, 0.192, 0),
            42: (0.203, -0.318, 0)
        }

        # Grayscale the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find aruco markers in the query image, top left corner
        corners, ids, _ = aruco.detectMarkers(
                image=gray,
                dictionary=ARUCO_DICT)
        
        verified_corners = []
        verified_xyz = []
        if ids is None:
            print("No aruco markers found")
            return [], [], display_image
        for i, aruco_id in enumerate(ids.flatten()):
            if aruco_id in XYZ_BY_ID:
                # corners matrix is [aruco marker][0][corner][x,y]]
                verified_corners.append(corners[i][0][TOP_RIGHT_CORNER_IND].flatten())
                verified_xyz.append(XYZ_BY_ID[aruco_id])

        # convert to numpy
        verified_corners = np.array(verified_corners)
        verified_xyz = np.array(verified_xyz)

        # Outline the aruco markers found in our query image
        display_image = aruco.drawDetectedMarkers(
                image=display_image,
                corners=corners,
                ids=ids)

        return verified_xyz, verified_corners, display_image


    def calibrate_charucoboard(self, image, display_image):

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
        display_image = aruco.drawDetectedMarkers(
                image=display_image,
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
            display_image = aruco.drawDetectedCornersCharuco(
                    image=display_image,
                    charucoCorners=charuco_corners,
                    charucoIds=charuco_ids)

            return [], corners_all, display_image

        else:
            print("Not able to detect a charuco board in image")
            return [], [], display_image


    def calibrate_all(self, ros_image, set_params=True):
        # Convert into OpenCV image.
        image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        display_image = image.copy()
        xyz_list = []
        corners_list = []

        calibration_funcs = [
            self.calibrate_checkerboard,
            self.calibrate_aruco_cv
        ]
        for calibration_func in calibration_funcs:
            xyz, corners, display_image = calibration_func(image, display_image)
            xyz_list.extend(xyz)
            corners_list.extend(corners)

        # Convert to np
        xyz_list = np.array(xyz_list)
        corners_list = np.array(corners_list)
        
        if set_params:
            print(xyz_list)
            print(corners_list)
            self.locate_camera(xyz_list, corners_list)

        return display_image


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
        print("actual", self.x_cam_wrt_world)
        #self.x_cam_wrt_world = np.array([[2], [-0.125], [1.18]])

        """
        Hand measured
        checkerboard, 0.7735 + 0.055 = 0.8285, 0.3165
        18, 0.727 + 0.055 = 0.7814, -0.085
        12, 0.2395 + 0.055 = 0.2945, 0.192
        42, 0.148 + 0.055 = 0.203,-0.318
        """

        # Report.
        print("r cam wrt world")
        print(self.R_cam_wrt_world)
        print("cam loc should be:", [1.335, 0.125, 1.18])
        print("Cam loc (x, y, z relative to 0,0,0): %6.3f, %6.3f, %6.3f" 
              % tuple(self.x_cam_wrt_world.reshape(3)))

        self.check_calibration(corners)

    #
    #   Undistort
    #
    #   Compute the normalized (image) coordinates from the pixels
    #
    def undistort(self, uv, obj_height=0.0):
        # Map to the normalized (image) coordinates.  As above, the API
        # assume a set of lists of points, so reshape accordingly.
        xybar = cv2.undistortPoints(uv.reshape(1,-1,2).astype(float), self.K, self.D).reshape(2)

        # Now map into the world.  Here I am assuming zw = 0...
        Rc = self.R_cam_wrt_world
        xc = self.x_cam_wrt_world.reshape(3)

        lam = -xc[2] / (Rc[2][0]*xybar[0] + Rc[2][1]*xybar[1] + Rc[2][2])
        xw = lam*(Rc[0][0]*xybar[0] + Rc[0][1]*xybar[1] + Rc[0][2]) + xc[0]
        yw = lam*(Rc[1][0]*xybar[0] + Rc[1][1]*xybar[1] + Rc[1][2]) + xc[1]

        # Adjust for the height of the object
        cam_height = self.x_cam_wrt_world[2]
        factor = obj_height / cam_height
        obj_to_cam_xy = (
            self.x_cam_wrt_world[0] - xw,
            self.x_cam_wrt_world[1] - yw
        )
        xw += obj_to_cam_xy[0] * factor
        yw += obj_to_cam_xy[1] * factor

        # hacky x axis correction for bloating
        xw += 0.025 / max(1, (xw - 0.10) * 100) + 0.01
        return (xw, yw)

    def undistort_cap(self, uv):
        return self.undistort(uv, CAP_HEIGHT)

    def undistort_band(self, uv):
        return self.undistort(uv, BAND_HEIGHT)

    def undistort_rim(self, uv):
        return self.undistort(uv, RIM_HEIGHT)

    def check_calibration(self, corners):
        # Pick a (u,v) pair.  I used the top-left corner for
        # testing, which is (-3.5, 2.5) * 0.0254 * 29 / 32
        uv = corners[0]
        xw, yw = self.undistort(uv)
        X_CENTER = 0.8285
        Y_CENTER = -0.3165
        # Check the location in number of squares...
        n_x = (xw - X_CENTER) / self.board.dim
        n_y = (yw - Y_CENTER) / self.board.dim
        print('location of top left corner in number of squares', [n_x, n_y],
              'we expect -1.5 and 1')


#
#  Detector Node Class
#

class Detector:

    def __init__(self):
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
        debug_topic = rospy.resolve_name("~debug_image")
        activation_topic = rospy.resolve_name("/activation")
        cap_output_topic = rospy.resolve_name("/bottle_cap_dets")
        band_output_topic = rospy.resolve_name("/bottle_band_det")
        rim_output_topic = rospy.resolve_name("/bottle_rim_det")
        target_output_topic = rospy.resolve_name("/target_det")

        this_file_dir = os.path.dirname(os.path.abspath(__file__))
        ASSETS_PATH = os.path.join(this_file_dir, "../assets/")

        # Decision tree color segmentation classifiers
        self.green_classifier = load(ASSETS_PATH + '134green.joblib')
        self.pink_classifier = load(ASSETS_PATH + '134pink.joblib')
        self.orange_classifier = load(ASSETS_PATH + '134orange.joblib')

        first_image = rospy.wait_for_message(source_topic, Image)
        self.checkCalibrator.calibrate_all(first_image)

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
        
        # Publish to the activation topic.
        self.activation_publisher = rospy.Publisher(activation_topic, Activation, queue_size=1)

        # Publish to the cap output topic.
        self.cap_publisher = rospy.Publisher(cap_output_topic,
                                                Detection,
                                                queue_size=1)

        # Publish to the band output topic:
        self.band_publisher = rospy.Publisher(band_output_topic,Detection,queue_size=1)

        # Publish to the rim output topic:
        self.rim_publisher = rospy.Publisher(rim_output_topic,Detection,queue_size=1)

        # Publish to the target output topic:
        self.target_publisher = rospy.Publisher(target_output_topic,Detection,queue_size=1)

        # Publish to the calibration topic.
        self.calibration_publisher = rospy.Publisher(calibration_topic,
                                         sensor_msgs.msg.Image,
                                         queue_size=1)

        # Publish to the debug topic.
        self.debug_publisher = rospy.Publisher(debug_topic,
                                         sensor_msgs.msg.Image,
                                         queue_size=1)

        # Report.
        rospy.loginfo("Detector configured with:")
        rospy.loginfo("Image source topic: " + source_topic)
        rospy.loginfo("Image output topic: " + output_topic)
        rospy.loginfo("Tplink Rect output topic: " + cap_output_topic)

    def treeThreshold(self, image, classifier):
        shp = image.shape
        pixels = image.reshape(-1, 3)
        out = classifier.predict(pixels)
        out = out.reshape(shp[0], shp[1], 1)
        return out

    def test_calibration(self, ros_image):
        #calibration_image = self.checkCalibrator.calibrate_charucoboard(ros_image)
        calibration_image = self.checkCalibrator.calibrate_all(ros_image, set_params=False)
        self.calibration_publisher.publish(
            self.bridge.cv2_to_imgmsg(calibration_image, "bgr8"))

    def detect(self, image, classifier, min_size, max_size, show_mask=False):
        # use detection tree threshold for hue
        mask = self.treeThreshold(image, classifier)[:,:,0]
        # use adaptive thresholding for saturation
        blur_v = cv2.GaussianBlur(image[:, :, 1],(3, 3),0)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if show_mask:
            ms = mask.shape
            mask[:, ms[1]/2:(ms[1]/2+2)] = 255
            mask[ms[0]/2:(ms[0]/2+2),:] = 255

            v_image = image[:,:,2]
            # Otsu's thresholding
            ret2,th2 = cv2.threshold(v_image,255,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

            # Otsu's thresholding after Gaussian filtering
            blur = cv2.GaussianBlur(v_image,(3, 3),0)
            ret3,th3 = cv2.threshold(blur,255,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

            print(image[ms[0]/2 - 2:ms[0]/2+2, ms[1]/2 - 2:ms[1]/2+2])
            print("contour areas:", sorted([cv2.contourArea(c) for c in contours], reverse=True))
            
            self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(mask, '8UC1'))
        try:
            # try all blobs since we have a size range
            for blob in contours:
                if cv2.contourArea(blob) > min_size and cv2.contourArea(blob) < max_size:
                    M = cv2.moments(blob)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                    return mask, center, M
            return mask, None, None

        except ValueError:
            return mask, None, None

    def process(self, ros_image):
        self.test_calibration(ros_image)
        # Convert into OpenCV image.
        cv_img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

        # Convert to rgb scale.
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        # convert from 360, 100, 100 to 180, 255, 255)

        '''This is how we used to do detection with adaptive thresholding, but
        we switched to using decision trees
        
        cap_tols = np.array([5, 30, 15]) * 2
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
        cap_contours, _ = cv2.findContours(cap_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        band_contours, _ = cv2.findContours(band_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
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
        
        if found and M is not None:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            cv2.circle(cv_img, center, 2, (0,0,255), -1)
            uv = np.array((center))
            xw, yw = self.checkCalibrator.undistort(uv)
            if detection_type == "cap":
                zw = CAP_HEIGHT
            elif detection_type == "band":
                zw = BAND_HEIGHT
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
        
        cap_mask, cap_center, _ = self.detect(
            hsv_img, self.green_classifier, 90, 600,
            show_mask=True)

        # hot pink
        band_mask, band_center, band_moments = self.detect(
            hsv_img, self.pink_classifier, 200, 1500,
            show_mask=False)

        rim_mask, rim_center, rim_moments = self.detect(
            hsv_img, self.pink_classifier, 10, 100,
            show_mask=False)

        target_mask, target_center, target_moments = self.detect(
            hsv_img, self.orange_classifier, 200, 100000,
            show_mask=False)

        """
        Other colors HSV ranges:
            hot pink: 165, 177 (160-180), 142 ( 120-150)
                cap_picked_color = (165, 140, 142)
                cap_tols = (10, 255, 40)
            hot orange: 16, 177 ( 170- 195), 163 (145 - 170)
                cap_picked_color = (16, 160, 160)
                cap_tols = (5, 255, 25)
            deep light green 82, 89 ( 75-95), 130 ( 110 - 130)
            light dark green: 82, 155 ( 130-160), 103 ( 90 - 110)
        """

        # Publish object detections

        if cap_center is not None:
            cv2.circle(cv_img, cap_center, 2, (0, 0, 255), -1)
            uv = np.array((cap_center))
            xw, yw = self.checkCalibrator.undistort_cap(uv)
            # actually 0.2, but we lower it for the sake of the robot
            zw=CAP_HEIGHT - 0.04
            detection_msg = Detection()
            detection_msg.position = Vector3()
            detection_msg.position.x = xw
            detection_msg.position.y = yw
            detection_msg.position.z = zw
            print('cap x, y, z:', xw[0], yw[0], zw)
            self.cap_publisher.publish(detection_msg)
        if band_center is not None:
            cv2.circle(cv_img, band_center, 2, (0,0,255), -1)
            uv = np.array((band_center))
            xw, yw = self.checkCalibrator.undistort_band(uv)
            zw=BAND_HEIGHT
            detection_msg = Detection()
            detection_msg.position = Vector3()
            detection_msg.position.x = xw
            detection_msg.position.y = yw
            detection_msg.position.z = zw
            print('band x, y, z:', xw[0], yw[0], zw)
            self.band_publisher.publish(detection_msg)
        if rim_center is not None:
            cv2.circle(cv_img, rim_center, 2, (0,0,255), -1)
            uv = np.array((rim_center))
            xw, yw = self.checkCalibrator.undistort_rim(uv)
            zw=RIM_HEIGHT
            detection_msg = Detection()
            detection_msg.position = Vector3()
            detection_msg.position.x = xw
            detection_msg.position.y = yw
            detection_msg.position.z = zw
            print('rim x, y, z:', xw[0], yw[0], zw)
            self.rim_publisher.publish(detection_msg)
        if target_center is not None:
            cv2.circle(cv_img, target_center, 2, (0,0,255), -1)
            uv = np.array((target_center))
            xw, yw = self.checkCalibrator.undistort_rim(uv)
            zw=0
            detection_msg = Detection()
            detection_msg.position = Vector3()
            detection_msg.position.x = xw
            detection_msg.position.y = yw
            detection_msg.position.z = zw
            print('target x, y, z:', xw[0], yw[0], zw)
            self.target_publisher.publish(detection_msg)

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
