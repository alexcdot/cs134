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


#
#  Detector Node Class
#
class Detector:
    def __init__(self):
        # Locate the XML file in the hw4code package's detector folder.
        import rospkg
        import os
        import errno
        XMLfile = rospkg.RosPack().get_path('hw4code') + '/detector/cascade.xml'
        if not os.path.isfile(XMLfile):
            raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), XMLfile)

        # Instantiate a cascade detector.
        self.detector = cv2.CascadeClassifier(XMLfile)

        # Set up the OpenCV Bridge.
        self.bridge = cv_bridge.CvBridge()

        # Pick the topic names.  The source image topic can be
        # remapped in the command line.  The '~' places the output
        # image topic will be under the node name.
        source_topic = rospy.resolve_name("image")
        output_topic = rospy.resolve_name("~image")

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

        # Report.
        rospy.loginfo("Detector configured with:")
        rospy.loginfo("Cascade XML file: " + XMLfile)
        rospy.loginfo("Image source topic: " + source_topic)
        rospy.loginfo("Image output topic: " + output_topic)

    def process(self, rosImage):
        # Convert into OpenCV image.
        cvImage = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")

        # Convert to gray scale.
        gray = cv2.cvtColor(cvImage, cv2.COLOR_BGR2GRAY)

        # Run the detector.
        objects = self.detector.detectMultiScale(gray,
                                                 scaleFactor=1.3,
                                                 minNeighbors=4,
                                                 minSize=(30,30),
                                                 flags=cv2.CASCADE_SCALE_IMAGE)

        # For the fun of it.  This should also be published!
        print objects

        # Indicate the objects in the image.
        for (x,y,w,h) in objects:
            cv2.rectangle(cvImage,(x,y),(x+w,y+h),(255,0,0),2)

        # Convert back into a ROS image and republish (for debugging).
        self.publisher.publish(
            self.bridge.cv2_to_imgmsg(cvImage, "bgr8"))


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
