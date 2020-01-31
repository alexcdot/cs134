/*
**   detectobject.cpp
**
**   Run a Cascade Object Detector in OpenCV.
**
**   Subscribers:    /image              Source image, to be remapped
**   Publishers:     /detector/image     Destination image
**                   /detector/???       Coordinates??
**
**   Services:       none
*/
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/package.h>


/*
**   Global Variables
**
**   We need to share the publisher and cascade detector with the callback.
*/
image_transport::Publisher  pub;	// Image publisher
cv::CascadeClassifier detector;		// Cascade detector


/*
**   Image Subscriber Callback
**
**   This message is of type std_msgs::Float64, i.e. it contains only
**   one number.  Use that as a new goal position, recomputing the
**   cubic spline.
*/
void imageCallback(const sensor_msgs::Image::ConstPtr& msgptr)
{
  cv::Mat cvImage;
  cv::Mat gray;
  std::vector<cv::Rect> objects;

  // Convert into OpenCV image.
  cvImage = cv_bridge::toCvShare(msgptr, "bgr8")->image;

  // Convert to gray scale.
  cv::cvtColor(cvImage, gray, cv::COLOR_BGR2GRAY);

  // Run the detector.
  detector.detectMultiScale(gray, objects, 1.3, 4, cv::CASCADE_SCALE_IMAGE,
			    cv::Size(30,30));

  // Report and indicate the objects in the image.
  for (int i = 0 ; i < objects.size() ; i++)
    {
      cv::Rect r = objects[i];
      ROS_INFO("Object %d: x %d y %d, width %d height %d",
	       i, r.x, r.y, r.width, r.height);
      cv::rectangle(cvImage, cv::Point(r.x, r.y), cv::Point(r.x+r.width, r.y+r.height),
		    cv::Scalar(255,0,0), 2);
    }

  // Convert back into a ROS image and republish (for debugging).
  pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvImage).toImageMsg());
}


/*
**   Main Code
*/
int main(int argc, char **argv)
{
  // Initialize the basic ROS node.
  ros::init(argc, argv, "detector");
  ros::NodeHandle n;

  // Locate the XML file in the hw4code package's detector folder.
  std::string XMLfile;
  XMLfile = ros::package::getPath("hw4code") + "/detector/cascade.xml";
  
  // Instantiate a cascade detector.
  if (!detector.load(XMLfile))
    {
      ROS_ERROR("Unable to load the cascade file '%s'!", XMLfile.c_str());
      return -1;
    }

  // Pick the topic names.  The source image topic can be
  // remapped in the command line.  The '~' places the output
  // image topic will be under the node name.
  std::string source_topic = ros::names::resolve("image");
  std::string output_topic = ros::names::resolve("~image");

  // Create an image transport instance.
  image_transport::ImageTransport it(n);

  // Subscribe to the source topic.  Using a queue size of one
  // means only the most recent message is stored for the next
  // subscriber callback.
  image_transport::Subscriber sub = it.subscribe(source_topic, 1, imageCallback);

  // Publish to the output tpoic.
  pub = it.advertise(output_topic, 1);

  // Report.
  ROS_INFO("Detector configured with:");
  ROS_INFO("Cascade XML file: %s", XMLfile.c_str());
  ROS_INFO("Image source topic: %s", source_topic.c_str());
  ROS_INFO("Image output topic: %s", output_topic.c_str());

  // Continually process until shutdown.
  ROS_INFO("Continually processing latest pending images...");
  ros::spin();

  // Report completion.
  ROS_INFO("Done!");
  return 0;
}
