The sequence of commands needed to calibrate the camera:

```
roscore

rosrun usb_cam usb_cam_node __name:=cam_feed _pixel_format:=yuyv _camera_name:=logitech_cam _image_width:=640 _image_height:=480

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0231 image:=/cam_feed/image_raw camera:=/cam_feed
```

Then, to run the detector node:

```
ROS_NAMESPACE=cam_feed rosrun image_proc image_proc

rosrun boogaloo pydetectobject.py
```

The sequence of commands needed to start the robot:

```
roslaunch boogaloo runsolver.launch
```