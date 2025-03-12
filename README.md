
## Camera Service Node

- The node subscribes to the camera topic (/camera/image_raw) and continuously receives image data.
- It uses OpenCV's contour detection technique to draw contours around red objects in the frame.
- From the detected red contour, the node calculates the position of a red box.

## Service Communication
- A service is created (e.g., /get_target) that returns the pose (position and orientation) of the detected red box.

## callback function processes images to detect the position of a red box from camera input
- Workflow :
1. Receive camera image and convert it into OpenCV format.
2. Convert image to HSV color space for easier color segmentation.
3. Create a mask to filter out everything except the red regions.
4. Use contour detection to locate the exact boundary of the red box.
5. Find the centroid (center position) of the detected red contour.
6. Transform pixel coordinates into real-world coordinates, applying necessary offsets and axis adjustments to match the UR5 robot’s coordinate frame.



