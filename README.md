
## Camera Service Node

![camera](https://github.com/DenWaritthon/ur_move_ws/blob/camera/pictures/camera.png)
![system](https://github.com/DenWaritthon/ur_move_ws/blob/camera/pictures/overview.png)

## work flow
1. Image Acquisition
   - Subscribe to the /camera/image_raw topic.
   - Convert incoming ROS image messages to OpenCV-compatible images using cv_bridge.
2. Color Space Conversion
   - Convert the acquired image from BGR to HSV color space for effective color filtering.
3. Color Filtering
- Create masks to isolate red regions:
   - Lower red hue range: [0, 120, 70] → [10, 255, 255]
- Upper red hue range (due to HSV color wrapping):
  - Lower boundary [170, 120, 70]
  - Upper boundary [180, 255, 255]
4. Contour Detection
- Apply OpenCV's findContours to locate contours from the masked red area.
  
5. Calculate the centroid (x, y) using image moments to determine the exact center of the detected red box:
   ``` bash 
    M = cv2.moments(contour)
  if M["m00"] != 0:
      cX = int(M["m10"] / M["m00"])
      cY = int(M["m01"] / M["m00"])
   ```
6. Coordinate Transformation