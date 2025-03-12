# Camera Service Node

![camera](https://github.com/DenWaritthon/ur_move_ws/blob/camera/pictures/camera.png)  
![system](https://github.com/DenWaritthon/ur_move_ws/blob/camera/pictures/overview.png)

---

## Work Flow

### 1. Image Acquisition
- **Subscribe** to the `/camera/image_raw` topic.
- **Convert** incoming ROS image messages to OpenCV-compatible images using `cv_bridge`.

### 2. Color Space Conversion
- **Convert** the acquired image from BGR to HSV color space for effective color filtering.

### 3. Color Filtering
- **Create masks** to isolate red regions:
  - **Lower red hue range:** `[0, 120, 70]` → `[10, 255, 255]`
  - **Upper red hue range** (to account for HSV color wrapping):
    - **Lower boundary:** `[170, 120, 70]`
    - **Upper boundary:** `[180, 255, 255]`

### 4. Contour Detection
- **Apply OpenCV's `findContours`** to locate contours from the masked red area:
  
  ```python
  contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

  if contours:
      # Select the largest contour assuming it's the target box
      largest_contour = max(contours, key=cv2.contourArea)


### 5. Calculate the Centroid

After detecting the largest red contour, compute its centroid (center position) using image moments. This provides the pixel coordinates (cX, cY) of the red box:

```python
M = cv2.moments(largest_contour)
if M["m00"] != 0:
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])

```

### 6. Coordinate Transformation
- This transformation involves applying offset adjustments calibrated
```python 
# Find the red box position from mask (this function returns pixel coordinates)
Y, X = self.find_box_position(mask_red)

# Adjust coordinates based on calibration:
X = (X * -1) + 0.55  # Flip X-axis and apply offset calibration
Y = (Y * -1)         # Flip Y-axis

if X is not None and Y is not None:
    response.target.position.x = X
    response.target.position.y = Y
    response.target.position.z = 0.0  # Flat workspace, z = 0
    response.target.orientation.w = 1.0  # No rotation applied

    self.get_logger().info(f"Detected Red Box at X={X:.2f}, Y={Y:.2f}")

- Flipping Axes: Multiplying by -1 aligns the camera's coordinate system with the robot's workspace.
- Offset Calibration: Adding 0.55 to the X-coordinate shifts the origin to match the UR5 base frame.