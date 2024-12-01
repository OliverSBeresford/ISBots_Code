# Camera Setup and Calibration Guide

1. **Position the Camera**

   Set up the camera in the desired position on your robot.

2. **Calibrate the Camera**

   Follow the FTC Camera Calibration Guide for detailed instructions.

   Run Frame Capture Utility
   Open and run the UtilityCameraFrameCapture.java OpMode to capture calibration images.

3. **Install MRCAL**

   Ensure MRCAL is installed using the following command:

   ```bash
   pip install mrcal
   ```

4. **Perform Camera Calibration with MRCAL**

   Run the calibration command using the captured images:

   ```bash
   mrcal-calibrate-cameras --lensmodel LENSMODEL_OPENCV8 \
   --image-glob 'VisionPortal-CameraFrameCapture-000000.png' \
   --corners-type apriltag --corners-width 8 --corners-height 8 \
   --corners-spacing 0.1016
   ```

5. **Apply Lens Intrinsics in Code**

   Use the calculated parameters (fx, fy, cx, cy):

   ```java
   builder.setLensIntrinsics(fx, fy, cx, cy);
   ```

6. **Set Camera Position and Orientation**

   Measure and define the camera's position relative to the robot's center:

   ```java
   cameraPosition = new Position(DistanceUnit.CM, x, y, z, 0);
   cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);
   ```

7. **Configure AprilTag Processor Settings**

   Set Decimation Level
   Optimize detection performance with:

   ```java
   myAprilTagProcessor.setDecimation(1);
   ```

8. **Implement Detection Logic**

   Check for the Desired Tag ID
   Add logic to identify the correct tag:

   ```java
   if (desired_tag_id < 0 || desired_tag_id == detection.id) {
       desiredTag = detection;
       break;
   }
   ```
   
   Calculate Errors for Position Control
   Determine range and heading errors:

   ```java
   rangeError = desiredTag.ftcPose.range - desired_distance;
   headingError = desiredTag.ftcPose.bearing;
   ```

9. **Drive to the Desired Location**

   Use the errors to control robot movement:

   ```java
   forward = rangeError * speed_gain;
   rotate = headingError * turn_gain;
   ```

## Notes:

Reduce Exposure & Increase Gain:
This can help reduce motion blur, but further testing is required to fine-tune these settings.
