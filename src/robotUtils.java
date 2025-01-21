package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotUtils {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private IMU imu;

    /* These are the various functions to initialize whatever parts of the RobotUtils class you need
     * 
     * setHardware: Initializes the hardware for the robot.
     * Parameters: DcMotor _leftDrive - The left drive motor.
     *             DcMotor _rightDrive - The right drive motor.
     *             IMU _imu - The IMU sensor.
     */
    public void setHardware(DcMotor _leftDrive, DcMotor _rightDrive, IMU _imu) {
        leftDrive = _leftDrive;
        rightDrive = _rightDrive;
        imu = _imu;
    }

    /* These functions relate to physical behaior of the robot */
    public void turnDegrees(LinearOpMode opMode, double turnAngle) {
        /* This function turn the robot a certain number of degrees
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         *             double turnAngle - The number of degrees to turn the robot.
         */

        // Making sure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null) {
            return;
        }

        double currentAngle;
        double error;
        double turnPower;
        double kP = 0.02; // Proportional constant, adjust for your robot
        double minPower = 0.1; // Minimum power to ensure the motors turn the robot

        // Determine the direction to turn and target heading
        double startingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double targetHeading = startingAngle + turnAngle;

        if (targetHeading > 180) {
            targetHeading -= 360; // Wrap around for angles > 180 degrees
        } else if (targetHeading < -180) {
            targetHeading += 360; // Wrap around for angles < -180 degrees
        }

        // Control loop to adjust motor power
        do {
            currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = targetHeading - currentAngle;

            // Normalize error to the range -180 to 180 degrees
            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            // Calculate motor power based on proportional control
            turnPower = kP * error;

            // Ensure the power is sufficient to move the robot but not excessive
            if (Math.abs(turnPower) < minPower) {
                turnPower = Math.copySign(minPower, turnPower);
            }
            if (Math.abs(turnPower) > 1) {
                turnPower = Math.copySign(1, turnPower);
            }

            // Apply motor power for turning
            leftDrive.setPower(turnPower);
            rightDrive.setPower(-turnPower);

            // Allow time for the motors to respond
            opMode.sleep(10);
        } while (Math.abs(error) > 1 && opMode.opModeIsActive());

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void driveStraight(LinearOpMode opMode, double distanceInInches, double power, double targetHeading) {
        /* This function drives the robot a certain distance in inches
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         *             double distanceInInches - The distance to drive the robot in inches.
         *             double power - The power to drive the robot at (number from 0 to 1).
         *             double targetHeading - The heading to drive the robot at.
         */

        // Making sure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null) {
            return;
        }

        int ticksPerRevolution = ((((1 + (46 / 17))) * (1 + (46 / 11))) * 28); // Got this from GOBilda
        double wheelDiameter = 3.77953; // In inches
        double wheelCircumference = Math.PI * wheelDiameter;
        int targetTicks = (int) ((distanceInInches / wheelCircumference) * ticksPerRevolution);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while (leftDrive.isBusy() && rightDrive.isBusy() && opMode.opModeIsActive()) {
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;

            double correction = error * 0.02; // Proportional constant
            leftDrive.setPower(power + correction);
            rightDrive.setPower(power - correction);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /* These functions relate to the AprilTag detection system. 
     * 
    */
    public VisionComponents initAprilTag(LinearOpMode opMode) {
        // Variables to store the position and orientation of the camera on the robot. Setting these
        // values requires a definition of the axes of the camera and robot:
        // Camera axes:
        // Origin location: Center of the lens
        // Axes orientation: +x right, +y down, +z forward (from camera's perspective)
        // Robot axes (this is typical, but you can define this however you want):
        // Origin location: Center of the robot at field height
        // Axes orientation: +x right, +y forward, +z upward
        // Position:
        // If all values are zero (no translation), that implies the camera is at the center of the
        // robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
        // inches above the ground - you would need to set the position to (-5, 7, 12).
        // Orientation:
        // If all values are zero (no rotation), that implies the camera is pointing straight up. In
        // most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
        // the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
        // it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
        // to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
        // Position(DistanceUnit unit, double x, double y, double z, long acquisitionTime)
        Position cameraPosition = new Position(DistanceUnit.CM, 5, 21.5, 18, 0);
        // YawPitchRollAngles(AngleUnit angleUnit, double yaw, double pitch, double roll, long acquisitionTime)
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        
        // Create an AprilTagProcessor by calling build.
        // Create the AprilTag processor and assign it to a variable.
        AprilTagProcessor myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        VisionPortal myVisionPortal = myVisionPortalBuilder.build();
        
        return new VisionComponents(myVisionPortal, myAprilTagProcessor);
    }

    public Pose3D getData(LinearOpMode opMode, AprilTagProcessor myAprilTagProcessor, bool debugEnabled) {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        // Display the number of AprilTags detected.
        if (debugEnabled) {
            opMode.telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        }

        // Iterate through list and call a function to return info for the first recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            if (myAprilTagDetection.metadata != null) {
                // Return the data without printing it if debug is disabled
                if (!debugEnabled) {
                    return myAprilTagDetection.robotPose;
                }

                // Field-relative data
                opMode.telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                opMode.telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "    (inch)");
                opMode.telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getPitch(), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getYaw(), 6, 1) + "    (deg)");
                
                // Raw data (relative to the apriltag)
                opMode.telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                opMode.telemetry.addLine("XYZ (RAW) " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getPosition().z, 6, 1) + "    (inch)");
                opMode.telemetry.addLine("PRY (RAW) " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getOrientation().getPitch(), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getOrientation().getYaw(), 6, 1) + "    (deg)");

                // Useful information for debugging
                opMode.telemetry.addLine("");
                opMode.telemetry.addLine("key:");
                opMode.telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
                opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

                return myAprilTagDetection.robotPose;
            } else {
                opMode.telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                opMode.telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
                return null;
            }
        }
    }

    /* Classes used to return specific data from the RobotUtils class */
    public static class VisionComponents {
        VisionPortal visionPortal;
        AprilTagProcessor aprilTagProcessor;

        public VisionComponents(VisionPortal vision_portal, AprilTagProcessor apriltag_processor) {
            visionPortal = vision_portal;
            aprilTagProcessor = apriltag_processor; 
        }
    }
}
