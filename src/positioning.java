package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.utils.RobotUtils;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Test Robot Navigation")
public class Positioning extends LinearOpMode {

    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    Position cameraPosition;
    YawPitchRollAngles cameraOrientation;
    VisionPortal myVisionPortal;

    /**
     * This OpMode illustrates the basics of AprilTag based localization.
     *
     * For an introduction to AprilTags, see the FTC-DOCS link below:
     * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
     *
     * In this sample, any visible tag ID will be detected and displayed, but only
     * tags that are included in the default "TagLibrary" will be used to compute the
     * robot's location and orientation. This default TagLibrary contains the current
     * Season's AprilTags and a small set of "test Tags" in the high number range.
     *
     * When an AprilTag in the TagLibrary is detected, the SDK provides
     * location and orientation of the robot, relative to the field origin. This
     * information is provided in the "robotPose" member of the returned "detection".
     *
     * To learn about the Field Coordinate System that is defined for
     * FTC (and used by this OpMode), see the FTC-DOCS link below:
     * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
     */
    @Override
    public void runOpMode() {
        USE_WEBCAM = true;
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
        cameraPosition = new Position(DistanceUnit.CM, -10, 0, 7, 0);
        // YawPitchRollAngles(AngleUnit angleUnit, double yaw, double pitch, double roll, long acquisitionTime)
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        // Initialize AprilTag before waitForStart.
        initAprilTag();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetryAprilTag();
            // Push telemetry to the Driver Station.
            telemetry.update();
            if (gamepad1.dpad_down) {
                // Temporarily stop the streaming session. This can save CPU
                // resources, with the ability to resume quickly when needed.
                myVisionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                // Resume the streaming session if previously stopped.
                myVisionPortal.resumeStreaming();
            }
            // Share the CPU.
            sleep(20);
        }
    }

    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        // Create an AprilTagProcessor by calling build.
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a recognized AprilTag.
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        telemetry.addData("Apriltags detected", myAprilTagDetections);
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            telemetry.addData("Metadata", myAprilTagDetection.metadata);
        }
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item2;
            // Display info about the detection.
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "    (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getPitch(), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getYaw(), 6, 1) + "    (deg)");
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    }
}
