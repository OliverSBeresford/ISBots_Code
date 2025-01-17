package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "testAprilTag (Blocks to Java)")
public class testAprilTag extends LinearOpMode {

    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    Position cameraPosition;
    YawPitchRollAngles cameraOrientation;
    VisionPortal myVisionPortal;
    RobotUtils robotUtils;

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

        // Initialize the robot utilities.
        robotUtils = new RobotUtils();
        
        // Initialize the apriltag processor.
        robotUtils.VisionComponents visionComponents = robotUtils.initAprilTag();
        myVisionPortal = visionComponents.visionPortal;
        myAprilTagProcessor = visionComponents.aprilTagProcessor;

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
                // Field-relative data
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "    (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getPitch(), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getYaw(), 6, 1) + "    (deg)");
                // Raw data (relative to the apriltag)
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ (RAW) " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getPosition().z, 6, 1) + "    (inch)");
                telemetry.addLine("PRY (RAW) " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getOrientation().getPitch(), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.rawPose.getOrientation().getYaw(), 6, 1) + "    (deg)");
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
