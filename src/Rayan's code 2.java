package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "HookBlocksOnBars", group = "Competition")
public class HookBlocksOnBars extends LinearOpMode {

    // Declare hardware variables
    private DcMotor leftDrive, rightDrive, liftMotor;
    private Servo hookServo;
    private OpenCvCamera camera;

    // Constants
    private static final double DRIVE_SPEED = 0.5;
    private static final double LIFT_SPEED = 0.6;
    private static final int LIFT_TARGET_POSITION = 1000; // Adjust as needed for lifting height
    private static final double HOOK_SERVO_LOCK_POSITION = 0.2;
    private static final double HOOK_SERVO_RELEASE_POSITION = 0.8;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        hookServo = hardwareMap.get(Servo.class, "hook_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Reset motor encoders
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        camera.setPipeline(new HookAlignmentPipeline()); // Define custom pipeline for detecting bars
        camera.openCameraDeviceAsync(() -> camera.startStreaming(640, 480));

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        // Wait for start signal
        waitForStart();

        if (opModeIsActive()) {
            // Main autonomous sequence
            moveForward(24, DRIVE_SPEED); // Move forward to initial position
            alignWithBar(); // Use camera to align with bar
            liftBlock(); // Lift block to hook height
            hookBlock(); // Attach block to bar
            releaseBlock(); // Release block from hook
            backAway(12, DRIVE_SPEED); // Move back to starting position
        }
    }

    // Function to move forward a certain distance (in inches)
    private void moveForward(double inches, double speed) {
        int targetPosition = (int) (inches * 100); // Convert inches to encoder counts (adjust for your robot)
        leftDrive.setTargetPosition(targetPosition);
        rightDrive.setTargetPosition(targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        while (leftDrive.isBusy() && rightDrive.isBusy() && opModeIsActive()) {
            telemetry.addData("Moving Forward", "Target: %d", targetPosition);
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // Function to lift block
    private void liftBlock() {
        liftMotor.setTargetPosition(LIFT_TARGET_POSITION);
        liftMotor.setPower(LIFT_SPEED);

        while (liftMotor.isBusy() && opModeIsActive()) {
            telemetry.addData("Lifting Block", "Position: %d", liftMotor.getCurrentPosition());
            telemetry.update();
        }

        liftMotor.setPower(0);
    }

    // Function to hook the block onto the bar
    private void hookBlock() {
        hookServo.setPosition(HOOK_SERVO_LOCK_POSITION);
        sleep(1000); // Wait for hook to engage
        telemetry.addData("Hooking Block", "Locked");
        telemetry.update();
    }

    // Function to release the block
    private void releaseBlock() {
        hookServo.setPosition(HOOK_SERVO_RELEASE_POSITION);
        sleep(1000); // Wait for hook to release
        telemetry.addData("Releasing Block", "Released");
        telemetry.update();
    }

    // Function to align with the bar using the camera
    private void alignWithBar() {
        telemetry.addData("Aligning", "Using Camera");
        telemetry.update();

        // Placeholder logic for alignment
        while (!isAligned() && opModeIsActive()) {
            // Adjust robot position based on camera feedback
            leftDrive.setPower(0.2);
            rightDrive.setPower(-0.2);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // Function to check if the robot is aligned with the bar
    private boolean isAligned() {
        // Placeholder logic - replace with actual camera alignment logic
        return true;
    }

    // Function to move backward a certain distance
    private void backAway(double inches, double speed) {
        moveForward(-inches, speed);
    }

    // Custom pipeline for detecting bars (define your logic here)
    class HookAlignmentPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Analyze frame and provide alignment feedback
            // Add your logic for detecting and aligning with the bar
            return input;
        }
    }
}
