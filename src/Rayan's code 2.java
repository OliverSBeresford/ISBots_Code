package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonomousHookingFinal", group = "FTC")
public class AutonomousHookingFinal extends LinearOpMode {

    // Declare motors, sensors, and servos
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null; // For lifting/lowering the arm
    private DcMotor intakeMotor = null; // For bringing in the block (wheelie thing)
    private Servo hookServo = null;
    private ColorSensor colorSensor = null;

    // Constants
    private static final double DRIVE_POWER = 0.5; // Speed for driving
    private static final double INTAKE_POWER = 0.8; // Power for intake motor
    private static final double ARM_POWER = 0.5; // Power for arm motor
    private static final double HOOK_POSITION_OPEN = 0.0; // Servo open
    private static final double HOOK_POSITION_CLOSED = 1.0; // Servo closed
    private static final int ARM_LIFT_DURATION_MS = 1000; // Time for lifting arm in milliseconds
    private static final int HOOKING_DELAY_MS = 1000; // Time for hooking in milliseconds

    @Override
    public void runOpMode() {
        // Hardware mapping
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        hookServo = hardwareMap.get(Servo.class, "hook_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Reverse one motor for tank drive
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the start of the match
        waitForStart();

        if (opModeIsActive()) {
            // Drive to block area
            moveToCoordinates(72, -72);

            // Detect and hook blocks
            while (opModeIsActive()) {
                if (detectBlock()) {
                    processAndHookBlock();
                } else {
                    telemetry.addData("Status", "No blocks detected.");
                    telemetry.update();
                    break;
                }
            }
        }
    }

    /**
     * Moves the robot to a specific set of coordinates.
     * Replace this placeholder with actual navigation logic.
     *
     * @param x Target X-coordinate.
     * @param y Target Y-coordinate.
     */
    private void moveToCoordinates(double x, double y) {
        telemetry.addData("Moving to", "Coordinates (" + x + ", " + y + ")");
        telemetry.update();
        sleep(2000); // Placeholder for actual movement logic
    }

    /**
     * Detects if a block is present using the color sensor.
     *
     * @return True if a block is detected, otherwise false.
     */
    private boolean detectBlock() {
        telemetry.addData("Color Sensor Red", colorSensor.red());
        telemetry.addData("Color Sensor Blue", colorSensor.blue());
        telemetry.addData("Color Sensor Green", colorSensor.green());
        telemetry.update();

        // Example logic: Detect a block if any color reading is above a threshold
        return (colorSensor.red() > 100 || colorSensor.blue() > 100 || colorSensor.green() > 100);
    }

    /**
     * Handles the full sequence for picking up and hooking a block.
     */
    private void processAndHookBlock() {
        telemetry.addData("Status", "Processing block");
        telemetry.update();

        // Step 1: Activate intake motor to pull the block in
        intakeMotor.setPower(INTAKE_POWER);
        moveSlightlyToAlign(); // Adjust position slightly if needed
        sleep(1500); // Allow time for block to be fully inside arm
        intakeMotor.setPower(0);

        // Step 2: Lift the arm to the required height
        liftArm();

        // Step 3: Drive forward to the colored bar
        driveToColoredBar();

        // Step 4: Lower the arm to the hooking position
        lowerArmToHook();

        // Step 5: Drive backward to complete hooking
        driveBackwardAfterHooking();

        telemetry.addData("Status", "Block successfully hooked");
        telemetry.update();
    }

    /**
     * Slightly adjusts the robot's position to align with the block.
     * Modify this logic based on your specific hardware setup.
     */
    private void moveSlightlyToAlign() {
        telemetry.addData("Adjusting position", "Aligning with block");
        telemetry.update();
        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(-DRIVE_POWER); // Turn slightly
        sleep(500); // Adjust time for fine alignment
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Lifts the arm to the required height for hooking.
     */
    private void liftArm() {
        telemetry.addData("Status", "Lifting arm");
        telemetry.update();
        armMotor.setPower(ARM_POWER);
        sleep(ARM_LIFT_DURATION_MS);
        armMotor.setPower(0);
    }

    /**
     * Drives the robot forward to reach the colored bar.
     */
    private void driveToColoredBar() {
        telemetry.addData("Status", "Driving to colored bar");
        telemetry.update();
        leftDrive.setPower(DRIVE_POWER);
        rightDrive.setPower(DRIVE_POWER);
        sleep(2000); // Adjust time based on distance to the bar
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    /**
     * Lowers the arm to hook the block onto the bar.
     */
    private void lowerArmToHook() {
        telemetry.addData("Status", "Lowering arm to hook");
        telemetry.update();
        armMotor.setPower(-ARM_POWER); // Lower arm
        sleep(500); // Lower to a predefined angle
        armMotor.setPower(0);

        // Further lower arm for the hook to secure
        armMotor.setPower(-ARM_POWER);
        sleep(200); // Fine adjustment for 10-degree additional lowering
        armMotor.setPower(0);
    }

    /**
     * Drives the robot backward to complete the hooking process.
     */
    private void driveBackwardAfterHooking() {
        telemetry.addData("Status", "Driving backward to finalize hooking");
        telemetry.update();
        leftDrive.setPower(-DRIVE_POWER);
        rightDrive.setPower(-DRIVE_POWER);
        sleep(1000); // Adjust time for backward movement
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}