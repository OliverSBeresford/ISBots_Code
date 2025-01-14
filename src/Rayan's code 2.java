package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonomousBlockHooking", group = "FTC")
public class AutonomousBlockHooking extends LinearOpMode {

    // Declare motors, sensors, and servos
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor verticalDrive = null; // For lifting the hook
    private Servo hookServo = null;
    private ColorSensor colorSensor = null;

    // Constants
    private static final double DRIVE_POWER = 0.5; // Speed for driving
    private static final double HOOK_POSITION_OPEN = 0.0; // Servo open
    private static final double HOOK_POSITION_CLOSED = 1.0; // Servo closed
    private static final int HOOKING_DELAY_MS = 1000; // Time for hooking in milliseconds

    @Override
    public void runOpMode() {
        // Hardware mapping
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        verticalDrive = hardwareMap.get(DcMotor.class, "vertical_drive");
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
                    hookBlock();
                } else {
                    telemetry.addData("Status", "No blocks detected.");
                    telemetry.update();
                    break;
                }
            }
        }
    }

    private void moveToCoordinates(double x, double y) {
        // Simulate driving to the block area (implement coordinate-based navigation if needed)
        telemetry.addData("Moving to", "Coordinates (" + x + ", " + y + ")");
        telemetry.update();
        sleep(2000); // Placeholder for actual movement logic
    }

    private boolean detectBlock() {
        // Use color sensor to identify blocks
        telemetry.addData("Color Sensor Red", colorSensor.red());
        telemetry.addData("Color Sensor Blue", colorSensor.blue());
        telemetry.addData("Color Sensor Green", colorSensor.green());
        telemetry.update();

        // Example logic: Detect a block if any color reading is above a threshold
        return (colorSensor.red() > 100 || colorSensor.blue() > 100 || colorSensor.green() > 100);
    }

    private void hookBlock() {
        // Hook the block using the servo
        telemetry.addData("Status", "Hooking block");
        telemetry.update();
        hookServo.setPosition(HOOK_POSITION_CLOSED);
        sleep(HOOKING_DELAY_MS); // Wait for the hook to grab the block

        // Lift the block
        verticalDrive.setPower(0.5);
        sleep(500); // Lift duration (adjust as needed)
        verticalDrive.setPower(0);

        // Release the block
        hookServo.setPosition(HOOK_POSITION_OPEN);
        telemetry.addData("Status", "Block hooked and lifted");
        telemetry.update();
    }
}