package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FTC Autonomous Mode", group = "Linear Opmode")
public class FTCAutonomous extends LinearOpMode {

    // Declare motors and sensors
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor armMotor = null;
    private ColorSensor colorSensor = null;

    // Constants for motor power and distances
    private static final double DRIVE_POWER = 0.6;
    private static final double ARM_POWER = 0.5;
    private static final long PICKUP_TIME = 1000; // 1 second
    private static final long DROP_TIME = 1000; // 1 second

    // Helper function to drive forward/backward
    private void drive(double power, long time) throws InterruptedException {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // Helper function to turn left/right
    private void turn(double power, long time) throws InterruptedException {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // Helper function to operate the arm
    private void moveArm(double power, long time) throws InterruptedException {
        armMotor.setPower(power);
        sleep(time);
        armMotor.setPower(0);
    }

    // Function to detect block color
    private String detectBlockColor() {
        if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()) {
            return "Red";
        } else if (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()) {
            return "Blue";
        } else if (colorSensor.green() > colorSensor.red() && colorSensor.green() > colorSensor.blue()) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            ElapsedTime runtime = new ElapsedTime();

            try {
                // Drive to the first block
                drive(DRIVE_POWER, 1500); // Assume ~1.5 meters forward

                // Detect block color
                String color = detectBlockColor();
                telemetry.addData("Detected Color", color);
                telemetry.update();

                // Pick up the block
                moveArm(ARM_POWER, PICKUP_TIME);

                // Turn towards the basket
                if (color.equals("Red")) {
                    turn(DRIVE_POWER, 500); // Adjust time for ~90-degree turn
                } else if (color.equals("Blue")) {
                    turn(-DRIVE_POWER, 500);
                } else if (color.equals("Yellow")) {
                    turn(DRIVE_POWER, 250); // Adjust time for ~45-degree turn
                }

                // Drive to the basket
                drive(DRIVE_POWER, 1500);

                // Drop the block
                moveArm(-ARM_POWER, DROP_TIME);

                // Return to starting position (optional)
                drive(-DRIVE_POWER, 1500);

            } catch (InterruptedException e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.update();
            }
        }
    }
}