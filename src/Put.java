package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotUtils;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Pathfinding & Block Placement", group = "Linear Opmode")
public class Put extends LinearOpMode {

    // Hardware components
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorEx armMotor;
    private Servo wrist;
    private CRServo intake;
    private ColorSensor colorSensor;
    private RobotUtils robotUtils;
    private static final int colorThreshold = 500; // Minimum color value for a block to be detected

    // Field dimensions and obstacle location
    private static final double FIELD_SIZE = 144.0; // 144 inches (12x12 ft field)
    private static final double OBSTACLE_WIDTH = 44.5; // in inches
    private static final double OBSTACLE_HEIGHT = 26.5; // in inches

    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.
    */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_SEARCH                = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    final double WRIST_FOLDED_LEFT = 0.1667;

    // Values for detecting blocks
    private static final int[] RED_RGB = {4000, 2000, 1200};
    private static final int[] BLUE_RGB = {1000, 2200, 4500};
    private static final int[] YELLOW_RGB = {6500, 8500, 2000};
    private static final int TOLERANCE = 500;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Create variables that we will use
        double wristPosition = WRIST_FOLDED_IN;
        double intakePower = INTAKE_OFF;

        // Initialize hardware
        leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initializing the imu based on how the driver hub is oriented on the robot
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));

        // Initialize the robot utility class to have access to useful methods
        robotUtils = new RobotUtils();
        robotUtils.setHardware(leftDrive, rightDrive, imu, intake, wrist, armMotor);

        // Set motor directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        
        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Indicate initialization complete
        telemetry.addLine("Initialization Complete. Waiting for Start...");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        armPosition = (int) ARM_SEARCH;
        wristPosition = WRIST_FOLDED_OUT;
        intakePower = INTAKE_COLLECT;

        while (opModeIsActive()) {
            telemetry.update();

            // Setting the intake power
            intake.setPower(intakePower);
            
            /* Changing the wrist's position */
            wrist.setPosition(wristPosition);
            
            // Move the arm to the correct position
            moveArm((int) armPosition);

            // Step 2: Detect the color of the block
            if (isYellowBlock()) {
                telemetry.addLine("Yellow block detected!");
                // moveToBasket(-72, -72); // We are red
            } else if (isBlueBlock()) {
                telemetry.addLine("Blue block detected!");
                // moveToBasket(72, 72); // Moving to the other basket, but in practice we would just leave it if we are the red team
            } else if (isRedBlock()) {
                telemetry.addLine("Red block detected!");
                // moveToBasket(-72, -72); // We are red
            } else {
                telemetry.addLine("Red: " + colorSensor.red() + " Green: " + colorSensor.green() + " Blue: " + colorSensor.blue());

                leftDrive.setPower(0);
                rightDrive.setPower(0);

                continue;
            }

            armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
            wristPosition = WRIST_FOLDED_IN;
            intakePower = INTAKE_OFF;
        }
    }

    // Helper functions

    private void moveForward(double power, double distance) {
        robotUtils.driveStraight(this, distance, power, distance);
    }

    private void moveBackward(double power, long duration) {
        leftDrive.setPower(-power);
        rightDrive.setPower(-power);
        sleep(duration);
        stopAllMotors();
    }

    private void stopAllMotors() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
        intake.setPower(0);
    }

    private void moveToBasket(int basketX, int basketY) {
        telemetry.addLine("Navigating to Basket...");
        telemetry.update();

        // // Navigate around the central obstacle
        // if (basketY > OBSTACLE_HEIGHT / 2) {
        //     // If basket is beyond the obstacle, go around the top
        //     moveForward(0.5, 1000); // Adjust timing
        //     turnLeft(90);
        //     moveForward(0.5, 1000); // Adjust timing
        // } else {
        //     // If basket is below the obstacle, go around the bottom
        //     moveForward(0.5, 1000); // Adjust timing
        //     turnRight(90);
        //     moveForward(0.5, 1000); // Adjust timing
        // }

        telemetry.addLine("Reached Basket Area.");
        telemetry.update();
    }

    private void pickUpBlock() {
        intake.setPower(INTAKE_COLLECT); // Start the intake to pick up the block
        sleep(1000);
        intake.setPower(INTAKE_OFF); // Stop the intake
        moveArm((int) ARM_SCORE_SAMPLE_IN_LOW); // Lift the block for transport
    }

    private void moveArm(int position) {
        armMotor.setTargetPosition(position); // Adjust based on arm's vertical position for baskets
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(2100);
        while (armMotor.isBusy() && opModeIsActive()) {
            // Wait for the arm to reach the position
        }
    }

    private void dropBlockInBasket() {
        wrist.setPosition(WRIST_FOLDED_OUT); // Adjust wrist to drop position
        intake.setPower(INTAKE_DEPOSIT);
    }

    private void resetArmPosition() {
        armMotor.setTargetPosition(0); // Return the arm to starting position
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(2100);
        while (armMotor.isBusy() && opModeIsActive()) {
            // Wait for the arm to reset
        }
    }

    private boolean isBlockPresent() {
        return (colorSensor.red() > colorThreshold || colorSensor.green() > colorThreshold || colorSensor.blue() > colorThreshold);
    }

    private boolean isRedBlock() {
        return isBlockPresent() &&
               isWithinRange(colorSensor.red(), RED_RGB[0], TOLERANCE) &&
               isWithinRange(colorSensor.green(), RED_RGB[1], TOLERANCE) &&
               isWithinRange(colorSensor.blue(), RED_RGB[2], TOLERANCE);
    }
    
    private boolean isBlueBlock() {
        return isBlockPresent() &&
               isWithinRange(colorSensor.red(), BLUE_RGB[0], TOLERANCE) &&
               isWithinRange(colorSensor.green(), BLUE_RGB[1], TOLERANCE) &&
               isWithinRange(colorSensor.blue(), BLUE_RGB[2], TOLERANCE);
    }
    
    private boolean isYellowBlock() {
        return isBlockPresent() &&
               isWithinRange(colorSensor.red(), YELLOW_RGB[0], TOLERANCE) &&
               isWithinRange(colorSensor.green(), YELLOW_RGB[1], TOLERANCE) &&
               isWithinRange(colorSensor.blue(), YELLOW_RGB[2], TOLERANCE);
    }

    private void turnLeft(double degrees) {
        // Use the turnDegrees function to turn left using the IMU
        robotUtils.turnDegrees(this, degrees);
        telemetry.addData("Turning left", "%d degrees", degrees);
        telemetry.update();
        sleep(10); // Placeholder
    }

    private void turnRight(double degrees) {
        // Use the turnDegrees function to turn right using the IMU
        robotUtils.turnDegrees(this, degrees);
        telemetry.addData("Turning right", "%d degrees", degrees);
        telemetry.update();
        sleep(10); // Placeholder
    }

    private boolean isWithinRange(int actual, int target, int tolerance) {
        return Math.abs(actual - target) <= tolerance;
    }

}

