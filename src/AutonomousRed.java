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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous(name = "Red autonomous", group = "Linear Opmode")
public class AutonomousRed extends LinearOpMode {

    // Hardware components
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotorEx armMotor;
    private Servo wrist;
    private CRServo intake;
    private ColorSensor colorSensor;
    private RobotUtils robotUtils;
    private Pose3D position;

    // Field dimensions and obstacle location
    private static final double FIELD_SIZE = 144.0; // 144 inches (12x12 ft field)
    private static final double OBSTACLE_WIDTH = 44.5; // in inches
    private static final double OBSTACLE_HEIGHT = 26.5; // in inches

    /* This constant is the number of encoder ticks for each degree of rotation of the arm. */
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
    final double ARM_DOWN                  = 180 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double CLIP_SPECIMEN             = 180 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;
    final double INTAKE_YEET       =  1;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    final double WRIST_FOLDED_LEFT = 0.1667;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int) ARM_COLLAPSED_INTO_ROBOT;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Create variables that we will use
        double wristPosition = WRIST_FOLDED_IN;
        double intakePower = INTAKE_OFF;
        double[] coordinates;
        boolean debugEnabled = true;
        Pose3D pose = null;

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
        robotUtils.setColor(robotUtils.RED);
        robotUtils.setColorSensor(colorSensor);
        robotUtils.initCamera(this);

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
        wristPosition = WRIST_FOLDED_IN;
        intakePower = INTAKE_OFF;

        wrist.setPosition(wristPosition);

        robotUtils.setYawIMU(20);

        double[] startCoordinates = {18, -68, 0};
        double[] targetCoordinates = {0, -24, 0};
        double[] currentPosition = {18, -68, 0};
        double dx = targetCoordinates[0] - startCoordinates[0];
        double dy = targetCoordinates[1] - startCoordinates[1];
        double distance = Math.hypot(dx, dy);

        wrist.setPosition(wristPosition);
        robotUtils.driveStraight(this, distance, 0.5, robotUtils.getYawIMU(), debugEnabled);
        currentPosition[0] += dx;
        currentPosition[1] += dy;
        robotUtils.driveStraight(this, 15, 0.5, robotUtils.getYawIMU(), debugEnabled); // Align
        robotUtils.driveStraight(this, -10, 0.5, robotUtils.getYawIMU(), debugEnabled); // Back up
        robotUtils.moveArm(this, (int) CLIP_SPECIMEN); // Move arm to right position  
        robotUtils.driveStraight(this, -5, 0.5, robotUtils.getYawIMU(), debugEnabled); // Back up
        intake.setPower(INTAKE_DEPOSIT); // Deposit specimen
        robotUtils.moveArm(this, (int) ARM_DOWN);


        /* *********************** Drive the blocks to the human player *********************** */ 
        robotUtils.moveArm(this, (int) ARM_COLLAPSED_INTO_ROBOT); // Move arm back to original position
        // Intake off
        intake.setPower(INTAKE_OFF);
        // Turn towards the wall where the blocks are
        robotUtils.turnDegrees(this, -90, debugEnabled); //Turn to the right
        // Drive to just before the first block
        robotUtils.driveStraight(this, 36, 0.5, robotUtils.getYawIMU(), debugEnabled);
        // Turn 90 degrees left and move forward to get in front of the block
        robotUtils.turnDegrees(this, -90, debugEnabled);
        robotUtils.driveStraight(this, -35, 0.5, robotUtils.getYawIMU(), debugEnabled);
        // Turn to align with the block and the observation zone
        robotUtils.turnDegrees(this, 22, debugEnabled);
        // Drive to the observation zone through the block to drag it along
        robotUtils.driveStraight(this, 50, 0.5, robotUtils.getYawIMU(), debugEnabled);
        // Back up a bit
        // robotUtils.driveStraight(this, -10, 0.5, robotUtils.getYawIMU(), debugEnabled);
        // // Turn towards the second block
        // robotUtils.turnDegrees(this, -20, debugEnabled);
        // // Move past the second block
        // robotUtils.driveStraight(this, -40, 0.5, robotUtils.getYawIMU(), debugEnabled);
        // // Align with the second block and move to the observation zone
        // robotUtils.turnDegrees(this, -5, debugEnabled);
        // robotUtils.driveStraight(this, 50, 0.5, robotUtils.getYawIMU(), debugEnabled);
        /* *********************** Drive the blocks to the human player *********************** */ 
    }
}

