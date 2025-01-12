package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.utils.RobotUtils;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This robot has a two-motor differential-steered (sometimes called tank or skid steer) drivetrain.
 * With a left and right drive motor.
 * The drive on this robot is controlled in an "Arcade" style, with the left stick Y axis
 * controlling the forward movement and the right stick X axis controlling rotation.
 * This allows easy transition to a standard "First Person" control of a
 * mecanum or omnidirectional chassis.
 *
 * The drive wheels are 96mm diameter traction (Rhino) or omni wheels.
 * They are driven by 2x 5203-2402-0019 312RPM Yellow Jacket Planetary Gearmotors.
 *
 * This robot's main scoring mechanism includes an arm powered by a motor, a "wrist" driven
 * by a servo, and an intake driven by a continuous rotation servo.

 * This OpMode uses the motor's encoder and the RunToPosition method to drive the arm to
 * specific setpoints. These are defined as a number of degrees of rotation away from the arm's
 * starting position.
 *
 * Make sure that the arm is reset into the robot, and the wrist is folded in before
 * you run start the OpMode. The motor's encoder is relative and will move the number of degrees
 * you request it to based on the starting position. So if it starts too high, all the motor
 * setpoints will be wrong.
 *
 * The wrist is powered by a goBILDA Torque Servo (2000-0025-0002).
 *
 * The intake wheels are powered by a goBILDA Speed Servo (2000-0025-0003) in Continuous Rotation mode.
 */


@TeleOp
public class ISBotsTeleOp extends LinearOpMode {
    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null; //the left drivetrain motor
    public DcMotor  rightDrive  = null; //the right drivetrain motor
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo


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
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.5;
    final double WRIST_FOLDED_LEFT = 0.1667;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    private IMU imu;

    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;
        double wristPosition = WRIST_FOLDED_IN;
        double lastWristPosition = WRIST_FOLDED_IN;
        double intakePower = INTAKE_OFF;
        boolean lastAState = false;
        boolean lastXState = false;
        boolean lastBState = false;
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));


        /* Define and Initialize Motors */
        leftDrive  = hardwareMap.get(DcMotor.class, "leftMotor"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "rightMotor"); //the right drivetrain motor
        armMotor   = hardwareMap.get(DcMotor.class, "armMotor"); //the arm motor


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until the driver presses stop */
        while (opModeIsActive()) {

            /* Set the drive and turn variables to follow the joysticks on the gamepad.
            the joysticks decrease as you push them up. So reverse the Y axis. */
            forward = -gamepad1.left_stick_y;
            rotate  = gamepad1.right_stick_x;


            /* Add how much we rotate with how much we are moving forwards
            to calculate how much power each motor needs. Positive value on 
            the right_stick_x means rotation to the right due to higher power
            on the left side and vice versa :0
            */

            left  = forward + rotate;
            right = forward - rotate;

            /* Normalize the values so neither exceed +/- 1.0 */
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

            /* Set the motor power to the variables we've mixed and normalized */
            leftDrive.setPower(left);
            rightDrive.setPower(right);



            /* Intake control
            If the user presses A, it sets the intake power to the variable that we made for it
            If the user presses X, it sets the servo to Off.
            And if the user presses B it reveres the servo to spit out the element.*/

            // If you click a (do only once before you take your finger off)
            if (gamepad1.a && !lastAState) {
                if (intakePower == INTAKE_OFF) {
                    intakePower = INTAKE_COLLECT;
                } else if (intakePower == INTAKE_COLLECT) {
                    intakePower = INTAKE_OFF;
                }
            }
            // If you click x (do only once before you take your finger off)
            else if (gamepad1.x && !lastXState) {
                if (wristPosition == WRIST_FOLDED_IN || wristPosition == WRIST_FOLDED_LEFT) {
                    wristPosition = WRIST_FOLDED_OUT;
                } else if (wristPosition == WRIST_FOLDED_OUT && lastWristPosition == WRIST_FOLDED_IN) {
                    wristPosition = WRIST_FOLDED_LEFT;
                    lastWristPosition = WRIST_FOLDED_LEFT;
                } else if (wristPosition == WRIST_FOLDED_OUT && lastWristPosition == WRIST_FOLDED_LEFT) {
                    wristPosition = WRIST_FOLDED_IN;
                    lastWristPosition = WRIST_FOLDED_IN;
                }
            }
            // If you click b (do only once before you take your finger off)
            if (gamepad1.b && !lastBState) {
                if (intakePower == INTAKE_OFF) {
                    intakePower = INTAKE_DEPOSIT;
                } else if (intakePower == INTAKE_DEPOSIT) {
                    intakePower = INTAKE_OFF;
                }
            }
            // Saving the states of each button
            lastAState = gamepad1.a;
            lastBState = gamepad1.b;
            lastXState = gamepad1.x;

            /* Arm control */
            if(gamepad1.right_bumper) {
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                wristPosition = WRIST_FOLDED_OUT;
                // wrist.setPosition(WRIST_FOLDED_OUT);
                intakePower = INTAKE_COLLECT;
            }

            else if (gamepad1.left_bumper) {
                /* This is about 20° up from the collecting position to clear the barrier */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.y) {
                /* This is the correct height to score the sample in the LOW BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
            }

            else if (gamepad1.dpad_left) {
                /* This turns off the intake, folds in the wrist, and moves the arm
                back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                intakePower = INTAKE_OFF;
                // wrist.setPosition(WRIST_FOLDED_IN);
                wristPosition = WRIST_FOLDED_IN;
            }

            else if (gamepad1.dpad_right) {
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;
                // wrist.setPosition(WRIST_FOLDED_IN);
                wristPosition = WRIST_FOLDED_IN;
            }

            else if (gamepad1.dpad_up) {
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                intakePower = INTAKE_OFF;
                // wrist.setPosition(WRIST_FOLDED_IN);
                wristPosition = WRIST_FOLDED_IN;
            }

            else if (gamepad1.dpad_down) {
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                intakePower = INTAKE_OFF;
                // wrist.setPosition(WRIST_FOLDED_IN);
                wristPosition = WRIST_FOLDED_IN;
            }


            /* Here we create a "fudge factor" for the arm position.
            This allows you to adjust the arm position slightly with the gamepad triggers.
            */

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            /* Here we set the target position of the arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
            armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

            ((DcMotorEx) armMotor).setVelocity(2100);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* Changing the wrist's position */
            wrist.setPosition(wristPosition);

            // Changing the intake power
            intake.setPower(intakePower);

            /* Check to see if our arm is over the current limit, and report via telemetry. */
            if (((DcMotorEx) armMotor).isOverCurrent()){
                telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
            }


            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("armTarget: ", armMotor.getTargetPosition());
            telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());
            telemetry.addData("Angular Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES));
            telemetry.addData("Orientation", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));
            telemetry.update();

        }
    }
}
