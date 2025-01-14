package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RobotUtils {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private IMU imu;

    // Constructor to initialize hardware
    public RobotUtils(DcMotor _leftDrive, DcMotor _rightDrive, IMU _imu) {
        leftDrive = _leftDrive;
        rightDrive = _rightDrive;
        imu = _imu;
    }

    // Method to turn the robot by a specific angle
    public void turnDegrees(LinearOpMode opMode, double turnAngle) {
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

    // Method to drive straight for a specific distance
    public void driveStraight(LinearOpMode opMode, double distanceInInches, double power, double targetHeading) {
        int ticksPerRevolution = ((((1 + (46 / 17))) * (1 + (46 / 11))) * 28); // Example for a typical motor
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
}
