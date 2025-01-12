package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RobotUtils {

    public static void turnDegrees(DcMotor leftDrive, DcMotor rightDrive, IMU imu, double turnAngle) {
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
            // Get the current angle and calculate error
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
            // Ensure the absolute value of the power is no greater than 1
            if (Math.abs(turnPower) > 1) {
                turnPower = Math.copySign(1, turnPower);
            }

            // Apply motor power for turning
            leftDrive.setPower(turnPower);
            rightDrive.setPower(-turnPower);

            // Allow time for the motors to respond
            Thread.sleep(10);
        } while (Math.abs(error) > 1); // Stop turning when the error is small

        // Stop the motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
