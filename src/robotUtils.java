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

    public void driveStraight(DcMotor leftDrive, DcMotor rightDrive, IMU imu, double distanceInInches, double power, double targetHeading) {
        int ticksPerRevolution = ((((1+(46/17))) * (1+(46/11))) * 28); // Example for a typical motor
        double wheelDiameter = 3.77953; // In inches
        double wheelCircumference = Math.PI * wheelDiameter;
        int targetTicks = (int)((distanceInInches / wheelCircumference) * ticksPerRevolution);
    
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);
    
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);
        
        // Keep adjusting the heading while we aren't there yet
        while (leftDrive.isBusy() && rightDrive.isBusy()) {
            // Check the current heading
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double error = targetHeading - currentHeading;
    
            // Adjust motor power to correct heading
            double correction = error * 0.02; // Proportional constant
            leftDrive.setPower(power + correction);
            rightDrive.setPower(power - correction);
        }
    
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Initialize AprilTag Detection.
     */
    private void initAprilTag(AprilTagProcessor myAprilTagProcessor, Position cameraPosition, YawPitchRollAngles cameraOrientation, VisionPortal myVisionPortal) {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);

        // Create an AprilTagProcessor by calling build.
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

}
