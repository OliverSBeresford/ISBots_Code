package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.HashSet;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Set;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class RobotUtils {

    private static DcMotor leftDrive = null;
    private static DcMotor rightDrive = null;
    private static IMU imu = null;
    private static CRServo intake = null;
    private static Servo wrist = null;
    private static DcMotorEx armMotor = null;
    private static AprilTagProcessor aprilTagProcessor = null;
    private static VisionPortal visionPortal = null;
    private static ColorSensor colorSensor = null;
    
    // Values for detecting blocks
    private static final int[] RED_RGB = {4000, 2000, 1200};
    private static final int[] BLUE_RGB = {1000, 2200, 4500};
    private static final int[] YELLOW_RGB = {6500, 8500, 2000};
    private static final int TOLERANCE = 500;

    // Define the 6x6 grid. 1 = obstacle, 0 = traversable
    private static final int[][] FIELD = {
        {0, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 1},
        {0, 0, 1, 1, 0, 0},
        {0, 0, 1, 1, 0, 0},
        {1, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0}
    };

    // Grid dimensions
    private static final int GRID_SIZE = 6;
    private static final double CELL_SIZE = 24.0; // Inches

    /* These are the various functions to initialize whatever parts of the RobotUtils class you need
     * 
     * setHardware: Initializes the hardware for the robot.
     * Parameters: DcMotor _leftDrive - The left drive motor.
     *             DcMotor _rightDrive - The right drive motor.
     *             IMU _imu - The IMU sensor.
     *             CRServo _intake - The intake servo.
     *             Servo _wrist - The wrist servo.
     *             DCMotorEx _armMotor - The arm motor.
     */
    public void setHardware(DcMotor _leftDrive, DcMotor _rightDrive, IMU _imu) {
        leftDrive = _leftDrive;
        rightDrive = _rightDrive;
        imu = _imu;
    }

    public void setHardware(DcMotor _leftDrive, DcMotor _rightDrive, IMU _imu, CRServo _intake, Servo _wrist, DcMotorEx _armMotor) {
        leftDrive = _leftDrive;
        rightDrive = _rightDrive;
        imu = _imu;
        intake = _intake;
        wrist = _wrist;
        armMotor = _armMotor;
    }

    public void setColorSensor(ColorSensor _colorSensor) {
        colorSensor = _colorSensor;
    }
    /* *********************** End initialization functions *********************** */

    /* *********************** These functions relate to physical behaior of the robot *********************** */
    public static void turnDegrees(LinearOpMode opMode, double turnAngle) {
        /* This function turn the robot a certain number of degrees
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         *             double turnAngle - The number of degrees to turn the robot.
         */

        // Making sure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null) {
            return;
        }

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

    public void driveStraight(LinearOpMode opMode, double distanceInInches, double power, double targetHeading) {
        /* This function drives the robot a certain distance in inches
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         *             double distanceInInches - The distance to drive the robot in inches.
         *             double power - The power to drive the robot at (number from 0 to 1).
         *             double targetHeading - The heading to drive the robot at.
         */

        // Making sure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null) {
            return;
        }

        int ticksPerRevolution = ((((1 + (46 / 17))) * (1 + (46 / 11))) * 28); // Got this from GOBilda
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

    private void moveArm(LinearOpMode opMode, int position) {
        /* This function moves the arm to a specific position
         * Parameters: int position - The position to move the arm to.
         */
        
        // Making sure hardware is initialized
        if (armMotor == null) {
            return;
        }

        armMotor.setTargetPosition(position); // Adjust based on arm's vertical position for baskets
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setVelocity(2100);
        while (armMotor.isBusy() && opMode.opModeIsActive()) {
            // Wait for the arm to reach the position
        }
    }

    public void moveToBlueBasket(LinearOpMode opMode, int armPosition, boolean debugEnabled) {
        /* This function moves the robot to the blue basket
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         */

        // Making sure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null || intake == null || wrist == null || armMotor == null || aprilTagProcessor == null || visionPortal == null) {
            return;
        }

        moveArm(opMode, armPosition);

        if (debugEnabled) {
            opMode.telemetry.addLine("Moved arm. Proceeding.");
            opMode.telemetry.update();
        }

        Pose3D currentPose;

        currentPose = getData(opMode, aprilTagProcessor, true);

        if (debugEnabled) {
            opMode.telemetry.addLine("Got data. Making grid.");
            opMode.telemetry.update();
        }

        // Drive to the blue basket
        // Convert field coordinates to grid indices
        int[] start;
        if (currentPose == null) {
            start = new int[]{0, 0};
        } else {
            start = fieldToGrid(currentPose.getPosition().x, currentPose.getPosition().y);
        }
        int[] target = fieldToGrid(60, 60);

        if (debugEnabled) {
            opMode.telemetry.addLine("Calculated grid. Starting pathfinding.");
            opMode.telemetry.update();
        }

        // Perform A* pathfinding
        List<int[]> path = aStar(opMode, FIELD, start, target);

        if (path == null) {
            opMode.telemetry.addData("Error", "No path found.");
            opMode.telemetry.update();
            return;
        }

        // Local variables for path following
        double[] currentField = {0, 0};
        double[] nextField = {0, 0};
        double dx;
        double dy;
        double distance;
        double targetHeading;
        int[] current;
        int[] next;

        // Follow the path
        for (int i = 1; i < path.size(); i++) {
            intake.setPower(0);

            current = path.get(i - 1);
            next = path.get(i);

            // Calculate direction and distance
            if (currentPose == null) {
                currentField = gridToField(current[0], current[1]);
            } else {
                currentField = new double[]{currentPose.getPosition().x, currentPose.getPosition().y};
            }
            nextField = gridToField(next[0], next[1]);

            dx = nextField[0] - currentField[0];
            dy = nextField[1] - currentField[1];
            distance = Math.hypot(dx, dy);
            targetHeading = Math.toDegrees(Math.atan2(dy, dx));

            // Turn and move
            turnToHeading(opMode, imu, targetHeading);
            driveStraight(opMode, distance, 0.5, targetHeading);

            // Correct position periodically
            currentPose = getData(opMode, aprilTagProcessor, false);
            opMode.telemetry.addData("Position", currentPose.getPosition());
            opMode.telemetry.addData("Heading", currentPose.getOrientation());
            opMode.telemetry.update();
        }
    }

    private static void turnToHeading(LinearOpMode opMode, IMU imu, double targetHeading) {
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double turnAngle = targetHeading - currentHeading;

        if (turnAngle > 180) turnAngle -= 360;
        if (turnAngle < -180) turnAngle += 360;

        turnDegrees(opMode, turnAngle);
    }
    /* *********************** End robot physical behavior functions *********************** */

    /* *********************** These functions relate to pathfinding *********************** */
    private static int[] fieldToGrid(double x, double y) {
        // Assuming the bottom left corner corresponds to the coordinates (-72, -72)
        int col = (int) Math.floor((x + 72) / CELL_SIZE);
        int row = (int) Math.floor((144 - (y + 72)) / CELL_SIZE);
        return new int[]{row, col};
    }

    private static double[] gridToField(int row, int col) {
        double x = (col - 3) * CELL_SIZE + CELL_SIZE / 2;
        double y = (3 - row) * CELL_SIZE - CELL_SIZE / 2;
        return new double[]{x, y};
    }

    private static List<int[]> aStar(LinearOpMode opMode, int[][] grid, int[] start, int[] goal) {
        int iterations = 0;

        PriorityQueue<Node2> openSet = new PriorityQueue<>(Comparator.comparingDouble(n -> n.fCost));
        Set<String> closedSet = new HashSet<>();
        openSet.add(new Node2(start, null, 0, heuristic(start, goal)));

        while (!openSet.isEmpty()) {
            iterations += 1;

            opMode.telemetry.addLine("Iteration: " + iterations);
            opMode.telemetry.update();

            Node2 current = openSet.poll();
            int[] pos = current.position;

            if (Arrays.equals(pos, goal)) return reconstructPath(current);

            closedSet.add(Arrays.toString(pos));

            for (int[] dir : new int[][]{{0, 1}, {1, 0}, {0, -1}, {-1, 0}}) {
                int[] neighbor = {pos[0] + dir[0], pos[1] + dir[1]};

                if (isValid(neighbor, grid) && !closedSet.contains(Arrays.toString(neighbor))) {
                    double gCost = current.gCost + 1;
                    double hCost = heuristic(neighbor, goal);
                    openSet.add(new Node2(neighbor, current, gCost, gCost + hCost));
                }
            }
        }

        return null; // No path found
    }

    private static boolean isValid(int[] pos, int[][] grid) {
        int row = pos[0], col = pos[1];
        return row >= 0 && row < GRID_SIZE && col >= 0 && col < GRID_SIZE && grid[row][col] == 0;
    }

    private static double heuristic(int[] a, int[] b) {
        return Math.abs(a[0] - b[0]) + Math.abs(a[1] - b[1]);
    }

    private static List<int[]> reconstructPath(Node2 node) {
        List<int[]> path = new ArrayList<>();
        while (node != null) {
            path.add(node.position);
            node = node.parent;
        }
        Collections.reverse(path);
        return path;
    }
    /* *********************** End pathfinding functions *********************** */
    

    /* *********************** These functions relate to the AprilTag detection system. *********************** */
    public VisionComponents initAprilTag(LinearOpMode opMode) {
        // Variables to store the position and orientation of the camera on the robot. Setting these
        // values requires a definition of the axes of the camera and robot:
        // Camera axes:
        // Origin location: Center of the lens
        // Axes orientation: +x right, +y down, +z forward (from camera's perspective)
        // Robot axes (this is typical, but you can define this however you want):
        // Origin location: Center of the robot at field height
        // Axes orientation: +x right, +y forward, +z upward
        // Position:
        // If all values are zero (no translation), that implies the camera is at the center of the
        // robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
        // inches above the ground - you would need to set the position to (-5, 7, 12).
        // Orientation:
        // If all values are zero (no rotation), that implies the camera is pointing straight up. In
        // most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
        // the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
        // it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
        // to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
        // Position(DistanceUnit unit, double x, double y, double z, long acquisitionTime)
        Position cameraPosition = new Position(DistanceUnit.CM, 5, 21.5, 18, 0);
        // YawPitchRollAngles(AngleUnit angleUnit, double yaw, double pitch, double roll, long acquisitionTime)
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        
        // Create an AprilTagProcessor by calling build.
        // Create the AprilTag processor and assign it to a variable.
        AprilTagProcessor myAprilTagProcessor = myAprilTagProcessorBuilder.build();

        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        VisionPortal myVisionPortal = myVisionPortalBuilder.build();

        // Assign the AprilTagProcessor and VisionPortal to the class variables.
        aprilTagProcessor = myAprilTagProcessor;
        visionPortal = myVisionPortal;
        
        return new VisionComponents(myVisionPortal, myAprilTagProcessor);
    }

    public Pose3D getData(LinearOpMode opMode, AprilTagProcessor myAprilTagProcessor, boolean debugEnabled) {
        /* This function returns pose data (i.e. the robot's position on the field) using an AprilTag Detection 
         * 
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         *             AprilTagProcessor myAprilTagProcessor - The AprilTagProcessor object that is used to detect AprilTags.
         *             bool debugEnabled - Whether or not to print debug information.
        */

        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        // Display the number of AprilTags detected.
        if (debugEnabled) {
            opMode.telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        }

        // Iterate through list and call a function to return info for the first recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            if (myAprilTagDetection.metadata != null) {
                // Return the data without printing it if debug is disabled
                if (!debugEnabled) {
                    return myAprilTagDetection.robotPose;
                }

                // Field-relative data
                opMode.telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                opMode.telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "    (inch)");
                opMode.telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getPitch(), 6, 1) + "" + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getRoll(), 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getOrientation().getYaw(), 6, 1) + "    (deg)");
                
                // Useful information for debugging
                opMode.telemetry.addLine("");
                opMode.telemetry.addLine("key:");
                opMode.telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
                opMode.telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

                opMode.telemetry.update();

                return myAprilTagDetection.robotPose;
            } else {
                opMode.telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                opMode.telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
                opMode.telemetry.update();
                return null;
            }
        }
        return null;
    }
    /* *********************** End AprilTag detection functions *********************** */
    
    /* *********************** Color sensing functions *********************** */
    private boolean isBlockPresent() {
        if (colorSensor == null) {
            return false;
        }
        return (colorSensor.red() > colorThreshold || colorSensor.green() > colorThreshold || colorSensor.blue() > colorThreshold);
    }

    public boolean isRedBlock() {
        return isBlockPresent() &&
               isWithinRange(colorSensor.red(), RED_RGB[0], TOLERANCE) &&
               isWithinRange(colorSensor.green(), RED_RGB[1], TOLERANCE) &&
               isWithinRange(colorSensor.blue(), RED_RGB[2], TOLERANCE);
    }
    
    public boolean isBlueBlock() {
        return isBlockPresent() &&
               isWithinRange(colorSensor.red(), BLUE_RGB[0], TOLERANCE) &&
               isWithinRange(colorSensor.green(), BLUE_RGB[1], TOLERANCE) &&
               isWithinRange(colorSensor.blue(), BLUE_RGB[2], TOLERANCE);
    }
    
    public boolean isYellowBlock() {
        return isBlockPresent() &&
               isWithinRange(colorSensor.red(), YELLOW_RGB[0], TOLERANCE) &&
               isWithinRange(colorSensor.green(), YELLOW_RGB[1], TOLERANCE) &&
               isWithinRange(colorSensor.blue(), YELLOW_RGB[2], TOLERANCE);
    }
    /* *********************** End color sensing functions *********************** */

    public boolean isAnyColorBlock() {
        return isRedBlock() || isBlueBlock() || isYellowBlock();
    }
    
    private boolean isWithinRange(int actual, int target, int tolerance) {
        return Math.abs(actual - target) <= tolerance;
    }
    
    /* *********************** Classes used to return specific data from the RobotUtils class *********************** */
    public static class VisionComponents {
        VisionPortal visionPortal;
        AprilTagProcessor aprilTagProcessor;

        public VisionComponents(VisionPortal vision_portal, AprilTagProcessor apriltag_processor) {
            visionPortal = vision_portal;
            aprilTagProcessor = apriltag_processor; 
        }
    }
    private static class Node2 {
        int[] position;
        Node2 parent;
        double gCost, fCost;

        Node2(int[] position, Node2 parent, double gCost, double fCost) {
            this.position = position;
            this.parent = parent;
            this.gCost = gCost;
            this.fCost = fCost;
        }
    }
    /* *********************** End classes used to return specific data from the RobotUtils class *********************** */
}
