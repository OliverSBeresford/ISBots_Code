package org.firstinspires.ftc.teamcode;

import java.util.List;
import java.util.LinkedList;
import java.util.Queue;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
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
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.RotatedRect;
import android.util.Size;

public class RobotUtils {

    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public IMU imu = null;
    public CRServo intake = null;
    public Servo wrist = null;
    public DcMotorEx armMotor = null;
    public AprilTagProcessor aprilTagProcessor = null;
    public VisionPortal visionPortal = null;
    public ColorBlobLocatorProcessor blobProcessor = null;
    public ColorSensor colorSensor = null;
    
    public double imuCorrection = 0.0;
    
    // Values for detecting blocks
    public final int[] RED_RGB = {4000, 2000, 1200};
    public final int[] BLUE_RGB = {1000, 2200, 4500};
    public final int[] YELLOW_RGB = {6500, 8500, 2000};
    public final int TOLERANCE = 500;
    public final int colorThreshold = 500;
    public final ColorRange BLUE = ColorRange.BLUE;
    public final ColorRange RED = ColorRange.RED;
    public ColorRange color = null;

    // Vision constants
    public final int CAMERA_RESOLUTION_WIDTH =  640;
    public final int CAMERA_RESOLUTION_HEIGHT = 480;
    public final int CAMERA_FOV_HORIZONTAL = 60;

    // Constants for pathfinding
    public final double[] BLUE_BASKET = {72, 72, 25.75};
    public final double[] RED_BASKET = {-72, -72, 25.75};
    public final double[] RED_HIGH_CHAMBER = {0, -24, 26};
    public final double[] BLUE_HIGH_CHAMBER = {0, 24, 26};
    public final double[] RED_ASCENT = {-24, 0, 20};
    public final double[] BLUE_ASCENT = {24, 0, 20};
    public final double[] RED_OBSERVATION = {72, -72, 0};
    public final double[] BLUE_OBSERVATION = {-72, 72, 0};

    // Grid dimensions
    public final double MAP_SIZE = 144.0;
    private static final int GRID_SIZE = 12;
    private static final double CELL_SIZE = 12; // Inches

    // Define the 12x12 grid. 1 = obstacle, 0 = traversable
    private static final int[][] FIELD = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
        {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0},
        {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };


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
        imu.resetYaw();
    }

    public void setHardware(DcMotor _leftDrive, DcMotor _rightDrive, IMU _imu, CRServo _intake, Servo _wrist, DcMotorEx _armMotor) {
        leftDrive = _leftDrive;
        rightDrive = _rightDrive;
        imu = _imu;
        intake = _intake;
        wrist = _wrist;
        armMotor = _armMotor;
        imu.resetYaw();
    }

    public void setColorSensor(ColorSensor _colorSensor) {
        colorSensor = _colorSensor;
    }

    public void setColor(ColorRange _color) {
        color = _color;
    }
    /* *********************** End initialization functions *********************** */

    /* *********************** These functions relate to physical behaior of the robot *********************** */
    public void turnDegrees(LinearOpMode opMode, double turnAngle, boolean debugEnabled) {
        // Ensure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null) {
            return;
        }
    
        if (debugEnabled) {
            opMode.telemetry.addData("Turning", turnAngle);
            opMode.telemetry.update();
        }
    
        double currentAngle;
        double error = 0.0;
        double turnPower;
        double previousError;
        double kP = 1.0 / 150.0; // Adjusted proportional constant
        double minPower = 0.2; // Lower minimum power for fine adjustments
        double maxPower = 0.8;
    
        // Get starting angle and calculate target heading
        double startingAngle = getYawIMU();
        double targetHeading = startingAngle + turnAngle;
    
        // Normalize target heading to -180 to 180 range
        if (targetHeading > 180) {
            targetHeading -= 360;
        } else if (targetHeading < -180) {
            targetHeading += 360;
        }
    
        // Control loop for turning
        do {
            previousError = error;
            currentAngle = getYawIMU();
            error = targetHeading - currentAngle;

            if (error < -180) {
                error += 360;
            } else if (error > 180) {
                error -= 360;
            }

            if (isWithinRange(-previousError, error, 10)) {
                error = previousError;
            }
    
            // Calculate turn power with proportional control
            turnPower = kP * error;

            if (Math.abs(turnPower) > maxPower) {
                turnPower = Math.copySign(maxPower, turnPower);
            } else if (Math.abs(turnPower) < minPower) {
                turnPower = Math.copySign(minPower, turnPower);
            }
    
            // Apply motor power
            leftDrive.setPower(-turnPower);
            rightDrive.setPower(turnPower);
    
            // Debugging information
            if (debugEnabled) {
                opMode.telemetry.addData("Target Heading", targetHeading);
                opMode.telemetry.addData("Current Angle", currentAngle);
                opMode.telemetry.addData("Error", error);
                opMode.telemetry.addData("Turn Power", turnPower);
                opMode.telemetry.addData("Starting Angle", startingAngle);
                opMode.telemetry.addData("Previous Error", previousError);
                opMode.telemetry.addData("kP", kP);
                opMode.telemetry.update();
            }
        } while (Math.abs(error) > 5 && opMode.opModeIsActive()); // Slightly relaxed threshold
    
        // Stop motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    
        if (debugEnabled) {
            opMode.telemetry.addData("Turn Complete", "Final Angle: " + getYawIMU());
            opMode.telemetry.update();
        }
    }    

    public void driveStraight(LinearOpMode opMode, double distanceInInches, double power, double targetHeading, boolean debugEnabled) {
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

        if (debugEnabled) {
            opMode.telemetry.addData("Driving", distanceInInches);
            opMode.telemetry.update();
        }

        distanceInInches = correctDistance(distanceInInches);
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

        // leftDrive.setPower(power);
        // rightDrive.setPower(power);

        while (leftDrive.isBusy() && rightDrive.isBusy() && opMode.opModeIsActive()) {
            double currentHeading = getYawIMU();;
            double error = targetHeading - currentHeading;

            double correction = error * 0.001; // Proportional constant
            leftDrive.setPower(power + correction);
            rightDrive.setPower(power - correction);

            // Allow time for motors to respond
            opMode.sleep(10);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveArm(LinearOpMode opMode, int position) {
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
            opMode.sleep(10);
        }
    }

    public void navigateTo(LinearOpMode opMode, int armPosition, double[] origin, double[] destination, double currentHeading, boolean debugEnabled) {
        /* This function moves the robot to the blue basket
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         */

        // Making sure hardware is initialized
        if (leftDrive == null || rightDrive == null || imu == null || intake == null || wrist == null || armMotor == null || aprilTagProcessor == null || visionPortal == null) {
            print(opMode, "Hardware is null", debugEnabled);
            return;
        }
        setYawIMU(currentHeading);

        // Create variables to store the target coordinates
        double targetX = destination[0], 
            targetY = destination[1],
            targetZ = destination[2];

        moveArm(opMode, armPosition);
        print(opMode, "Moved arm. Proceeding", debugEnabled);
        opMode.sleep(50);

        Pose3D currentPose;
        currentPose = getData(opMode, aprilTagProcessor, debugEnabled);
        print(opMode, "Got data. Making grid", debugEnabled);
        opMode.sleep(50);

        // Drive to the blue basket
        // Convert field coordinates to grid indices
        int[] start = fieldToGrid(origin[0], origin[1]);

        // Making start coordinates where we are according to apriltags
        if (currentPose != null) {
            start = fieldToGrid(currentPose.getPosition().x, currentPose.getPosition().y);
            setYawIMU(currentPose.getOrientation().getYaw(AngleUnit.DEGREES));
        }

        // Target grid coordinates
        int[] target = fieldToGrid(targetX, targetY);

        print(opMode, "Calculated grid. Starting pathfinding", debugEnabled);

        // Give time for robot to respond
        opMode.sleep(50);

        // Perform bfs pathfinding
        print(opMode, "Going into bfs", debugEnabled);
        LinkedList<int[]> path = bfs(opMode, FIELD, start, target, debugEnabled);
        if (path == null) {
            print(opMode, "Path was null", debugEnabled);
            return;
        }
        if (debugEnabled) {
            opMode.telemetry.addData("Start", "Row: " + start[0] + ", Col: " + start[1]);
            printPath(opMode, path);
            opMode.sleep(1000);
        }

        // Give time for robot to respond
        opMode.sleep(10);

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
        int[] next;

        // Follow the path
        int[] previous = start;
        for (int[] current : path) {
            if (!opMode.opModeIsActive()) {
                return;
            }
            nextField = gridToField(previous[0], previous[1]);
            // Calculate direction and distance
            if (currentPose == null) {
                currentField = gridToField(current[0], current[1]);
            } else {
                currentField = new double[]{currentPose.getPosition().x, currentPose.getPosition().y};
                setYawIMU(currentPose.getOrientation().getYaw(AngleUnit.DEGREES));
            }

            // Calculate movement
            dx = currentField[0] - nextField[0];
            dy = currentField[1] - nextField[1];
            distance = Math.hypot(dx, dy);
            targetHeading = Math.toDegrees(Math.atan2(-dx, dy));
            print(opMode, "Target heading (in calculating): " + targetHeading, debugEnabled);
            print(opMode, "Dx: " + dx + ", Dy: " + dy + "\nTarget y: " + previous[0] + ", target x: " + previous[1], debugEnabled);

            // Turn and move
            print(opMode, "Turning to correct heading", debugEnabled);
            turnToHeading(opMode, imu, targetHeading, debugEnabled);

            print(opMode, "Driving straight", debugEnabled);
            opMode.sleep(10);
            driveStraight(opMode, distance, 0.5, targetHeading, debugEnabled);
            opMode.sleep(10);
            print(opMode, "Drove straight", debugEnabled);

            // Periodically correct using AprilTag
            currentPose = getData(opMode, aprilTagProcessor, debugEnabled);
            if (currentPose != null && debugEnabled) {
                opMode.telemetry.addData("Position", currentPose.getPosition());
                opMode.telemetry.addData("Heading", currentPose.getOrientation());
                opMode.telemetry.update();
            }
            
            previous = current;
        }
    }

    public void printPath(LinearOpMode opMode, LinkedList<int[]> path) {
        opMode.telemetry.addLine("Path: ");
        for (int[] position : path) {
            opMode.telemetry.addLine("(Row: " + position[0] + ", Col: " + position[1] + ")");
        }
        opMode.telemetry.update();
    }

    public void turnTowardsBlob(LinearOpMode opMode, boolean debugEnabled) {
        if (imu == null || blobProcessor == null) {
            print(opMode, "Turning towards blob encountered error", debugEnabled);
            return;
        }
    
        // Get blob information
        List<ColorBlobLocatorProcessor.Blob> blobs = blobProcessor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);
    
        if (blobs.isEmpty()) {
            opMode.telemetry.addData("Blob Status", "No blob detected");
            opMode.telemetry.update();
            return; // No blob to turn towards
        }
    
        // Use the first detected blob
        ColorBlobLocatorProcessor.Blob blob = blobs.get(0);
        RotatedRect boxFit = blob.getBoxFit();
        double blobX = boxFit.center.x;
    
        // Calculate angle to blob
        double angleToBlob = ((blobX - CAMERA_RESOLUTION_WIDTH / 2.0) / CAMERA_RESOLUTION_WIDTH) * CAMERA_FOV_HORIZONTAL;
    
        // Get current heading and compute target heading
        turnDegrees(opMode, -angleToBlob, debugEnabled);
    
        if (debugEnabled) {
            opMode.telemetry.addData("Turning Complete", "Aligned with Blob");
            opMode.telemetry.update();
        }
    }
    

    public void turnToHeading(LinearOpMode opMode, IMU imu, double targetHeading, boolean debugEnabled) {
        double currentHeading = getYawIMU();
        double turnAngle = targetHeading - currentHeading;

        if (turnAngle > 180) turnAngle -= 360;
        if (turnAngle < -180) turnAngle += 360;

        turnDegrees(opMode, turnAngle, debugEnabled);
    }

    public void setYawIMU(double yaw) {
        if (imu != null) {
            imuCorrection = yaw - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
    }

    public double getYawIMU() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuCorrection;
        }
        return 0;
    }

    /* *********************** End robot physical behavior functions *********************** */

    /* *********************** These functions relate to pathfinding *********************** */
    private int[] fieldToGrid(double x, double y) {
        // Normalize x and y to grid space
        int col = (int) Math.floor(((x + MAP_SIZE / 2) / CELL_SIZE) - (x == -(MAP_SIZE / 2) ? 0 : 1e-9));
        int row = (int) Math.floor(((MAP_SIZE / 2 - y) / CELL_SIZE) - (y == (MAP_SIZE / 2) ? 0 : 1e-9)); // Y decreases downward
        return new int[]{row, col};
    }     

    private double[] gridToField(int row, int col) {
        double x = col * CELL_SIZE - MAP_SIZE / 2 + CELL_SIZE / 2; // Add CELL_SIZE/2 for cell center
        double y = MAP_SIZE / 2 - row * CELL_SIZE - CELL_SIZE / 2; // Subtract CELL_SIZE/2 for cell center
        return new double[]{x, y};
    }    

    private LinkedList<int[]> bfs(LinearOpMode opMode, int[][] grid, int[] start, int[] target, boolean debugEnabled) {
        if (!isValid(start[0], start[1], grid, new boolean[GRID_SIZE][GRID_SIZE]) || !isValid(target[0], target[1], grid, new boolean[GRID_SIZE][GRID_SIZE])) {
            print(opMode, "Invalid\nStartY: " + start[0] + "\nStartX: " + start[1] + "\nTargetY: " + target[0] + "\nTargetX: " + target[1], debugEnabled);
            return null;
        }


        int[][] directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        boolean[][] visited = new boolean[GRID_SIZE][GRID_SIZE];
        Queue<int[]> queue = new LinkedList<>();
        int[][] parent = new int[GRID_SIZE][GRID_SIZE]; // Track paths

        queue.add(start);
        visited[start[0]][start[1]] = true;

        while (!queue.isEmpty()) {
            int[] current = queue.poll();
            if (current[0] == target[0] && current[1] == target[1]) {
                return reconstructPath(parent, start, target);
            }

            for (int[] dir : directions) {
                int newRow = current[0] + dir[0];
                int newCol = current[1] + dir[1];
                if (isValid(newRow, newCol, grid, visited)) {
                    queue.add(new int[]{newRow, newCol});
                    visited[newRow][newCol] = true;
                    parent[newRow][newCol] = current[0] * GRID_SIZE + current[1];
                }
            }
        }

        return null; // No path found
    }

    private LinkedList<int[]> reconstructPath(int[][] parent, int[] start, int[] target) {
        LinkedList<int[]> path = new LinkedList<>();
        int current = target[0] * GRID_SIZE + target[1];
        while (current != start[0] * GRID_SIZE + start[1]) {
            int row = current / GRID_SIZE;
            int col = current % GRID_SIZE;
            path.addFirst(new int[]{row, col});
            current = parent[row][col];
        }
        return path;
    }

    private boolean isValid(int row, int col, int[][] grid, boolean[][] visited) {
        return row >= 0 && row < GRID_SIZE && col >= 0 && col < GRID_SIZE && grid[row][col] == 0 && !visited[row][col];
    }    

    /* *********************** End pathfinding functions *********************** */
    

    /* *********************** These functions relate to the Vision system. *********************** */
    public void initAprilTag(LinearOpMode opMode) {
        // Camera position and orientation
        Position cameraPosition = new Position(DistanceUnit.CM, -10, 0, 7, 0);
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
    }

    public Pose3D getData(LinearOpMode opMode, AprilTagProcessor myAprilTagProcessor, boolean debugEnabled) {
        /* This function returns pose data (i.e. the robot's position on the field) using an AprilTag Detection 
         * 
         * Parameters: LinearOpMode opMode - The LinearOpMode object that is used to run the robot.
         *             AprilTagProcessor myAprilTagProcessor - The AprilTagProcessor object that is used to detect AprilTags.
         *             bool debugEnabled - Whether or not to print debug information.
        */
        print(opMode, "Getting data.", debugEnabled);

        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        // Display the number of AprilTags detected.
        if (debugEnabled) {
            opMode.telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
            if (JavaUtil.listLength(myAprilTagDetections) == 0) {
                opMode.telemetry.update();
            }
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

                if (imu != null) {
                    opMode.telemetry.addData("IMU yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                }

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

    public void initCamera(LinearOpMode opMode) {
        if (color == null) {
            color = RED;
        }

        // Camera position and orientation
        Position cameraPosition = new Position(DistanceUnit.CM, -10, 0, 7, 0);
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
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    
        // Build a "Color Locator" vision processor based on the ColorBlobLocatorProcessor class.
        ColorBlobLocatorProcessor.Builder myColorBlobLocatorProcessorBuilder = new ColorBlobLocatorProcessor.Builder();
        // - Specify the color range you are looking for.
        myColorBlobLocatorProcessorBuilder.setTargetColorRange(color);
        // 50% width/height square centered on screen
        myColorBlobLocatorProcessorBuilder.setRoi(ImageRegion.asUnityCenterCoordinates(-1.0, 0.0, 1.0, -1.0));
        // - Define which contours are included.
        myColorBlobLocatorProcessorBuilder.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY);
        // - Turn the display of contours ON or OFF.  Turning this on helps debugging but takes up valuable CPU time.
        myColorBlobLocatorProcessorBuilder.setDrawContours(true);
        // - Include any pre-processing of the image or mask before looking for Blobs.

        ColorBlobLocatorProcessor myColorBlobLocatorProcessor = myColorBlobLocatorProcessorBuilder.build();

        //  - Add the ColorBlobLocatorProcessor created above.
        myVisionPortalBuilder.addProcessor(myColorBlobLocatorProcessor);
        //  - Set the desired video resolution.
        myVisionPortalBuilder.setCameraResolution(new Size(CAMERA_RESOLUTION_WIDTH, CAMERA_RESOLUTION_HEIGHT));
        //  - Choose your video source. This may be for a webcam or for a Phone Camera.
        myVisionPortalBuilder.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Assign the AprilTagProcessor and VisionPortal to the class variables.
        visionPortal = myVisionPortalBuilder.build();
        aprilTagProcessor = myAprilTagProcessor;
        blobProcessor = myColorBlobLocatorProcessor;
    }
    /* *********************** End Vision functions *********************** */
    
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
    
    public boolean isAnyColorBlock() {
        return isRedBlock() || isBlueBlock() || isYellowBlock();
    }
    
    private boolean isWithinRange(int actual, int target, int tolerance) {
        return Math.abs(actual - target) <= tolerance;
    }

    private boolean isWithinRange(double actual, double target, double tolerance) {
        return Math.abs(actual - target) <= tolerance;
    }
    /* *********************** End color sensing functions *********************** */

    /* *********************** Telemetry helper functions *********************** */
    private void print(LinearOpMode opMode, String message, boolean debugEnabled) {
        if (!debugEnabled) {
            return;
        }
        opMode.telemetry.addLine(message);
        opMode.telemetry.addLine("Heading: " + getYawIMU());

        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (armMotor != null && ((DcMotorEx) armMotor).isOverCurrent()){
            opMode.telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }

        opMode.telemetry.update();
    }
    /* *********************** End of telemetry helper functions *********************** */

    /* *********************** Math *********************** */
    private double correctDistance(double distance) {
        if (distance < 0) {
            return 1.23896 * distance + 1.8399;
        }
        return 1.24593 * distance - 1.05246;
    }

    /* *********************** End math functions *********************** */
    
    /* *********************** Classes used to return specific data from the RobotUtils class *********************** */
    public static class VisionComponents {
        VisionPortal visionPortal;
        AprilTagProcessor aprilTagProcessor;
        ColorBlobLocatorProcessor colorBlobProcessor;

        public VisionComponents(VisionPortal vision_portal, AprilTagProcessor apriltag_processor) {
            visionPortal = vision_portal;
            aprilTagProcessor = apriltag_processor; 
        }

        public void setBlobProcessor(ColorBlobLocatorProcessor colorBlobLocatorProcessor) {
            colorBlobProcessor = colorBlobLocatorProcessor;
        }
    }
    /* *********************** End classes used to return specific data from the RobotUtils class *********************** */
}
