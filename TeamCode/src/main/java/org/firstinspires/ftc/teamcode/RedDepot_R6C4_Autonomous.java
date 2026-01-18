package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous program for Red Depot starting at R6,C4
 * Robot collects and shoots balls in two cycles
 * VERSION: V0
 */
@Autonomous(name="RedDepot_R6C4_Autonomous_V0", group="Competition")
public class RedDepot_R6C4_Autonomous extends LinearOpMode {

    // Hardware components
    private DcMotor frontRight, backRight, frontLeft, backLeft;
    private DcMotor ejectionMotor, intakeMotor;
    private CRServo feederServoRight, feederServoLeft;

    private ElapsedTime runtime = new ElapsedTime();

    // ===== DRIVE MOVEMENT CONSTANTS =====
    // Adjust these to change how the robot moves
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.7;                      // Normal driving speed (0.0 to 1.0)
    private static final double PRELOAD_INTAKE_DRIVE_SPEED = 0.3;       // Very slow for pre-loaded balls
    private static final double TURN_SPEED = 0.3;                       // Rotation speed (slower = smoother)
    private static final double TILE_SIZE = 24.0;                       // FTC field tile size in inches
    private static final double INITIAL_BACKUP = 3.5;                   // Initial backward movement
    private static final double LONG_MOVE_THRESHOLD = 48.0;             // 2 tiles - threshold for extended deceleration

    // ===== DECELERATION SETTINGS =====
    // Controls how smoothly the robot stops
    private static final double SHORT_DECEL_DISTANCE = 3.0;     // Deceleration for moves < 2 tiles
    private static final double LONG_DECEL_DISTANCE = 8.0;      // Extended deceleration for moves ≥ 2 tiles
    private static final double MIN_SPEED = 0.2;                // Minimum speed during deceleration

    // ===== ROTATION CALIBRATION =====
    // Adjust ROTATION_CALIBRATION if robot over/under rotates
    private static final double ROTATION_CALIBRATION = 0.95;
    private static final double ROBOT_WIDTH = 14.0;
    private static final double DEGREES_TO_INCHES = (ROBOT_WIDTH * 3.1415 * ROTATION_CALIBRATION) / 180.0;

    // ===== BALL HANDLING POWER LEVELS =====
    // Adjust these to control ejection/intake speeds (0.0 to 1.0+)
    private static final double EJECTION_POWER = 0.825;           // Ejection motor speed (reduced from 0.7)
    private static final double INTAKE_POWER = 0.7;             // Intake motor speed (reduced from 1.1)
    private static final double FEEDER_POWER = 0.6;             // Feeder servo speed (reduced from 1.1)

    // ===== TIMING CONSTANTS =====
    // Adjust these to optimize ball handling timing
    private static final int PRELOAD_MOTOR_WARMUP_MS = 2000;    // Ejection motor warmup before feeders
    private static final int PRELOAD_FEEDER_RUN_MS = 5000;      // Feeders run time after warmup
    private static final int EJECTION_WARMUP_MS = 1500;         // Time for ejection motor to reach speed
    private static final int FIRST_BALL_EJECT_MS = 2500;        // Time to eject first ball
    private static final int SECOND_BALL_EJECT_MS = 2500;       // Time to eject second ball
    private static final int FIRST_BALL_INTAKE_MS = 2000;       // Time to intake first ball
    private static final int BALL_SETTLE_MS = 500;              // Pause to let balls settle

    @Override
    public void runOpMode() {
        // Initialize all hardware
        initializeHardware();

        telemetry.addData("Status", "Ready for Red Depot Autonomous V0");
        telemetry.addData("Start Position", "R6, C4 facing top");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // ===== STEP 1-2: Move backward 3.5 inches =====
            telemetry.addData("Step", "1-2: Move backward 3.5 inches");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, -INITIAL_BACKUP, -INITIAL_BACKUP, -INITIAL_BACKUP, -INITIAL_BACKUP, 5.0);

            // ===== FIRST CYCLE: Shoot pre-loaded balls =====
            telemetry.addData("Cycle", "1: Shooting pre-loaded balls");
            telemetry.update();

            // Steps 3-8: Rotate and shoot with staggered motor start
            shootPreloadedBalls();

            // ===== NAVIGATE TO INTAKE POSITION =====
            telemetry.addData("Navigation", "Moving to R5,C5 for intake");
            telemetry.update();

            // Steps 9-10: Navigate to R5,C4
            navigateToPreIntakePosition();

            // ===== SECOND CYCLE: Collect ball from R5,C5 =====
            telemetry.addData("Cycle", "2: Collecting from R5,C5");
            telemetry.update();

            // Steps 11-15: Start intake THEN move to collect
            intakeBallsWithMotorFirst();

            // Steps 16-20: Navigate back to shooting position
            //returnToShootingPosition();

            // Steps 21-28: Shoot collected balls
            //ejectBalls();

            // ===== FINAL POSITIONING =====
            telemetry.addData("Action", "Moving to final position");
            telemetry.update();

            // Step 29: Rotate 45° CW from current position
            //rotate(TURN_SPEED, 45, 5.0);

            // Step 30: Move forward 1 tile (24 inches)
            //encoderDrive(DRIVE_SPEED, TILE_SIZE, TILE_SIZE, TILE_SIZE, TILE_SIZE, 5.0);

            telemetry.addData("Status", "AUTONOMOUS COMPLETE");
            telemetry.update();
        }
    }

    /**
     * Initialize all motors and servos with correct directions
     * Only needs to be modified if hardware connections change
     */
    private void initializeHardware() {
        // Drive motors
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ball handling motors and servos
        ejectionMotor = hardwareMap.get(DcMotor.class, "ejection_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        feederServoRight = hardwareMap.get(CRServo.class, "feeder_servo_right");
        feederServoLeft = hardwareMap.get(CRServo.class, "feeder_servo_left");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        ejectionMotor.setDirection(DcMotor.Direction.FORWARD);
        feederServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        feederServoRight.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
    }

    /**
     * STEPS 3-8: Shoot pre-loaded balls
     * Step 3: Rotate 20° CW while starting ejection motors
     * Step 4: Wait 2s for motor warmup
     * Step 5: Start feeders
     * Step 6: Wait 5s (ejecting)
     * Step 7: Stop all
     * Step 8: Rotate 160° CW to position intake forward
     */
    private void shootPreloadedBalls() {
        telemetry.addData("Action", "Step 3: Rotate 20° CW + start motors");
        telemetry.update();

        // Step 3: Start ejection motors DURING rotation
        ejectionMotor.setPower(EJECTION_POWER);
        intakeMotor.setPower(INTAKE_POWER);
        rotate(TURN_SPEED, 20, 5.0);

        // Step 4: Wait for motors to reach speed
        telemetry.addData("Action", "Step 4: Motor warmup");
        telemetry.update();
        sleep(PRELOAD_MOTOR_WARMUP_MS);

        // Step 5-6: Start feeders and run
        telemetry.addData("Action", "Step 5-6: Feeders ejecting");
        telemetry.update();
        feederServoLeft.setPower(FEEDER_POWER);
        feederServoRight.setPower(FEEDER_POWER);
        sleep(PRELOAD_FEEDER_RUN_MS);

        // Step 7: Stop all
        stopBallHandling();

        // Step 8: Rotate 160° CW to position intake forward
        telemetry.addData("Action", "Step 8: Rotate 160° CW");
        telemetry.update();
        rotate(TURN_SPEED, 160, 10.0);
    }

    /**
     * STEPS 9-10: Navigate to pre-intake position R5,C4
     * Step 9: Move forward 24" to R5,C4
     * Step 10: Rotate 90° CW to face north
     */
    private void navigateToPreIntakePosition() {
        // Step 9: Move forward 1 tile to R5,C4 center
        telemetry.addData("Action", "Step 9: Move to R5,C4");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, TILE_SIZE, TILE_SIZE, TILE_SIZE, TILE_SIZE, 5.0);

        // Step 10: Rotate 90° CW to face north
        telemetry.addData("Action", "Step 10: Rotate 90° CW");
        telemetry.update();
        rotate(TURN_SPEED, 90, 5.0);
    }

    /**
     * STEPS 11-15: Intake balls - START MOTOR FIRST, THEN MOVE
     * Step 11: Start intake motor (BEFORE moving)
     * Step 12: Move forward 24" to R5,C5 (intake already running)
     * Step 13: Wait 2s (first ball intake complete)
     * Step 14: Pause 500ms (ball settles)
     * Step 15: Stop intake motor
     *
     * Simplified to collect only one ball
     */
    private void intakeBallsWithMotorFirst() {
        // Step 11: Start intake system FIRST (before moving)
        telemetry.addData("Action", "Step 11: Start intake motor");
        telemetry.update();
        intakeMotor.setPower(INTAKE_POWER);
        sleep(300); // Brief pause to let motor spin up

        // Step 12: NOW move forward to R5,C5 (intake already running)
        telemetry.addData("Action", "Step 12: Move to R5,C5 (intake running)");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, TILE_SIZE, TILE_SIZE, TILE_SIZE, TILE_SIZE, 5.0);

        // Step 13: Wait for first ball intake
        telemetry.addData("Action", "Step 13: Wait for ball");
        telemetry.update();
        sleep(FIRST_BALL_INTAKE_MS);

        // Step 14: Pause to let ball settle
        telemetry.addData("Action", "Step 14: Ball settle");
        telemetry.update();
        sleep(BALL_SETTLE_MS);

        // Step 15: Stop intake
        intakeMotor.setPower(0);
        telemetry.addData("Action", "Step 15: Intake complete");
        telemetry.update();
    }

    /**
     * STEPS 16-20: Navigate back to shooting position
     * Step 16: Move backward 24" to R5,C4
     * Step 17: Rotate 90° CCW (face west)
     * Step 18: Move backward 24" to R6,C4 - COMMENTED
     * Step 19: Start ejection motors (early) - COMMENTED
     * Step 20: Rotate 45° CW (face east toward red depot) - COMMENTED
     */
    private void returnToShootingPosition() {
        // Step 16: Move back to R5,C4 (only 24" since we only moved 24" forward)
        telemetry.addData("Action", "Step 16: Move back to R5,C4");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -TILE_SIZE, -TILE_SIZE, -TILE_SIZE, -TILE_SIZE, 5.0);

        // Step 17: Rotate 90° CCW (face west)
        telemetry.addData("Action", "Step 17: Rotate 90° CCW");
        telemetry.update();
        rotate(TURN_SPEED, -90, 5.0);

        /* COMMENTED BECAUSE WE DO NOT NEED THIS
        // Step 18: Move back 1 tile to R6,C4
        telemetry.addData("Action", "Step 18: Move back to R6,C4");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -TILE_SIZE, -TILE_SIZE, -TILE_SIZE, -TILE_SIZE, 5.0);

        // Step 19: Start ejection motor early for warmup
        telemetry.addData("Action", "Step 19: Start ejection motors");
        telemetry.update();
        ejectionMotor.setPower(EJECTION_POWER);
        intakeMotor.setPower(INTAKE_POWER);

        // Step 20: Rotate 45° CW to face red depot
        telemetry.addData("Action", "Step 20: Rotate 45° CW");
        telemetry.update();
        rotate(TURN_SPEED, 45, 5.0);
	*/
    }

    /**
     * STEPS 21-28: Eject balls (number depends on successful intake)
     * Step 21: Wait 1.5s (motor warmup)
     * Step 22-23: Start feeders, wait 2.5s (first ball)
     * Step 24-25: Continue feeders, wait 2.5s (second ball - no pause)
     * Step 26: Stop all
     */
    private void ejectBalls() {
        // Step 21: Wait for ejection motor to reach full speed
        telemetry.addData("Action", "Step 21: Warmup");
        telemetry.update();
        sleep(EJECTION_WARMUP_MS);

        // Step 22-23: First ball
        telemetry.addData("Action", "Step 22-23: First ball");
        telemetry.update();
        feederServoLeft.setPower(FEEDER_POWER);
        feederServoRight.setPower(FEEDER_POWER);
        sleep(FIRST_BALL_EJECT_MS);

        // Step 24-25: Second ball (no pause - continuous feeders)
        telemetry.addData("Action", "Step 24-25: Second ball");
        telemetry.update();
        sleep(SECOND_BALL_EJECT_MS);

        // Step 26: Stop all
        stopBallHandling();
        telemetry.addData("Action", "Step 26: Complete");
        telemetry.update();
    }

    /**
     * Stop all ball handling motors and servos
     */
    private void stopBallHandling() {
        ejectionMotor.setPower(0);
        intakeMotor.setPower(0);
        feederServoLeft.setPower(0);
        feederServoRight.setPower(0);
    }

    /**
     * Drive using encoders with dynamic deceleration based on distance
     * Long movements (≥2 tiles) use extended deceleration for smoother stops
     * @param speed Maximum speed (0.0 to 1.0)
     * @param frontRightInches Distance for front right wheel
     * @param backRightInches Distance for back right wheel
     * @param frontLeftInches Distance for front left wheel
     * @param backLeftInches Distance for back left wheel
     * @param timeoutS Maximum time to wait
     */
    private void encoderDrive(double speed, double frontRightInches, double backRightInches,
                              double frontLeftInches, double backLeftInches, double timeoutS) {
        int frTarget, brTarget, flTarget, blTarget;

        if (opModeIsActive()) {
            frTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            brTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);
            flTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            blTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);

            frontRight.setTargetPosition(frTarget);
            backRight.setTargetPosition(brTarget);
            frontLeft.setTargetPosition(flTarget);
            backLeft.setTargetPosition(blTarget);

            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            // Calculate average movement distance to determine deceleration strategy
            double avgDistance = (Math.abs(frontRightInches) + Math.abs(backRightInches) +
                    Math.abs(frontLeftInches) + Math.abs(backLeftInches)) / 4.0;

            // Use extended deceleration for long movements (≥2 tiles)
            double decelDistance = (avgDistance >= LONG_MOVE_THRESHOLD) ?
                    LONG_DECEL_DISTANCE : SHORT_DECEL_DISTANCE;

            int decelThreshold = (int)(decelDistance * COUNTS_PER_INCH);

            // Move to position with automatic deceleration near target
            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())) {

                // Calculate remaining distance for each wheel
                int frRemaining = Math.abs(frTarget - frontRight.getCurrentPosition());
                int brRemaining = Math.abs(brTarget - backRight.getCurrentPosition());
                int flRemaining = Math.abs(flTarget - frontLeft.getCurrentPosition());
                int blRemaining = Math.abs(blTarget - backLeft.getCurrentPosition());

                int minRemaining = Math.min(Math.min(frRemaining, brRemaining),
                        Math.min(flRemaining, blRemaining));

                // Gradually slow down based on distance remaining
                double currentSpeed;
                if (minRemaining < decelThreshold) {
                    double decelRatio = (double)minRemaining / decelThreshold;
                    currentSpeed = MIN_SPEED + (speed - MIN_SPEED) * decelRatio;
                } else {
                    currentSpeed = speed;
                }

                frontRight.setPower(Math.abs(currentSpeed));
                backRight.setPower(Math.abs(currentSpeed));
                frontLeft.setPower(Math.abs(currentSpeed));
                backLeft.setPower(Math.abs(currentSpeed));
            }

            // Stop all motors
            frontRight.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);

            // Return to normal mode
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Rotate the robot by specified degrees
     * @param speed Rotation speed (0.0 to 1.0)
     * @param degrees Degrees to rotate (positive = clockwise, negative = counter-clockwise)
     * @param timeoutS Maximum time to wait
     * TWEAK: Adjust ROTATION_CALIBRATION constant if robot over/under rotates
     */
    private void rotate(double speed, double degrees, double timeoutS) {
        double inches = degrees * DEGREES_TO_INCHES;

        if (opModeIsActive()) {
            encoderDrive(speed, -inches, -inches, inches, inches, timeoutS);
        }
    }

    /**
     * Reset all drive motor encoders to zero
     */
    private void resetEncoders() {
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}