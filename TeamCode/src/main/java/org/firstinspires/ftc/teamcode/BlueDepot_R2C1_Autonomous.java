package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="BlueDepot_R2C1_Autonomous_V0", group="Competition")
public class BlueDepot_R2C1_Autonomous extends LinearOpMode {

    // Drive motors
    private DcMotor frontRight, backRight, frontLeft, backLeft;

    // Manipulation motors and servos
    private DcMotor ejectionMotor, intakeMotor;
    private CRServo feederServoRight, feederServoLeft;

    private ElapsedTime runtime = new ElapsedTime();

    // Drive constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.78;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.7;
    private static final double TURN_SPEED = 0.3;
    private static final double TILE_SIZE = 24.0;
    private static final double HALF_TILE = 12.0;

    // Deceleration parameters
    private static final double DECEL_DISTANCE_INCHES = 3.0;
    private static final double MIN_SPEED = 0.2;

    // Rotation calibration
    private static final double ROTATION_CALIBRATION = 0.95;
    private static final double ROBOT_WIDTH = 14.0;
    private static final double DEGREES_TO_INCHES = (ROBOT_WIDTH * 3.1415 * ROTATION_CALIBRATION) / 180.0;

    // Motor/servo powers
    private static final double EJECTION_POWER = 0.6;
    private static final double INTAKE_POWER = 1.0;
    private static final double FEEDER_POWER = 1.1;

    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");

        // Set drive motor directions
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize manipulation motors and servos
        ejectionMotor = hardwareMap.get(DcMotor.class, "ejection_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        feederServoRight = hardwareMap.get(CRServo.class, "feeder_servo_right");
        feederServoLeft = hardwareMap.get(CRServo.class, "feeder_servo_left");

        // Set manipulation motor/servo directions
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        ejectionMotor.setDirection(DcMotor.Direction.FORWARD);
        feederServoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        feederServoRight.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        // setup webcam
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        telemetry.addData("Status", "Ready for Blue Depot Autonomous V0");
        telemetry.addData("Start Position", "R1, C2 facing right");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Step 1: Bot starts at R1, C2 facing right
            telemetry.addData("Step", "1: Starting position R1,C2");
            telemetry.update();

            // Step 2: Move forward to R1, C3
            telemetry.addData("Step", "2: Move to R1,C3");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, TILE_SIZE, TILE_SIZE, TILE_SIZE, TILE_SIZE, 5.0);
            sleep(300);

            // Step 3-4: Strafe right to R3, C3 (2 tiles) - START EJECTION EARLY
            telemetry.addData("Step", "3-4: Strafe to R3,C3 + start ejection motor");
            telemetry.update();

            // Start ejection motor during strafe for head start
            ejectionMotor.setPower(EJECTION_POWER);
            intakeMotor.setPower(INTAKE_POWER);

            encoderDrive(DRIVE_SPEED, -TILE_SIZE * 2, TILE_SIZE * 2, TILE_SIZE * 2, -TILE_SIZE * 2, 10.0);
            sleep(300);

            // Step 5: Rotate clockwise 45 degrees
            telemetry.addData("Step", "5: Rotate 45° CW");
            telemetry.update();
            rotate(TURN_SPEED, 45, 5.0);
            sleep(300);

            // Steps 6-13: First ejection sequence (motor already running)
            telemetry.addData("Step", "6-13: Complete first ejection");
            telemetry.update();
            completeEjectionWithHeadStart();
            sleep(300);

            // Step 14: Rotate clockwise 135 degrees
            telemetry.addData("Step", "14: Rotate 135° CW");
            telemetry.update();
            rotate(TURN_SPEED, 135, 5.0);
            sleep(300);

            // Step 15: Start intake system
            telemetry.addData("Step", "15: Start intake");
            telemetry.update();
            intakeMotor.setPower(INTAKE_POWER);

            // Step 16: Move forward 30 inches (1.25 tiles)
            telemetry.addData("Step", "16: Move forward 30 inches");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, TILE_SIZE * 1.25, TILE_SIZE * 1.25, TILE_SIZE * 1.25, TILE_SIZE * 1.25, 5.0);
            sleep(800);

            // Step 17: Start ejection motor early (head start) while moving back
            telemetry.addData("Step", "17: Start ejection motor (head start)");
            telemetry.update();
            ejectionMotor.setPower(EJECTION_POWER);
            intakeMotor.setPower(INTAKE_POWER);

            // Step 18: Stop intake
            intakeMotor.setPower(0);
            telemetry.addData("Step", "18: Intake complete");
            telemetry.update();
            sleep(300);

            // Step 19: Move back 30 inches (1.25 tiles) to R2, C3
            telemetry.addData("Step", "19: Move back 30 inches to R2,C3");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, -TILE_SIZE * 1.25, -TILE_SIZE * 1.25, -TILE_SIZE * 1.25, -TILE_SIZE * 1.25, 5.0);
            sleep(300);

            // Step 22.5: Start intake to prepare for shooting
            intakeMotor.setPower(INTAKE_POWER);

            // Step 22: Rotate anti-clockwise 135 degrees
            telemetry.addData("Step", "22: Rotate 135° CCW");
            telemetry.update();
            rotate(TURN_SPEED, -135, 5.0);
            sleep(300);

            // Steps 23-30: Complete second ejection sequence (motor already running)
            telemetry.addData("Step", "23-30: Complete second ejection");
            telemetry.update();
            completeEjectionWithHeadStart();
            sleep(300);

            // Step 31: Strafe right 1 tile to exit starting zone
            telemetry.addData("Step", "31: Strafe right 1 tile");
            telemetry.update();
            encoderDrive(DRIVE_SPEED, -TILE_SIZE, TILE_SIZE, TILE_SIZE, -TILE_SIZE, 10.0);
            sleep(300);

            telemetry.addData("Status", "AUTONOMOUS COMPLETE");
            telemetry.update();
        }
    }

    private void completeEjectionWithHeadStart() {
        // Ejection motor and intake motor already running from earlier

        // Step 6/23: Reduced wait time - motor has been running during movement
        sleep(1000);

        // Step 7/24: Start feeders to shoot first ball
        feederServoLeft.setPower(FEEDER_POWER);
        feederServoRight.setPower(FEEDER_POWER);
        telemetry.addData("Ejection", "First ball - feeders started");
        telemetry.update();

        // Step 8/25: Let first ball eject
        sleep(2500);

        // Step 9/26: Stop feeders to prepare for second ball
        feederServoLeft.setPower(0);
        feederServoRight.setPower(0);
        telemetry.addData("Ejection", "Feeders stopped");
        telemetry.update();

        // Step 10/27: Pause between balls
        sleep(800);

        // Step 11/28: Restart feeders to shoot second ball
        feederServoLeft.setPower(FEEDER_POWER);
        feederServoRight.setPower(FEEDER_POWER);
        telemetry.addData("Ejection", "Second ball - feeders restarted");
        telemetry.update();

        // Step 12/29: Let second ball eject
        sleep(2500);

        // Step 13/30: Stop all motors and servos
        ejectionMotor.setPower(0);
        intakeMotor.setPower(0);
        feederServoLeft.setPower(0);
        feederServoRight.setPower(0);
        telemetry.addData("Ejection", "Complete - all stopped");
        telemetry.update();
    }

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
            int decelThreshold = (int)(DECEL_DISTANCE_INCHES * COUNTS_PER_INCH);

            while (opModeIsActive() && (runtime.seconds() < timeoutS) &&
                    (frontRight.isBusy() && backRight.isBusy() && frontLeft.isBusy() && backLeft.isBusy())) {

                int frRemaining = Math.abs(frTarget - frontRight.getCurrentPosition());
                int brRemaining = Math.abs(brTarget - backRight.getCurrentPosition());
                int flRemaining = Math.abs(flTarget - frontLeft.getCurrentPosition());
                int blRemaining = Math.abs(blTarget - backLeft.getCurrentPosition());

                int minRemaining = Math.min(Math.min(frRemaining, brRemaining),
                        Math.min(flRemaining, blRemaining));

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

            frontRight.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            backLeft.setPower(0);

            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void rotate(double speed, double degrees, double timeoutS) {
        double inches = degrees * DEGREES_TO_INCHES;

        if (opModeIsActive()) {
            encoderDrive(speed, -inches, -inches, inches, inches, timeoutS);
        }
    }

    /*
        Start the wheels
        Check for an April Tag
        Check rotation
        Align with April Tag
        Stop Moving
    */
    private void rotateTillAprilTag(double speed, double timeout) {
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setPower(speed);
        backRight.setPower(speed);
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    }

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