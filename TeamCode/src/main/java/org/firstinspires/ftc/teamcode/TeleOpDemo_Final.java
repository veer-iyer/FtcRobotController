package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * TeleOp Demo - V0
 * Ball Handler Robot - DECODE 25 Competition
 */
@TeleOp(name="TeleOp Demo Final")
public class TeleOpDemo_Final extends OpMode {

    // Ball handling hardware
    private DcMotor intakeMotor;
    private DcMotor ejectionMotor;
    private CRServo feederServoLeft;
    private CRServo feederServoRight;

    // Drive motors (mecanum configuration)
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Configuration constants - TWEAK THESE VALUES
    private final double SPEED_FACTOR = 0.8;      // Drive speed: 0.3 (slow) to 1.0 (fast)
    private final double EJECTION_POWER = 0.825;   // Powered ejection
    private final double NORMAL_EJECTION_POWER = 0.65; // Normal Ejection power
    private final double TRIGGER_THRESHOLD = 0.1; // Trigger sensitivity: 0.0 to 0.3

    /**
     * INITIALIZATION METHOD
     * Runs ONCE when you press the INIT button.
     * Sets up all motors and servos before the match starts.
     */
    @Override
    public void init() {
        // Connect to hardware using Driver Station configuration names
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        ejectionMotor = hardwareMap.get(DcMotor.class, "ejection_motor");
        feederServoLeft = hardwareMap.get(CRServo.class, "feeder_servo_left");
        feederServoRight = hardwareMap.get(CRServo.class, "feeder_servo_right");

        // Drive motors (NOTE: front/back swapped in config vs physical robot)
        frontLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "back_right");
        backLeft = hardwareMap.get(DcMotor.class, "front_left");
        backRight = hardwareMap.get(DcMotor.class, "front_right");

        // Configure all motors to run without encoders (direct power control)
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ejectionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse left side motors so forward is forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Stop all servos initially
        feederServoLeft.setPower(0);
        feederServoRight.setPower(0);
    }

    /**
     * MAIN LOOP METHOD
     * Runs CONTINUOUSLY (50-100 times per second) during the match.
     * Reads controller inputs and controls the robot.
     */
    @Override
    public void loop() {
        // Handle all robot subsystems
        handleDriving();
        handleIntake();
        handleEjection();
        handleFeeders();
        displayTelemetry();
    }

    /**
     * DRIVING CONTROL
     * Controls mecanum drive using joysticks.
     *
     * CONTROLS:
     * - Left Stick Y: Forward/Backward
     * - Left Stick X: Rotate Left/Right
     * - Right Stick X: Strafe Left/Right
     *
     * TWEAKABLE:
     * - Change SPEED_FACTOR at top of file (0.3 = slow, 1.0 = fast)
     */
    private void handleDriving() {
        // Read joystick inputs (negative Y because joystick is inverted)
        double drive = -gamepad1.left_stick_y;   // Forward/backward
        double strafe = -gamepad1.right_stick_x; // Strafe left/right
        double turn = gamepad1.left_stick_x;     // Rotation

        // Calculate power for each wheel (mecanum drive formula)
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        // Normalize: keep all powers within -1.0 to 1.0 range
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Apply speed reduction for better control
        frontLeftPower *= SPEED_FACTOR;
        frontRightPower *= SPEED_FACTOR;
        backLeftPower *= SPEED_FACTOR;
        backRightPower *= SPEED_FACTOR;

        // Send power to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * INTAKE CONTROL
     * Controls the intake motor.
     *
     * CONTROLS:
     * - Left Bumper (LB): Intake balls on field
     * - X: Reverse intake to remove balls from robot
     *
     * TWEAKABLE:
     * - Change power value in setPower() call (currently -1.0)
     * - Negative power = intake balls IN
     * - Positive power = push balls OUT
     */
    private void handleIntake() {
        if (gamepad1.left_bumper) {
            // Intake ball
            intakeMotor.setPower(-1.0);
        } else if (gamepad1.x) {
            // Reverse intake
            intakeMotor.setPower(1.0);
        } else {
            // Stop intake when button released
            intakeMotor.setPower(0);
        }
    }

    /**
     * EJECTION CONTROL
     * Controls the ejection motor for shooting balls.
     *
     * CONTROLS:
     * - Left Trigger: Run ejection motor
     *
     * TWEAKABLE:
     * - Change EJECTION_POWER at top of file (0.5 = slow, 1.0 = max)
     * - Change TRIGGER_THRESHOLD for sensitivity (0.1 = light touch required)
     */
    private void handleEjection() {
        if (gamepad1.b) {
            // Run ejection motor
            ejectionMotor.setPower(EJECTION_POWER);
        } else if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
            // Eject with normal power
            ejectionMotor.setPower(NORMAL_EJECTION_POWER);
        } else {
            // Stop ejection when trigger released
            ejectionMotor.setPower(0);
        }

    }

    /**
     * FEEDER SERVO CONTROL
     * Controls feeder servos to move balls from intake to ejection.
     *
     * CONTROLS:
     * - Right Trigger: Feed balls forward (toward ejection)
     * - Right Bumper (RB): Feed balls backward (unjam/return to storage)
     *
     * TWEAKABLE:
     * - Change servo power values (currently ±1.0)
     * - Left and Right servos spin opposite directions due to physical mounting
     * - To reverse direction: swap the + and - signs
     */
    private void handleFeeders() {
        if (gamepad1.right_bumper) {
            // REVERSE: Unjam by pulling balls back to storage
            feederServoLeft.setPower(-1.0);  // Backward
            feederServoRight.setPower(1.0);  // Backward (opposite physical direction)

        } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            // FORWARD: Move balls toward ejection
            feederServoLeft.setPower(1.0);   // Forward
            feederServoRight.setPower(-1.0); // Forward (opposite physical direction)

        } else {
            // STOP: No button pressed
            feederServoLeft.setPower(0);
            feederServoRight.setPower(0);
        }
    }

    /**
     * TELEMETRY DISPLAY
     * Shows robot status on Driver Station screen.
     * Helps drivers and coaches see what the robot is doing.
     *
     * TWEAKABLE:
     * - Add more telemetry lines to display additional info
     * - Remove lines you don't need to see
     */
    private void displayTelemetry() {
        // Show mechanism status
        telemetry.addData("Intake",
                gamepad1.left_bumper ? "REVERSE (Human)" : "STOPPED");
        telemetry.addData("Ejection",
                gamepad1.left_trigger > TRIGGER_THRESHOLD ? "RUNNING" : "STOPPED");
        telemetry.addData("Feeders",
                gamepad1.right_bumper ? "REVERSE (Unjam)" :
                        (gamepad1.right_trigger > TRIGGER_THRESHOLD ? "FORWARD" : "STOPPED"));

        // Show configuration settings
        telemetry.addData("Speed", "%.0f%%", SPEED_FACTOR * 100);
        telemetry.addData("Ejection Power", "%.0f%%", EJECTION_POWER * 100);

        // Update display
        telemetry.update();
    }
}