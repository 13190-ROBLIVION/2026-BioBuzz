package Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Canonical field-centric teleop (merged Drivetrain + otosLocalizer).
 *
 * - Heading and position come from the SparkFun OTOS ("sensor_otos") when it is
 *   connected and calibrated. This also gives us X/Y position and velocity for
 *   free (velocity is what shooting-on-the-move compensation will need).
 * - If the OTOS is missing or calibration fails, we fall back to the control
 *   hub IMU so the robot still drives at a meet instead of refusing to start.
 *
 * Controls:
 *   Left stick        = drive / strafe (field-centric)
 *   Right stick X     = rotate
 *   OPTIONS (edge)    = reset OTOS tracking + IMU yaw
 *   Hold LEFT BUMPER  = slow mode
 */
@TeleOp(name = "Drivetrain", group = "Teleop")
public class Drivetrain extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private IMU imu;
    private SparkFunOTOS otos;

    private boolean useOtos = false;

    private static class constants {
        private static final double DEADZONE = 0.05;
        // Cm per second? The buffer for the bot to still be counted as still.
        private static final double stillSpeedBuffer = 0.5;
        // In Degrees per second. The buffer for the bot to still be counted as still.
        private static final double stillTurnBuffer = 2.0;
        private static final double SLOW_MODE_MULTIPLIER = 0.35;
        private static final double STRAFE_GAIN = 1.1; // mecanums strafe slower than they drive
    }

    @Override
    public void runOpMode() {
        initDriveMotors();
        initImu();
        initOtos();

        telemetry.addLine("FIELD CENTRIC READY");
        telemetry.addData("Localizer", useOtos ? "OTOS" : "IMU FALLBACK");
        telemetry.addLine("Left Stick = Drive / Strafe");
        telemetry.addLine("Right Stick X = Rotate");
        telemetry.addLine("OPTIONS = Reset Heading + Position");
        telemetry.addLine("Hold LEFT BUMPER = Slow Mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        imu.resetYaw();

        if (useOtos) {
            otos.resetTracking(); // zero X/Y/heading at the robot's starting spot
        }

        boolean previousOptions = false;

        while (opModeIsActive()) {
            // Edge-detected reset so holding the button doesn't re-zero every loop
            boolean optionsPressed = gamepad1.options;

            if (optionsPressed && !previousOptions) {
                imu.resetYaw();

                if (useOtos) {
                    otos.resetTracking();
                }
            }

            previousOptions = optionsPressed;

            // ----- Inputs -----
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (Math.abs(y) < constants.DEADZONE) {
                y = 0;
            }

            if (Math.abs(x) < constants.DEADZONE) {
                x = 0;
            }

            if (Math.abs(rx) < constants.DEADZONE) {
                rx = 0;
            }

            // ----- Heading: OTOS primary, IMU fallback -----
            SparkFunOTOS.Pose2D pose = null;
            SparkFunOTOS.Pose2D vel = null;
            double botHeadingRadians;

            if (useOtos) {
                pose = otos.getPosition();
                vel = otos.getVelocity();
                botHeadingRadians = Math.toRadians(normalizeDegrees(pose.h));
            } else {
                botHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // ----- Field-centric rotation (same math as before, + strafe gain) -----
            double rotX = x * Math.cos(-botHeadingRadians) - y * Math.sin(-botHeadingRadians);
            double rotY = x * Math.sin(-botHeadingRadians) + y * Math.cos(-botHeadingRadians);

            rotX *= constants.STRAFE_GAIN;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            boolean slowMode = gamepad1.left_bumper;

            if (slowMode) {
                frontLeftPower *= constants.SLOW_MODE_MULTIPLIER;
                frontRightPower *= constants.SLOW_MODE_MULTIPLIER;
                backLeftPower *= constants.SLOW_MODE_MULTIPLIER;
                backRightPower *= constants.SLOW_MODE_MULTIPLIER;
            }

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            // ----- Telemetry -----
            telemetry.addLine("===== POSE =====");
            telemetry.addData("Localizer", useOtos ? "OTOS" : "IMU FALLBACK");
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(botHeadingRadians));

            if (useOtos) {
                telemetry.addData("X", "%.1f in", pose.x);
                telemetry.addData("Y", "%.1f in", pose.y);
                telemetry.addData("Speed", "%.1f in/s", Math.hypot(vel.x, vel.y));
                telemetry.addData("Turn Rate", "%.1f deg/s", vel.h);
            }

            telemetry.addLine();
            telemetry.addLine("===== ENCODERS =====");
            telemetry.addData("Front Left", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Front Right", frontRightMotor.getCurrentPosition());
            telemetry.addData("Back Left", backLeftMotor.getCurrentPosition());
            telemetry.addData("Back Right", backRightMotor.getCurrentPosition());

            telemetry.addLine();
            telemetry.addLine("===== MOTOR POWER =====");
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("BR", "%.2f", backRightPower);

            telemetry.addLine();
            telemetry.addLine("===== STATUS =====");
            telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
            telemetry.addLine("OPTIONS = Reset Heading + Position");

            telemetry.update();
        }
    }

    private void initDriveMotors() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void initImu() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD // TODO: verify on the robot, op modes disagree on this
        );

        imu.initialize(new IMU.Parameters(orientation));
    }

    private void initOtos() {
        // tryGet: if sensor_otos isn't in the robot config, drive anyway on the IMU
        otos = hardwareMap.tryGet(SparkFunOTOS.class, "sensor_otos");

        if (otos == null) {
            useOtos = false;
            return;
        }

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0)); // TODO: set to the OTOS mounting offset once measured

        // Keep the robot completely still during init, this spins the OTOS IMU calibration
        boolean calibrated = otos.calibrateImu();

        if (!calibrated) {
            telemetry.addLine("OTOS calibration FAILED - falling back to hub IMU");
            telemetry.update();

            useOtos = false;
            return;
        }

        useOtos = true;
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180) {
            angle -= 360;
        }

        while (angle <= -180) {
            angle += 360;
        }

        return angle;
    }

    /**
     * True when the robot is (nearly) stationary — not driving AND not rotating.
     * Uses the OTOS velocity. In IMU fallback mode there is no linear velocity
     * source, so we conservatively report "moving" (returns false).
     */
    public boolean isStable() {
        if (!useOtos) {
            return false; // safe default: assume moving when we can't measure
        }

        SparkFunOTOS.Pose2D vel = otos.getVelocity();

        double speedInPerSec = Math.hypot(vel.x, vel.y);
        double turnDegPerSec = Math.abs(vel.h);

        return speedInPerSec < constants.stillSpeedBuffer && turnDegPerSec < constants.stillTurnBuffer;
    }
}
