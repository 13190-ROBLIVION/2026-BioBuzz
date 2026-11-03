package Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "Drivetrain", group = "Teleop")
public class Drivetrain extends LinearOpMode {
    // ----- Hardware (created in onConstruct, like components) -----
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private IMU imu;
    private SparkFunOTOS otos;
    private boolean useOtos = false;

    // ----- Actor state (written by one tick step, read by the next) -----
    // Input state (from getInputs)
    private double driveForward; // -1..1, left stick Y through the input pipeline
    private double driveStrafe; // -1..1, left stick X through the input pipeline
    private double driveTurn; // -1..1, right stick X through the input pipeline
    private boolean slowMode;

    // Localization state (from calculateHeading)
    private double botHeadingRadians;
    private SparkFunOTOS.Pose2D pose = null;
    private SparkFunOTOS.Pose2D velocity = null;

    // Output state (from fieldRotation)
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    // ----- Per-frame input bookkeeping -----
    private boolean optionsPressedLastFrame = false;

    // ----- Input pipeline state ----
    // Deadzone hysteresis latches: a stick becomes active past DZ_ACTIVATE and
    // only goes back to zeroed below DZ_REACTIVATE, so resting on the
    // threshold can't chatter between "stopped" and "creeping".
    private boolean driveStickActive = false;
    private boolean turnStickActive = false;

    // ----- Delta time -----
    // One clock for the whole play session; delta = now - lastTick.
    private final ElapsedTime playClock = new ElapsedTime();
    private double lastTickSeconds = 0.0;
    private double lastDeltaTimeSeconds = 0.0; // the delta this tick ran with

    // ----- Stability -----
    // stillDurationSeconds accumulates DeltaSeconds while the bot is still and
    // resets the moment it moves
    private double stillDurationSeconds = 0.0;

    private static class constants {
        // ----- Input feel -----
        // Revert to the old feel: DRIVE_CURVE_EXP = 1.0, TURN_CURVE_EXP = 1.0,
        // DRIVE_TAU = 0.0, TURN_TAU = 0.0. Change ONE thing at a time and let
        // the driver feel it in practice before keeping it!
        // Stick deadzone, as magnitudes: activate past DZ_ACTIVATE, stay active
        // until below DZ_REACTIVATE (hysteresis).
        private static final double DZ_ACTIVATE = 0.05;
        private static final double DZ_REACTIVATE = 0.02;
        // Response curve exponent on the deadzone-rescaled magnitude.
        // 1.0 = linear, 2.0 = quadratic: much finer control near center,
        // full speed at full deflection either way.
        private static final double DRIVE_CURVE_EXP = 2.0;
        private static final double TURN_CURVE_EXP = 1.5;
        // Exponential smoothing time constants, in SECONDS. Bigger = smoother
        // but slightly laggier; 0.0 = off (snap straight to the raw stick).
        private static final double DRIVE_TAU = 0.10;
        private static final double TURN_TAU = 0.05;
        // Inches per second. The buffer for the bot to still be counted as still.
        private static final double stillSpeedBuffer = 0.5;
        // Degrees per second. The buffer for the bot to still be counted as still.
        private static final double stillTurnBuffer = 2.0;
        // How long the bot must stay still before isStable() flips true.
        private static final double stillDebounceSeconds = 0.25;
        private static final double SLOW_MODE_MULTIPLIER = 0.35;
        private static final double STRAFE_GAIN = 1.1; // mecanums strafe slower than they drive
    }

    // ----- Game loop -----

    @Override
    public void runOpMode() {
        onConstruct();
        waitForStart();
        beginPlay();

        while (opModeIsActive()) {
            lastDeltaTimeSeconds = getDeltaTimeSeconds();
            tick(lastDeltaTimeSeconds);
        }
    }

    /**
     * gets a single DeltaTime float, in SECONDS. This does the
     * same: one clock, delta = now - last. (The old pattern of "new
     * ElapsedTime() every loop" always reported ~0, which makes any
     * delta-scaled behavior silently dead.)
     */
    private double getDeltaTimeSeconds() {
        double now = playClock.seconds();
        double deltaTimeSeconds = now - lastTickSeconds;
        lastTickSeconds = now;

        return deltaTimeSeconds;
    }

    // ----- Lifecycle steps -----

    /**
     * Runs before the op mode runs. Not quite the constructor.
     * OnConstruction / PostInitializeComponents.
     */
    private void onConstruct() {
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
    }

    /**
     * BeginPlay - one-shot setup at the start of play.
     */
    private void beginPlay() {
        // No isStopRequested() check here: if stop was asked for, the tick loop
        // below never runs anyway - and resetting sensors beforehand is harmless.
        imu.resetYaw();

        if (useOtos) {
            otos.resetTracking(); // zero X/Y/heading at the robot's starting spot
        }

        playClock.reset();
        lastTickSeconds = 0.0;
    }

    /**
     * Stuff to run while the bot is playing.
     *
     * @param deltaTimeSeconds seconds since the previous tick
     */
    private void tick(double deltaTimeSeconds) {
        getPreviousOptions(); // input edge detection first, it can re-zero heading
        getInputs(deltaTimeSeconds);
        calculateHeading();
        fieldRotation();
        updateStability(deltaTimeSeconds);
        tickTelemetry();
    }

    // ----- Tick steps (each reads what the previous one wrote) -----

    /**
     * EnhancedInput edge handling - act only on the transition
     * "not pressed -> pressed", so holding the button re-zeros exactly once.
     */
    void getPreviousOptions() {
        boolean optionsPressed = gamepad1.options;

        if (optionsPressed && !optionsPressedLastFrame) {
            imu.resetYaw();

            if (useOtos) {
                otos.resetTracking();
            }
        }

        optionsPressedLastFrame = optionsPressed;
    }

    /**
     * PlayerInput -> input axes, through an EnhancedInput-style modifier
     * chain: radial deadzone with hysteresis -> response curve -> exponential
     * smoothing. Raw sticks are inverted (stick up is negative), everything
     * scales by real time (deltaTimeSeconds), and each stage can be switched
     * off via its constant for the old feel.
     */
    void getInputs(double deltaTimeSeconds) {
        // -- Deadzone: radial on the drive stick (it's one stick vector, so a
        //    diagonal shouldn't be "twice as dead" as a cardinal direction),
        //    1D on the turn stick. Rescaled so output starts at 0 at the
        //    deadzone edge instead of jumping straight to the threshold value.
        double rawForward = -gamepad1.left_stick_y;
        double rawStrafe = gamepad1.left_stick_x;
        double mag = Math.hypot(rawForward, rawStrafe);

        if (!driveStickActive && mag >= constants.DZ_ACTIVATE) {
            driveStickActive = true;
        } else if (driveStickActive && mag < constants.DZ_REACTIVATE) {
            driveStickActive = false;
        }

        double targetForward = 0.0;
        double targetStrafe = 0.0;

        if (driveStickActive && mag > 0.0) {
            double rescaled = Math.max(0.0, (mag - constants.DZ_ACTIVATE) / (1.0 - constants.DZ_ACTIVATE));
            double scale = Math.pow(rescaled, constants.DRIVE_CURVE_EXP) / mag;

            targetForward = rawForward * scale;
            targetStrafe = rawStrafe * scale;
        }

        double rawTurn = -gamepad1.right_stick_x;
        double turnMag = Math.abs(rawTurn);

        if (!turnStickActive && turnMag >= constants.DZ_ACTIVATE) {
            turnStickActive = true;
        } else if (turnStickActive && turnMag < constants.DZ_REACTIVATE) {
            turnStickActive = false;
        }

        double targetTurn = 0.0;

        if (turnStickActive) {
            double turnRescaled = Math.max(0.0, (turnMag - constants.DZ_ACTIVATE) / (1.0 - constants.DZ_ACTIVATE));
            targetTurn = Math.signum(rawTurn) * Math.pow(turnRescaled, constants.TURN_CURVE_EXP);
        }

        slowMode = gamepad1.left_bumper;

        // -- Exponential smoothing toward the shaped targets. Frame-rate
        //    independent: with tau = 0.10s
        //    an axis reaches ~95% of the stick position in ~0.3s, and decays
        //    just as smoothly on release instead of cutting off hard.
        driveForward = smoothAxis(driveForward, targetForward, deltaTimeSeconds, constants.DRIVE_TAU);
        driveStrafe = smoothAxis(driveStrafe, targetStrafe, deltaTimeSeconds, constants.DRIVE_TAU);
        driveTurn = smoothAxis(driveTurn, targetTurn, deltaTimeSeconds, constants.TURN_TAU);
    }

    /**
     * One exponential-smoothing step: move 'current' toward 'target' by a
     * fraction of the remaining distance that depends on real elapsed time,
     * not on how many loop iterations happened. tau <= 0 disables smoothing.
     */
    private static double smoothAxis(double current, double target, double deltaTimeSeconds, double tau) {
        if (tau <= 0.0) {
            return target;
        }

        double alpha = 1.0 - Math.exp(-deltaTimeSeconds / tau);

        return current + (target - current) * alpha;
    }

    /**
     * the localization half of MovementComponent. Fills heading, pose and
     * velocity for the rest of the tick.
     */
    void calculateHeading() {
        if (useOtos) {
            pose = otos.getPosition();
            velocity = otos.getVelocity();
            botHeadingRadians = Math.toRadians(normalizeDegrees(pose.h));
        } else {
            velocity = null;
            botHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
    }

    /**
     * the actuation half - rotate the input into the
     * robot frame (field-centric), mix mecanum powers, apply slow mode, drive.
     */
    void fieldRotation() {
        double rotX = driveStrafe * Math.cos(-botHeadingRadians) - driveForward * Math.sin(-botHeadingRadians);
        double rotY = driveStrafe * Math.sin(-botHeadingRadians) + driveForward * Math.cos(-botHeadingRadians);

        rotX *= constants.STRAFE_GAIN;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTurn), 1.0);

        frontLeftPower = (rotY + rotX + driveTurn) / denominator;
        backLeftPower = (rotY - rotX + driveTurn) / denominator;
        frontRightPower = (rotY - rotX - driveTurn) / denominator;
        backRightPower = (rotY + rotX - driveTurn) / denominator;

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
    }

    /**
     * accumulate HeldDuration += DeltaTime
     * while the condition holds, reset otherwise, trigger past threshold.
     * This is why deltaTime matters: the debounce time is real seconds
     * regardless of how fast the loop runs.
     */
    void updateStability(double deltaTimeSeconds) {
        boolean instantStill = false;

        if (useOtos && velocity != null) {
            double speedInPerSec = Math.hypot(velocity.x, velocity.y);
            double turnDegPerSec = Math.abs(velocity.h);

            instantStill = speedInPerSec < constants.stillSpeedBuffer && turnDegPerSec < constants.stillTurnBuffer;
        }

        if (instantStill) {
            stillDurationSeconds += deltaTimeSeconds;
        } else {
            stillDurationSeconds = 0.0;
        }
    }

    /**
     * True once the bot has been still for stillDebounceSeconds. Requires the
     * OTOS (no linear velocity in IMU fallback mode). This is the signal
     * Shooting.isShootingReady() wants.
     */
    public boolean isStable() {
        if (!useOtos) {
            return false; // safe default: assume moving when we can't measure
        }

        return stillDurationSeconds >= constants.stillDebounceSeconds;
    }

    /**
     * Debug stuff. IDK
     */
    void tickTelemetry() {
        double displayDelta = Math.max(lastDeltaTimeSeconds, 0);

        telemetry.addLine("===== POSE =====");
        telemetry.addData("Localizer", useOtos ? "OTOS" : "IMU FALLBACK");
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(botHeadingRadians));

        if (useOtos && pose != null && velocity != null) {
            telemetry.addData("X", "%.1f in", pose.x);
            telemetry.addData("Y", "%.1f in", pose.y);
            telemetry.addData("Speed", "%.1f in/s", Math.hypot(velocity.x, velocity.y));
            telemetry.addData("Turn Rate", "%.1f deg/s", velocity.h);
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
        telemetry.addData("stat fps", displayDelta > 0 ? String.format(Locale.US, "%.0f", 1.0 / displayDelta) : "-");
        telemetry.addData("Stability", isStable() ? "STABLE" : String.format(Locale.US, "settling %.2fs", stillDurationSeconds));
        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addLine("OPTIONS = Reset Heading + Position");

        telemetry.update();
    }

    // ----- Helpers -----

    private double normalizeDegrees(double angle) {
        while (angle > 180) {
            angle -= 360;
        }

        while (angle <= -180) {
            angle += 360;
        }

        return angle;
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
}