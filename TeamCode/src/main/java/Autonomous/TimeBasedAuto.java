package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class TimeBasedAuto extends LinearOpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    private DcMotor transferMotor, intakeMotor;

    private DcMotorEx shooterMotor;

    private Servo feederServo;

    private IMU imu;

    private final ElapsedTime matchTimer = new ElapsedTime();

    private static final double TO_SHOT_IN = 27.0;

    private static final double TO_PARK_FORWARD_IN = 71.0;

    private static final double TO_PARK_LEFT_IN = 20.0;

    private static final double RED_AIM_DEGREES = -42.0;

    private static final double BLUE_AIM_DEGREES = -46.0;

    private static final double CALIBRATION_POWER = 0.35;

    private static final double FORWARD_IPS_AT_CAL_POWER = 18.0;

    private static final double STRAFE_IPS_AT_CAL_POWER = 13.0;

    private static final double FORWARD_POWER = 0.35;

    private static final double STRAFE_POWER = 0.30;

    private static final double YAW_TO_TURN_SIGN = -1.0;

    private static final double DRIVE_HEADING_KP = 0.014;

    private static final double TURN_KP = 0.013;

    private static final double MAX_TURN_POWER = 0.28;

    private static final double MIN_TURN_POWER = 0.09;

    private static final double SHOOTER_TARGET_TICKS_PER_SEC = 1500.0;

    private static final double SHOOTER_TOLERANCE_TPS = 100.0;

    private static final double FEEDER_REST = 0.18;

    private static final double FEEDER_PUSH = 0.62;

    private static final double TRANSFER_POWER = 0.40;

    private static final int SHOTS_TO_FIRE = 3;

    private static final double SHOT_CUTOFF_SEC = 18.0;

    private static final double MOVEMENT_CUTOFF_SEC = 28.5;

    private boolean redAlliance = true;

    private boolean shootingRequested = true;

    private boolean shooterAvailable;

    @Override
    public void runOpMode() {
        initDrive();

        initImu();

        initMechanisms();

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) redAlliance = true;

            if (gamepad1.dpad_right) redAlliance = false;

            if (gamepad1.dpad_down) shootingRequested = false;

            if (gamepad1.dpad_up) shootingRequested = true;

            telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");

            telemetry.addData("Shooter Available", shooterAvailable);

            telemetry.addData("Shooting Enabled", shootingRequested);

            telemetry.addLine("LEFT: Red | RIGHT: Blue");

            telemetry.addLine("DOWN: No Shooting | UP: Shooting");

            telemetry.update();

            sleep(50);
        }

        if (isStopRequested()) return;

        waitForStart();

        if (!opModeIsActive()) return;

        matchTimer.reset();

        imu.resetYaw();

        boolean doShooting = shootingRequested && shooterAvailable;

        try {
            if (doShooting) shooterMotor.setVelocity(SHOOTER_TARGET_TICKS_PER_SEC);

            forwardInches(TO_SHOT_IN, FORWARD_POWER);

            double targetAngle = redAlliance ? RED_AIM_DEGREES : BLUE_AIM_DEGREES;

            boolean aimed = turnToHeading(targetAngle, 2.5);

            if (doShooting && aimed && matchTimer.seconds() < SHOT_CUTOFF_SEC) {
                shootPreloadedPollen();
            }

            stopShooter();

            if (!turnToHeading(0.0, 3.0)) {
                telemetry.addLine("Heading recovery failed");

                telemetry.update();

                return;
            }

            forwardInches(TO_PARK_FORWARD_IN, FORWARD_POWER);

            strafeRightInches(-TO_PARK_LEFT_IN, STRAFE_POWER);
        } finally {
            stopDrive();

            stopShooter();

            if (transferMotor != null) transferMotor.setPower(0);

            if (intakeMotor != null) intakeMotor.setPower(0);

            if (feederServo != null) feederServo.setPosition(FEEDER_REST);
        }
    }

    private void initDrive() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");

        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] driveMotors = { frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor };

        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setPower(0);
        }
    }

    private void initImu() {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,

                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );

        imu.initialize(parameters);
    }

    private void initMechanisms() {
        shooterMotor = hardwareMap.tryGet(DcMotorEx.class, "shooterMotor");

        feederServo = hardwareMap.tryGet(Servo.class, "feederServo");

        transferMotor = hardwareMap.tryGet(DcMotor.class, "transferMotor");

        intakeMotor = hardwareMap.tryGet(DcMotor.class, "intakeMotor");

        shooterAvailable = shooterMotor != null && feederServo != null;

        if (shooterMotor != null) {
            shooterMotor.setDirection(DcMotor.Direction.FORWARD);

            shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            shooterMotor.setVelocity(0);
        }

        if (feederServo != null) feederServo.setPosition(FEEDER_REST);

        if (transferMotor != null) {
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            transferMotor.setPower(0);
        }

        if (intakeMotor != null) {
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            intakeMotor.setPower(0);
        }
    }

    private double yaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double yawError(double target) {
        return AngleUnit.normalizeDegrees(target - yaw());
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void setDrivePower(double forward, double strafeRight, double turn) {
        double fl = forward + strafeRight + turn;

        double fr = forward - strafeRight - turn;

        double bl = forward - strafeRight + turn;

        double br = forward + strafeRight - turn;

        double max = Math.max(
            1.0,
            Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),

                Math.max(Math.abs(bl), Math.abs(br))
            )
        );

        frontLeftMotor.setPower(fl / max);

        frontRightMotor.setPower(fr / max);

        backLeftMotor.setPower(bl / max);

        backRightMotor.setPower(br / max);
    }

    private void stopDrive() {
        setDrivePower(0, 0, 0);
    }

    private boolean turnToHeading(double targetHeading, double tolerance) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        while (opModeIsActive() && timer.seconds() < 3.0 && matchTimer.seconds() < MOVEMENT_CUTOFF_SEC) {
            double error = yawError(targetHeading);

            if (Math.abs(error) <= tolerance) {
                stopDrive();

                return true;
            }

            double turnPower = clamp(
                Math.abs(error) * TURN_KP,

                MIN_TURN_POWER,

                MAX_TURN_POWER
            );

            double turn = YAW_TO_TURN_SIGN * Math.signum(error) * turnPower;

            setDrivePower(0, 0, turn);

            telemetry.addData("Target Heading", targetHeading);

            telemetry.addData("Actual Heading", yaw());

            telemetry.addData("Heading Error", error);

            telemetry.update();

            idle();
        }

        stopDrive();

        return Math.abs(yawError(targetHeading)) <= tolerance;
    }

    private void forwardInches(double inches, double power) {
        moveTimed(inches, 0, power, FORWARD_IPS_AT_CAL_POWER);
    }

    private void strafeRightInches(double inches, double power) {
        moveTimed(0, inches, power, STRAFE_IPS_AT_CAL_POWER);
    }

    private void moveTimed(double forwardInches, double strafeInches, double power, double inchesPerSecond) {
        double distance = forwardInches != 0 ? forwardInches : strafeInches;

        if (Math.abs(distance) < 0.01 || inchesPerSecond <= 0 || power <= 0) return;

        double speedAtPower = inchesPerSecond * (power / CALIBRATION_POWER);

        double duration = Math.abs(distance) / speedAtPower;

        double holdHeading = 0;

        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        while (opModeIsActive() && timer.seconds() < duration && matchTimer.seconds() < MOVEMENT_CUTOFF_SEC) {
            double error = yawError(holdHeading);

            double correction = clamp(YAW_TO_TURN_SIGN * error * DRIVE_HEADING_KP, -0.16, 0.16);

            double forward = forwardInches != 0 ? Math.signum(distance) * power : 0;

            double strafe = strafeInches != 0 ? Math.signum(distance) * power : 0;

            setDrivePower(forward, strafe, correction);

            telemetry.addData("Auto Time", "%.1f", matchTimer.seconds());

            telemetry.addData("Target Heading", holdHeading);

            telemetry.addData("Actual Heading", yaw());

            telemetry.update();

            idle();
        }

        stopDrive();
    }

    private void shootPreloadedPollen() {
        for (int shot = 0; shot < SHOTS_TO_FIRE && opModeIsActive(); shot++) {
            if (matchTimer.seconds() >= SHOT_CUTOFF_SEC) break;

            boolean ready = waitForFlywheel(2.5);

            if (!ready) {
                telemetry.addLine("Shooter not ready");

                telemetry.update();

                break;
            }

            if (transferMotor != null) transferMotor.setPower(TRANSFER_POWER);

            feederServo.setPosition(FEEDER_PUSH);

            pauseMs(220, SHOT_CUTOFF_SEC);

            feederServo.setPosition(FEEDER_REST);

            if (transferMotor != null) transferMotor.setPower(0);

            pauseMs(450, SHOT_CUTOFF_SEC);

            telemetry.addData("Shots Attempted", shot + 1);

            telemetry.update();
        }
    }

    private boolean waitForFlywheel(double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();

        ElapsedTime stableTimer = new ElapsedTime();

        timer.reset();

        stableTimer.reset();

        boolean previouslyInTolerance = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds && matchTimer.seconds() < SHOT_CUTOFF_SEC) {
            double actualVelocity = Math.abs(shooterMotor.getVelocity());

            boolean withinTolerance = Math.abs(actualVelocity - SHOOTER_TARGET_TICKS_PER_SEC) < SHOOTER_TOLERANCE_TPS;

            if (withinTolerance && !previouslyInTolerance) stableTimer.reset();

            if (withinTolerance && stableTimer.milliseconds() >= 180) return true;

            previouslyInTolerance = withinTolerance;

            telemetry.addData("Shooter Velocity", actualVelocity);

            telemetry.addData("Target Velocity", SHOOTER_TARGET_TICKS_PER_SEC);

            telemetry.update();

            idle();
        }

        return false;
    }

    private void pauseMs(long milliseconds, double cutoffSeconds) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();

        while (opModeIsActive() && timer.milliseconds() < milliseconds && matchTimer.seconds() < cutoffSeconds) {
            idle();
        }
    }

    private void stopShooter() {
        if (shooterMotor != null) shooterMotor.setVelocity(0);
    }
}
