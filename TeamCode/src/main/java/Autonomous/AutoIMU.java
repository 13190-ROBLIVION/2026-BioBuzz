package Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "IMU Auto", group = "Autonomous")
public class AutoIMU extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private IMU imu;

    private static final double DRIVE_KP = 0.015;
    private static final double TURN_KP = 0.012;

    private static final double MIN_TURN_POWER = 0.12;
    private static final double MAX_TURN_POWER = 0.45;

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeft");

        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRight");

        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeft");

        backRightMotor = hardwareMap.get(DcMotor.class, "backRight");

        // Set motors to stop safely
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );

        imu.initialize(parameters);

        telemetry.addLine("IMU Auto Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopMotors();
            return;
        }

        imu.resetYaw();

        // Who the hell put this here
        // This code can't throw an exception, why the try block?
        try {
            // Drive forward for 1.2 seconds
            driveTimed(0.35, 0.0, 1200, 0);

            // Turn to 90 degrees
            turnToHeading(90);

            // Strafe for 0.7 seconds while holding 90 degrees
            driveTimed(0.0, 0.35, 700, 90);
        } finally {
            stopMotors();
        }
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private double headingError(double targetHeading) {
        return AngleUnit.normalizeDegrees(targetHeading - getHeading());
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void setDrivePower(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(
            1.0,
            Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)))
        );

        frontLeftMotor.setPower(fl / max);
        frontRightMotor.setPower(fr / max);
        backLeftMotor.setPower(bl / max);
        backRightMotor.setPower(br / max);
    }

    private void driveTimed(double forward, double strafe, long milliseconds, double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.milliseconds() < milliseconds) {
            double error = headingError(targetHeading);

            double correction = clip(-error * DRIVE_KP, -0.20, 0.20);

            setDrivePower(forward, strafe, correction);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Actual Heading", getHeading());
            telemetry.addData("Heading Error", error);
            telemetry.update();

            idle();
        }

        stopMotors();
    }

    private void turnToHeading(double targetHeading) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < 3.0) {
            double error = headingError(targetHeading);

            if (Math.abs(error) <= 2.0) {
                break;
            }

            double turnPower = clip(Math.abs(error) * TURN_KP, MIN_TURN_POWER, MAX_TURN_POWER);

            double turnAmount = -Math.signum(error) * turnPower;

            setDrivePower(0.0, 0.0, turnAmount);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Actual Heading", getHeading());
            telemetry.addData("Heading Error", error);
            telemetry.update();

            idle();
        }

        stopMotors();
    }

    private void stopMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
