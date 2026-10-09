package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class drivetrain extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private IMU imu;

    private static final double DEADZONE = 0.05;
    private static final double SLOW_MODE_MULTIPLIER = 0.35;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        imu = hardwareMap.get(IMU.class, "imu");

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontRightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontLeftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        frontRightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        backLeftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        backRightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        frontLeftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        frontRightMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        backLeftMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        backRightMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                );

        imu.initialize(new IMU.Parameters(orientation));

        telemetry.addLine("FIELD CENTRIC READY");
        telemetry.addLine("Left Stick = Drive / Strafe");
        telemetry.addLine("Right Stick X = Rotate");
        telemetry.addLine("OPTIONS = Reset Yaw");
        telemetry.addLine("Hold LEFT BUMPER = Slow Mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        imu.resetYaw();

        while (opModeIsActive()) {

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (Math.abs(y) < DEADZONE) {
                y = 0;
            }

            if (Math.abs(x) < DEADZONE) {
                x = 0;
            }

            if (Math.abs(rx) < DEADZONE) {
                rx = 0;
            }

            double botHeading =
                    imu.getRobotYawPitchRollAngles()
                            .getYaw(AngleUnit.RADIANS);

            double rotX =
                    x * Math.cos(-botHeading)
                            - y * Math.sin(-botHeading);

            double rotY =
                    x * Math.sin(-botHeading)
                            + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator =
                    Math.max(
                            Math.abs(rotY)
                                    + Math.abs(rotX)
                                    + Math.abs(rx),
                            1.0
                    );

            double frontLeftPower =
                    (rotY + rotX + rx) / denominator;

            double backLeftPower =
                    (rotY - rotX + rx) / denominator;

            double frontRightPower =
                    (rotY - rotX - rx) / denominator;

            double backRightPower =
                    (rotY + rotX - rx) / denominator;

            boolean slowMode = gamepad1.left_bumper;

            if (slowMode) {
                frontLeftPower *= SLOW_MODE_MULTIPLIER;
                frontRightPower *= SLOW_MODE_MULTIPLIER;
                backLeftPower *= SLOW_MODE_MULTIPLIER;
                backRightPower *= SLOW_MODE_MULTIPLIER;
            }

            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);

            double headingDegrees =
                    imu.getRobotYawPitchRollAngles()
                            .getYaw(AngleUnit.DEGREES);

            telemetry.addLine(" IMU ");
            telemetry.addData(
                    "Heading",
                    "%.2f degrees",
                    headingDegrees
            );

            telemetry.addLine();
            telemetry.addLine(" ENCODERS ");

            telemetry.addData(
                    "Front Left",
                    frontLeftMotor.getCurrentPosition()
            );

            telemetry.addData(
                    "Front Right",
                    frontRightMotor.getCurrentPosition()
            );

            telemetry.addData(
                    "Back Left",
                    backLeftMotor.getCurrentPosition()
            );

            telemetry.addData(
                    "Back Right",
                    backRightMotor.getCurrentPosition()
            );

            telemetry.addLine();
            telemetry.addLine("===== JOYSTICKS =====");

            telemetry.addData(
                    "Forward / Back",
                    "%.2f",
                    y
            );

            telemetry.addData(
                    "Strafe",
                    "%.2f",
                    x
            );

            telemetry.addData(
                    "Rotation",
                    "%.2f",
                    rx
            );

            telemetry.addLine();
            telemetry.addLine("===== MOTOR POWER =====");

            telemetry.addData(
                    "FL Power",
                    "%.2f",
                    frontLeftPower
            );

            telemetry.addData(
                    "FR Power",
                    "%.2f",
                    frontRightPower
            );

            telemetry.addData(
                    "BL Power",
                    "%.2f",
                    backLeftPower
            );

            telemetry.addData(
                    "BR Power",
                    "%.2f",
                    backRightPower
            );

            telemetry.addLine();
            telemetry.addLine("===== STATUS =====");

            telemetry.addData(
                    "Slow Mode",
                    slowMode ? "ON" : "OFF"
            );

            telemetry.addLine("OPTIONS = Reset Heading");

            telemetry.update();
        }
    }
}
