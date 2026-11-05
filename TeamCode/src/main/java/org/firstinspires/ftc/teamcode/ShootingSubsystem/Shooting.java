package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Shooting {
    // Load libftcrobotcontroller.so (built from TeamCode/src/main/cpp via CMake/NDK).
    // Runs once, when the Shooting class is first loaded.
    static {
        System.loadLibrary("ftcrobotcontroller");
    }

    public Shooting() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        shootingMotor.setVelocity(RPMtoTPS(constants.idleRPM, constants.shootingMotorCPR));

        // Grab the OTOS if the robot has one. Drivetrain owns calibration and
        // tracking reset — we only ever READ velocity from here.
        otos = hardwareMap.tryGet(SparkFunOTOS.class, "sensor_otos");

        if (otos != null) {
            // Set units ourselves so the still-check is correct even if
            // Drivetrain never initialized the sensor (e.g. in an auto).
            otos.setLinearUnit(DistanceUnit.INCH);
            otos.setAngularUnit(AngleUnit.DEGREES);
        }

        telemetry.addLine("Initialized shooting stuff!");
        telemetry.update();
    }

    private DcMotorEx shootingMotor;
    private SparkFunOTOS otos;

    // Random Constants
    private static class constants {
        static final double idleRPM = 0;
        static final double shootingMotorCPR = 28;

        // Same buffers as Drivetrain's still-check (keep in sync).
        static final double stillSpeedBuffer = 0.5; // in/s
        static final double stillTurnBuffer = 2.0; // deg/s
    }

    private boolean isShooting = false;

    private void whileShooting() {
        isShooting = true;

        while (isShooting) {
            telemetry.addData("Current speed is: ", TPStoRPM(shootingMotor.getVelocity(), constants.shootingMotorCPR));
            linearOpMode.idle();
        }
    }

    /**
     * Just set the motors to a power.
     * You need to stop them later.
     * @param shotPower
     */
    public void shoot(double shotPower) {
        shootingMotor.setPower(shotPower);

        telemetry.addData("Set shooting motor power to: ", shootingMotor.getPower());
        telemetry.addLine("Running until stopped!");
        telemetry.update();

        whileShooting();

        return;
    }

    /**
     * Shoot with a timer for how long the motor spins based on power.
     * @param shotPower
     * @param activationTime
     */
    public void shoot(double shotPower, double activationTime) {
        shootingMotor.setPower(shotPower);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activationTime >= timer.milliseconds()) {
            linearOpMode.idle(); // I think this idles it.
        }

        resetShootingMotors();

        return;
    }

    /**
     * Shoot for a certain amount of time based on RPM instead of power.
     * @param targetRPM
     * @param activationTime
     */
    public void shootRPM(double targetRPM, double activationTime) {
        shootingMotor.setVelocity(RPMtoTPS(targetRPM, constants.shootingMotorCPR));

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activationTime >= timer.milliseconds()) {
            telemetry.addData(
                "Current shooting motor RPM is: ",
                TPStoRPM(shootingMotor.getVelocity(), constants.shootingMotorCPR)
            );

            linearOpMode.idle(); // I think this idles it.
        }

        resetShootingMotors();

        return;
    }

    /**
     * Shoot based on RPM.
     * You need to stop it later.
     * @param targetRPM
     */
    public void shootRPM(double targetRPM) {
        shootingMotor.setVelocity(RPMtoTPS(targetRPM, constants.shootingMotorCPR));

        whileShooting();

        return;
    }

    // Reset the motors back to idle
    public void resetShootingMotors() {
        shootingMotor.setVelocity(constants.idleRPM);

        telemetry.addData(
            "Reset motor. Speed is now: ",
            TPStoRPM(shootingMotor.getVelocity(), constants.shootingMotorCPR)
        );
        telemetry.update();

        return;
    }

    public void stopShootingMotors() {
        shootingMotor.setPower(0);
        shootingMotor.setVelocity(0);

        telemetry.addData(
            "Stopped motor. Speed is now: ",
            TPStoRPM(shootingMotor.getVelocity(), constants.shootingMotorCPR)
        );
        telemetry.update();

        return;
    }

    /**
     * True when the robot is (nearly) stationary — not driving AND not rotating.
     * Reads the OTOS straight from the HardwareMap. In IMU fallback mode (no
     * OTOS) there is no linear velocity source, so we conservatively report
     * "moving" (returns false).
     */
    private boolean isBotStill() {
        if (otos == null) {
            return false; // safe default: assume moving when we can't measure
        }

        SparkFunOTOS.Pose2D vel = otos.getVelocity();

        double speedInPerSec = Math.hypot(vel.x, vel.y);
        double turnDegPerSec = Math.abs(vel.h);

        return speedInPerSec < constants.stillSpeedBuffer && turnDegPerSec < constants.stillTurnBuffer;
    }

    public double getShootingMotorVelocity() {
        return shootingMotor.getVelocity();
    }

    // Implemented in C++ (src/main/cpp), exposed through the JNI bridge in ftcrobotcontroller.cpp.
    public static native double RPMtoTPS(double RPM, double countsPerRev);

    public static native double TPStoRPM(double TPS, double countsPerRev);

    public boolean isShootingReady() {
        return isBotStill();
    }
}
