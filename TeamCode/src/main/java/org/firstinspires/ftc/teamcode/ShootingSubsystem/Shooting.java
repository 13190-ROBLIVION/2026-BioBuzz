package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooting {
    private DcMotorEx shootingMotor;

    /**
     * Initialize the motors for shooting
     * I think this should be done at first start
     */
    public void initShooting() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");

        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized shooting stuff!");
        telemetry.update();
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
    }

    /**
     * Shoot with a timer for how long the motor spins.
     * @param shotPower
     * @param activationTime
     */
    public void shoot(double shotPower, double activationTime) {
        shootingMotor.setPower(shotPower);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activationTime >= timer.milliseconds()) {
            linearOpMode.idle();    // I think this idles it.
        }

        stopShootingMotors();
    }

    public void shoot(double targetRPM, double activationTime) {
        shootingMotor.setPower(shotPower);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (activationTime >= timer.milliseconds()) {
            linearOpMode.idle();    // I think this idles it.
        }

        stopShootingMotors();
    }

    // Optional: stop the motors
    public void stopShootingMotors() {
        shootingMotor.setPower(0);

        telemetry.addData("Set motor 1 power to: ", shootingMotor.getPower());
        telemetry.update();
    }

    private double RPMtoTPS(double RPM, double countsPerRev){
        return ((RPM/60) * countsPerRev);
    }
}