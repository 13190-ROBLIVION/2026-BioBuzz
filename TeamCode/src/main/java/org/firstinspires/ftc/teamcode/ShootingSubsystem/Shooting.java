package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooting {
    public Shooting() {
        shootingMotor = hardwareMap.get(DcMotorEx.class, "shootingMotor");

        shootingMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shootingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootingMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized shooting stuff!");
        telemetry.update();
    }
        
    
    private DcMotorEx shootingMotor;

    // Random Constants
    private static class constants {
        static final double idleRPM = 0;
        static final double shootingMotorCPR = 28;
    }

    private boolean isShooting = false;

    private void whileShooting() {
        isShooting = true;

        while(isShooting) {
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
            linearOpMode.idle();    // I think this idles it.
        }

        stopShootingMotors();

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
            telemetry.addData("Current shooting motor RPM is: ", TPStoRPM(shootingMotor.getVelocity(), constants.shootingMotorCPR));

            linearOpMode.idle();    // I think this idles it.
        }

        stopShootingMotors();

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

    // Optional: stop the motors
    public void stopShootingMotors() {
        shootingMotor.setPower(0);
        shootingMotor.setVelocity(constants.idleRPM);

        telemetry.addData("Stopped motor. Speed is now: ", shootingMotor.getPower());
        telemetry.update();

        return;
    }

    private double RPMtoTPS(double RPM, double countsPerRev) { return ((RPM/60) * countsPerRev); }
    private double TPStoRPM(double TPS, double countsPerRev) { return ((TPS / countsPerRev) * 60); }

    public boolean isShootingReady() { return true; }
}
