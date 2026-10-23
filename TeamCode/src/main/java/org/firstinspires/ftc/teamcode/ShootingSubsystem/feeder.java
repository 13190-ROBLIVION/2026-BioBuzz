package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class feeder {
    DcMotor feederMotor;

    /**
     * feeder Motor Initialization
     */
    public void initfeeder() {
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized feeder.");
        telemetry.update();
    }

    private double RPMtoTPS(double RPM, double countsPerRev){
        return ((RPM/60) * countsPerRev);
    }

    private double TPStoRPM(double TPS, double countsPerRev){
        return ((TPS/countsPerRev) * 60);
    }

    /**
     * Sets feeder motor to a speed.
     * @param spd speed to set to
     */
    public void feederGo(double spd) {
        feederMotor.setPower(spd);

        telemetry.addData("Set feeder power: ", feederMotor.getPower());
        telemetry.addLine("Run until stop");
        telemetry.update();
    }

    /**
     * Stops feeder motors. Simple right?
     */
    public void stopfeeder() {
        feederMotor.setPower(0);

        telemetry.addLine("Stopped feeder motor");
        telemetry.update();
    }

    /**
     * Sets feeder motor to a speed for a set time.
     * @param spd speed to set to
     * @param time time to run motor in millis
     */
    public void feederGo(double spd, double time) {
        feederMotor.setPower(spd);

        telemetry.addData("Running feeder for millis: ", time);
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (time >= timer.milliseconds()) {
            linearOpMode.idle();
        }

        stopfeeder();
    }
}

