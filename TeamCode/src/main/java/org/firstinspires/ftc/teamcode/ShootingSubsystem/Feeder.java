package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Feeder {
    DcMotor feederMotor;
    private boolean feedingCycleRunning;
    private final int feedTimeMillis=1000;

    /**
     * Intake Motor Initialization
     */
    public void initFeeder() {
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized feeder.");
        telemetry.update();
    }

    public void startFeeder(double spd) {
        feederMotor.setPower(spd);
        telemetry.addData("Set feeder speed to: ",spd);
    }

    public void feedOnce(double spd) {
        if (feedingCycleRunning) {
            telemetry.addLine("[!] feeding cycle already running");
        }
        feedingCycleRunning=true;
        startFeeder(spd);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (feedTimeMillis >= timer.milliseconds()) {
            linearOpMode.idle();
        }
        stopFeeder();
        feedingCycleRunning=false;
    }


    public void stopFeeder() {
        feederMotor.setPower(0);

        telemetry.addLine("Stopped feeder motor.");
    }
}

