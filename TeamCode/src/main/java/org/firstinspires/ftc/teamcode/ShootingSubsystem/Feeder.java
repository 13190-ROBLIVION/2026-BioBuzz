package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Feeder {
    DcMotor feederMotor;

    /**
     * Intake Motor Initialization
     */
    public void initIntake() {
        feederMotor = hardwareMap.get(DcMotor.class, "feederMotor");

        feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized feeder.");
        telemetry.update();
    }

    public void feedOnce() {

    }

    public void stopFeeder() {
        feederMotor.setPower(0);
    }
}

