package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IntakeTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        waitForStart();

        telemetry.addData("Test", 1);
        telemetry.update();
    }
}
