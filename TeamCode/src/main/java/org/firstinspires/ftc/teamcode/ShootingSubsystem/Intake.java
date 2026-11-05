package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private Telemetry telemetry;
    DcMotor intakeMotor;

    /**
     * Intake Motor Initialization
     */
    public void initIntake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // TODO: Reverse it if needed
        // shootingMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized intake.");
        telemetry.update();
    }

    private double RPMtoTPS(double RPM, double countsPerRev) {
        return (RPM / 60) * countsPerRev;
    }

    private double TPStoRPM(double TPS, double countsPerRev) {
        return (TPS / countsPerRev) * 60;
    }

    /**
     * Sets intake motor to a speed.
     * @param spd speed to set to
     */
    public void intakeGo(double spd) {
        intakeMotor.setPower(spd);

        telemetry.addData("Set intake power: ", intakeMotor.getPower());
        telemetry.addLine("Run until stop");
        telemetry.update();
    }

    public void startIntake(double spd) {
        intakeGo(Math.abs(spd));
    }

    public void reverseIntake(double spd) {
        intakeGo(-Math.abs(spd));
    }

    /**
     * Stops intake motors. Simple right?
     */
    public void stopIntake() {
        intakeMotor.setPower(0);

        telemetry.addLine("Stopped intake motor");
        telemetry.update();
    }

    /**
     * Sets intake motor to a speed for a set time.
     * @param opMode caller LinearOpMode context
     * @param spd speed to set to
     * @param time time to run motor in millis
     */
    public void intakeGo(LinearOpMode opMode, double spd, double time) {
        intakeMotor.setPower(spd);

        telemetry.addData("Running intake for millis: ", time);
        telemetry.update();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (time >= timer.milliseconds() && opMode.opModeIsActive()) {
            opMode.idle();
        }

        stopIntake();
    }
}
