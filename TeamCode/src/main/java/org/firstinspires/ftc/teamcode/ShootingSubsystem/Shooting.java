package org.firstinspires.ftc.teamcode.ShootingSubsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Shooting {

    // Inner class to hold the shoot data
    public static class ShootData {
        public DcMotor motor1;
        public DcMotor motor2;
        public double motor1Power;    // TODO: Change to top and bottom motor or left and rightr motor
        public double motor2Power;
        public double activationTime;
        public double deltaTime;

        public ShootData(DcMotor motor1, DcMotor motor2,
                         double motor1Power, double motor2Power,
                         double activationTime, double deltaTime) {
            this.motor1 = motor1;
            this.motor2 = motor2;
            this.motor1Power = motor1Power;
            this.motor2Power = motor2Power;
            this.activationTime = activationTime = -1;
            this.deltaTime = deltaTime = -1;
        }
    }

    /**
     * Initialize the motors for shooting
     * I think this should be done at first start
     */
    public void initShooting(DcMotor motor1, DcMotor motor2) {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");    // TODO: Change to top and bottom motor or left and right motor (USE AI)
        motor2 = hardwareMap.get(DcMotor.class, "motor2");    // TODO: Change to top and bottom motor or left and right motor

        // TODO: Reverse it if needed
        // motor1.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized Shooting!");
        telemetry.update();
    }

    /**
     * Just set the motors to a power.
     * You need to stop them later.
     * @param data
     */
    public void shoot(ShootData data) {
        data.motor1.setPower(data.motor1Power);
        data.motor2.setPower(data.motor2Power);

        // If a timing is specified
        if (data.deltaTime >= 0 && data.activationTime >= 0) {
            double timeSpent = 0;
            while (data.activationTime >= timeSpent) {
                timeSpent += data.deltaTime;
            }

            stopShootingMotors(data);
        }
        else {
            telemetry.addData("Set motor 1 power to: ", data.motor1.getPower());    // TODO: Update names
            telemetry.addData("Set motor 2 power to: ", data.motor2.getPower());    // TODO: Update names
            telemetry.addLine("Running until stopped!");
            telemetry.update();
        }
    }

    // Optional: stop the motors
    public void stopShootingMotors(ShootData data) {
        data.motor1.setPower(0);
        data.motor2.setPower(0);

        telemetry.addData("Set motor 1 power to: ", data.motor1.getPower());    // TODO: Update names
        telemetry.addData("Set motor 2 power to: ", data.motor2.getPower());    // TODO: Update names
        telemetry.update();
    }
}