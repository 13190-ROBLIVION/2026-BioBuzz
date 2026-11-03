
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class intakeTest extends LinearOpMode {

    private DcMotor intakeMotor1;
    private DcMotor intakeMotor2;

    @Override
    public void runOpMode() {

        intakeMotor1 = hardwareMap.get(DcMotor.class, "intakeMotor1");
        intakeMotor2 = hardwareMap.get(DcMotor.class, "intakeMotor2");

        intakeMotor1.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor2.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            double power1 = 0;
            double power2 = 0;

            if (gamepad1.a) {
                power1 = 0.4;
                power2 = 0.4;

            } else if (gamepad1.b) {
                power1 = -0.4;
                power2 = -0.4;

            } else if (gamepad1.x) {
                power1 = 0.4;

            } else if (gamepad1.y) {
                power2 = 0.4;
            }

            intakeMotor1.setPower(power1);
            intakeMotor2.setPower(power2);

            telemetry.addData("Motor 1 Power", power1);
            telemetry.addData("Motor 2 Power", power2);

            telemetry.addData(
                    "Motor 1 Encoder",
                    intakeMotor1.getCurrentPosition()
            );

            telemetry.addData(
                    "Motor 2 Encoder",
                    intakeMotor2.getCurrentPosition()
            );

            telemetry.update();
        }

        intakeMotor1.setPower(0);
        intakeMotor2.setPower(0);
    }
}