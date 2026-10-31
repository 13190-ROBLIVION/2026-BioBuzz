package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "BioBuzz Pedro Test")
public class PedroTest extends OpMode {

    private Follower follower;

    private int testNumber = 1;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    @Override
    public void init() {
        // Create Pedro follower
        follower = Constants.create(hardwareMap);

        telemetry.addLine("BioBuzz Pedro Test");
        telemetry.addLine("Choose a test with D-pad");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Select one movement test
        if (gamepad1.dpad_right && !lastRight) {
            testNumber = Math.min(4, testNumber + 1);
        }

        if (gamepad1.dpad_left && !lastLeft) {
            testNumber = Math.max(1, testNumber - 1);
        }

        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        telemetry.addData("Selected Test", testNumber);
        telemetry.addLine("1: Forward 24 inches");
        telemetry.addLine("2: Strafe 24 inches");
        telemetry.addLine("3: Turn 90 degrees");
        telemetry.addLine("4: Bezier curve");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Keep localization updated
        follower.update();

        // Display robot position
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));

        telemetry.addData("Selected Test", testNumber);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop following when interrupted
        follower.stop();
    }
}
