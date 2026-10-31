import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
// Import FTC and Pedro Pathing classes
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// Register the autonomous program
@Autonomous(name = "Pedro Path Test", group = "Pedro")
public class PedroPathTest extends OpMode {

    // Controls path following
    private Follower follower;

    // Creates poses using degree headings
    private final PoseFactory p = PoseFactory.degrees();

    // Set the robot's starting position and heading
    private final Pose start = p.of(24, 24, 0);

    // Set the straight-line target
    private final Pose forward = p.of(24, 36, 0);

    // Set a target with a 90-degree heading
    private final Pose turn90 = p.of(36, 36, 90);

    // Set more targets for the square route
    private final Pose square3 = p.of(36, 24, 180);
    private final Pose square4 = p.of(24, 24, 270);

    // Set the curve's control point and endpoint
    private final Pose curveControl = p.of(40, 48, 45);
    private final Pose curveEnd = p.of(48, 36, 90);

    // Store the paths
    private Path straightPath;
    private Path turnPath;
    private Path squarePath1;
    private Path squarePath2;
    private Path curvePath;

    // Track the current autonomous step
    private int state = 0;

    // Track the simulated shooting pause
    private long pauseStart = 0;
    private static final long SHOOT_PAUSE_MS = 1500;

    @Override
    public void init() {
        // Initialize Pedro Pathing
        follower = Constants.create(hardwareMap);

        // Set the starting pose
        follower.setPose(start);

        // Create the straight path
        straightPath = line(start, forward).linear(start, forward);

        // Create the path ending at 90 degrees
        turnPath = line(forward, turn90).linear(forward, turn90);

        // Create square-route segments
        squarePath1 = line(turn90, square3).linear(turn90, square3);
        squarePath2 = line(square3, square4).linear(square3, square4);

        // Create the curved path
        curvePath = curve(square4, curveControl, curveEnd).linear(square4, curveEnd);

        // Display initialization status
        telemetry.addLine("Pedro Path Test initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        // Start the first path
        state = 1;
        follower.follow(straightPath);
    }

    @Override
    public void loop() {
        // Update path following
        follower.update();

        // Start the next action when the current path finishes
        if (!follower.isBusy()) {
            switch (state) {
                case 1:
                    state = 2;
                    follower.follow(turnPath);
                    break;
                case 2:
                    state = 3;
                    follower.follow(squarePath1);
                    break;
                case 3:
                    state = 4;
                    follower.follow(squarePath2);
                    break;
                case 4:
                    // Start the simulated shooting pause
                    state = 5;
                    pauseStart = System.currentTimeMillis();
                    break;
                case 5:
                    // Wait 1.5 seconds without blocking the loop
                    if (System.currentTimeMillis() - pauseStart >= SHOOT_PAUSE_MS) {
                        state = 6;
                        follower.follow(curvePath);
                    }
                    break;
                case 6:
                    // Stop after the final path
                    state = 7;
                    follower.stop();
                    break;
            }
        }

        // Get the robot's estimated position
        Pose actual = follower.pose();

        // Display actual position and heading
        telemetry.addData("State", state);
        telemetry.addData("Actual X", actual.x());
        telemetry.addData("Actual Y", actual.y());
        telemetry.addData("Actual Heading", Math.toDegrees(actual.heading()));

        // Select the expected target for the current step
        Pose target = start;
        switch (state) {
            case 1:
                target = forward;
                break;
            case 2:
                target = turn90;
                break;
            case 3:
                target = square3;
                break;
            case 4:
            case 5:
                target = square4;
                break;
            case 6:
                target = curveEnd;
                break;
        }

        // Display the target position and heading
        telemetry.addData("Target X", target.x());
        telemetry.addData("Target Y", target.y());
        telemetry.addData("Target Heading", Math.toDegrees(target.heading()));

        // Send telemetry to the Driver Station
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop path following when the OpMode ends
        if (follower != null) {
            follower.stop();
        }
    }
}
