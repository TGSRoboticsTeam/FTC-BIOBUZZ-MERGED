
package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.line;

@Autonomous(name = "Swerve Test Auto", group = "Pedro Pathing")
public class testAuto extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(0, 0, 0);
    private final Pose forwardEnd = poseFactory.of(16, 0, 0);
    private final Pose reverseEnd = poseFactory.of(8, 0, 0);

    private Path forwardPath;
    private Path reversePath;

    private static final double PAUSE_SECONDS = 3.0;
    private static final double TURN_TIMEOUT_SECONDS = 10.0;
    private static final double TURN_POWER = 0.30;
    private static final double FULL_TURN = 2.0 * Math.PI;

    private enum State {
        FORWARD,
        PAUSE_AFTER_FORWARD,
        REVERSE,
        PAUSE_AFTER_REVERSE,
        ROTATE,
        PAUSE_AFTER_ROTATE,
        COMPLETE,
        TURN_TIMEOUT
    }

    private State state = State.COMPLETE;

    private final ElapsedTime pauseTimer = new ElapsedTime();
    private final ElapsedTime turnTimer = new ElapsedTime();

    private double previousHeading;
    private double accumulatedTurn;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        forwardPath = line(startPose, forwardEnd)
                .linear(startPose, forwardEnd);

        reversePath = line(forwardEnd, reverseEnd)
                .linear(forwardEnd, reverseEnd);

        telemetry.addLine("SWERVE TEST AUTO");
        telemetry.addLine("Forward 12 inches");
        telemetry.addLine("Pause 10 seconds");
        telemetry.addLine("Reverse 6 inches");
        telemetry.addLine("Pause 10 seconds");
        telemetry.addLine("Rotate 360 degrees");
        telemetry.addLine("Pause 10 seconds");
        telemetry.addLine("");
        telemetry.addLine("Watch the pause countdown.");
        telemetry.addLine("Press STOP if the robot behaves unexpectedly.");
        telemetry.update();
    }

    @Override
    public void start() {
        state = State.FORWARD;
        follower.follow(forwardPath);
    }

    @Override
    public void loop() {

        switch (state) {

            case FORWARD:
                follower.update();

                if (!follower.isBusy()) {
                    beginPause(State.PAUSE_AFTER_FORWARD);
                }
                break;

            case PAUSE_AFTER_FORWARD:
                runPause();

                if (pauseTimer.seconds() >= PAUSE_SECONDS) {
                    follower.follow(reversePath);
                    state = State.REVERSE;
                }
                break;

            case REVERSE:
                follower.update();

                if (!follower.isBusy()) {
                    beginPause(State.PAUSE_AFTER_REVERSE);
                }
                break;

            case PAUSE_AFTER_REVERSE:
                runPause();

                if (pauseTimer.seconds() >= PAUSE_SECONDS) {
                    previousHeading = follower.pose().heading();
                    accumulatedTurn = 0.0;
                    turnTimer.reset();
                    state = State.ROTATE;
                }
                break;

            case ROTATE:
                follower.manual(0, 0, TURN_POWER);
                follower.update();

                double heading = follower.pose().heading();
                double delta = normalizeAngle(heading - previousHeading);

                accumulatedTurn += delta;
                previousHeading = heading;

                if (Math.abs(accumulatedTurn) >= FULL_TURN) {
                    beginPause(State.PAUSE_AFTER_ROTATE);

                } else if (turnTimer.seconds() >= TURN_TIMEOUT_SECONDS) {
                    commandZeroMovement();
                    state = State.TURN_TIMEOUT;
                }
                break;

            case PAUSE_AFTER_ROTATE:
                runPause();

                if (pauseTimer.seconds() >= PAUSE_SECONDS) {
                    commandZeroMovement();
                    state = State.COMPLETE;
                }
                break;

            case COMPLETE:
            case TURN_TIMEOUT:
                commandZeroMovement();
                break;
        }

        updateTelemetry();
    }

    /*
     * Enter a pause once.
     * The timer is reset only here, never on each loop.
     */
    private void beginPause(State pauseState) {
        commandZeroMovement();
        pauseTimer.reset();
        state = pauseState;
    }

    /*
     * Explicitly command zero translation and rotation,
     * then update Pedro to apply the command.
     */
    private void commandZeroMovement() {
        follower.manual(0, 0, 0);
        follower.update();
    }

    /*
     * This method runs continuously during the pause.
     * The next action cannot start until the timer expires.
     */
    private void runPause() {
        commandZeroMovement();
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    private void updateTelemetry() {
        Pose pose = follower.pose();

        telemetry.addLine("=== SWERVE AUTO ===");
        telemetry.addData("State", state);
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData(
                "Heading (deg)",
                "%.1f",
                Math.toDegrees(pose.heading())
        );
        telemetry.addData(
                "Accumulated turn (deg)",
                "%.1f",
                Math.toDegrees(accumulatedTurn)
        );
        telemetry.addData("Follower mode", follower.mode());

        if (isPauseState()) {
            double remaining = Math.max(
                    0.0,
                    PAUSE_SECONDS - pauseTimer.seconds()
            );

            telemetry.addLine("");
            telemetry.addLine("************************");
            telemetry.addLine("        PAUSED");
            telemetry.addLine("    ZERO MOVEMENT");
            telemetry.addData("Seconds remaining", "%.1f", remaining);
            telemetry.addLine("************************");
        }

        if (state == State.COMPLETE) {
            telemetry.addLine("TEST COMPLETE");
        }

        if (state == State.TURN_TIMEOUT) {
            telemetry.addLine("ROTATION TIMED OUT");
        }

        telemetry.update();
    }

    private boolean isPauseState() {
        return state == State.PAUSE_AFTER_FORWARD
                || state == State.PAUSE_AFTER_REVERSE
                || state == State.PAUSE_AFTER_ROTATE;
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
    }
}
