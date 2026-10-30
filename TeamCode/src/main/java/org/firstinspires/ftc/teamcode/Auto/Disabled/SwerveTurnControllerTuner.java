
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Auto.Important.Constants;

@TeleOp(name = "Swerve Turn Controller Tuner", group = "Tuning")
@Disabled
public class SwerveTurnControllerTuner extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Use your existing, working follower setup.
        Follower follower = Constants.create(hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        // Movement power: start low to make testing easier to observe.
        final double DRIVE_POWER = 0.15;
        final double STRAFE_POWER = 0.15;
        final double TURN_POWER = 0.15;

        // Duration of each movement, in seconds.
        final double MOVE_TIME = 1.5;
        final double PAUSE_TIME = 0.75;

        telemetry.addLine("Swerve Turn Controller Tuner");
        telemetry.addLine("Forward -> Strafe -> Turn -> Repeat");
        telemetry.addLine("Clear the area before starting.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Phases: forward, pause, strafe, pause, turn, pause.
        int phase = 0;
        timer.reset();

        while (opModeIsActive()) {

            double duration = (phase % 2 == 0)
                    ? MOVE_TIME : PAUSE_TIME;

            // Advance to the next phase when this one finishes.
            if (timer.seconds() >= duration) {
                phase = (phase + 1) % 6;
                timer.reset();
            }

            // Send the movement command for the current phase.
            switch (phase) {

                case 0:
                    // Drive forward: observe how quickly all pods
                    // steer to their forward-facing angles.
                    follower.manual(DRIVE_POWER, 0, 0);
                    telemetry.addLine("FORWARD");
                    break;

                case 1:
                    // Pause so the robot can settle before the
                    // next movement.
                    follower.manual(0, 0, 0);
                    telemetry.addLine("PAUSE");
                    break;

                case 2:
                    // Strafe: observe the pods changing angle
                    // to move sideways.
                    follower.manual(0, STRAFE_POWER, 0);
                    telemetry.addLine("STRAFE");
                    break;

                case 3:
                    follower.manual(0, 0, 0);
                    telemetry.addLine("PAUSE");
                    break;

                case 4:
                    // Rotate in place: the pods must continuously
                    // adjust to the angles needed for rotation.
                    follower.manual(0, 0, TURN_POWER);
                    telemetry.addLine("TURN");
                    break;

                case 5:
                    follower.manual(0, 0, 0);
                    telemetry.addLine("PAUSE");
                    break;
            }

            // Update the follower every loop.
            follower.update();

            telemetry.addData("Phase", phase);
            telemetry.addData("Phase time", "%.2f", timer.seconds());
            telemetry.update();
        }

        // Stop commanding movement when the OpMode ends.
        follower.manual(0, 0, 0);
        follower.update();
    }
}
