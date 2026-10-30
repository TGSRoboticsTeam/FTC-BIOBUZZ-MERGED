
package org.firstinspires.ftc.teamcode.Auto.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Auto.Important.Constants;

@TeleOp(name = "Swerve Turn Tuner", group = "Pedro Pathing")
@Disabled
public class SwerveTurnTuner extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        telemetry.addLine("SWERVE TURN TEST");
        telemetry.addLine(
                "Checks steering and drive motor directions."
        );
        telemetry.addLine(
                "Securely lift the robot before pressing START."
        );
        telemetry.addLine(
                "Press STOP immediately if anything behaves unexpectedly."
        );
        telemetry.update();
    }

    @Override
    public void loop() {
        // Command rotation only, with no forward or strafe input.
        follower.manual(0, 0, 0.25);
        follower.update();

        telemetry.addLine("Swerve Turn Test is running");
        telemetry.addLine("Turn command: 0.25");
        telemetry.addLine("Press STOP to end the test.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.manual(0, 0, 0);
            follower.update();
        }
    }
}
