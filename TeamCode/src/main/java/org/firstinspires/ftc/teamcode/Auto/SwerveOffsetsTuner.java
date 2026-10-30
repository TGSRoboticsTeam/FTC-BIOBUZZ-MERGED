
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;

@TeleOp(name = "Swerve Offsets Tuner", group = "Pedro Pathing")
public class SwerveOffsetsTuner extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        telemetry.addLine("Swerve Offsets Test");
        telemetry.addLine("Check that all modules are clear.");
        telemetry.addLine("Press START to begin.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.manual(0.15, 0, 0);
        follower.update();

        telemetry.addData("LF power",
                follower.drivetrain.debug().get("leftFront"));
        telemetry.addData("RF power",
                follower.drivetrain.debug().get("rightFront"));
        telemetry.addData("LB power",
                follower.drivetrain.debug().get("leftBack"));
        telemetry.addData("RB power",
                follower.drivetrain.debug().get("rightBack"));

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
