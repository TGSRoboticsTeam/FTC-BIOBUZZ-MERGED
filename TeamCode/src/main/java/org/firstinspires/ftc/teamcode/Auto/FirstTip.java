package org.firstinspires.ftc.teamcode.Auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Important.Constants;

@Autonomous(name = "FirstTip", group = "Autonomous")
public class FirstTip extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.9603, 133.2113, 270);
    private final Pose path1 = poseFactory.of(112.7472, 110.9655, 270);
    private final Pose point2 = poseFactory.of(131.0271, 45.7004, 270);

    // Launcher motor declarations
    private DcMotor launcher1;
    private DcMotor launcher2;
    private DcMotor intake1;
    private DcMotor intake2;

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        // Hardware Map for the two launcher motors
        launcher1 = hardwareMap.get(DcMotor.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotor.class, "launcher2");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");


        waitForStart();

        // 1. Turn on motors
        launcher1.setPower(0.93);
        launcher2.setPower(0.93);
        intake1.setPower(0.4);
        intake2.setPower(0.4);


        // 2. Wait to shoot (2000 ms = 2 seconds)
        sleep(4000);

        // 3. Turn off motors
        launcher1.setPower(0.0);
        launcher2.setPower(0.0);

        // 4. Start movement
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }

        // Failsafe: ensure motors are off if OpMode stops early
        launcher1.setPower(0.0);
        launcher2.setPower(0.0);
    }

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }
}