
package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FlywheelMaxVelocityTest", group = "Tools")
public class FlywheelMaxVelocityTest extends LinearOpMode {

    private DcMotorEx launcher1, launcher2;

    @Override
    public void runOpMode() {

        launcher1 = hardwareMap.get(DcMotorEx.class, "launcher1");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setDirection(DcMotorSimple.Direction.FORWARD);
        launcher2.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Press PLAY to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        launcher1.setPower(1.0);
        launcher2.setPower(1.0);

        double maxV1 = 0.0;
        double maxV2 = 0.0;

        while (opModeIsActive()) {

            double velocity1 = Math.abs(launcher1.getVelocity());
            double velocity2 = Math.abs(launcher2.getVelocity());

            maxV1 = Math.max(maxV1, velocity1);
            maxV2 = Math.max(maxV2, velocity2);

            telemetry.addData("--- LAUNCHER 1 ---", "");
            telemetry.addData("Current Velocity", velocity1);
            telemetry.addData("Maximum Velocity", maxV1);

            telemetry.addData("--- LAUNCHER 2 ---", "");
            telemetry.addData("Current Velocity", velocity2);
            telemetry.addData("Maximum Velocity", maxV2);

            telemetry.update();

            idle();
        }

        launcher1.setPower(0);
        launcher2.setPower(0);
    }
}
