
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.Auto.Important.Constants;

import java.util.Map;

@TeleOp(name = "Swerve Localization Test", group = "Pedro Pathing")
@Disabled
public class LocalizationTest extends OpMode {

    private Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        telemetry.addLine("SWERVE LOCALIZATION TEST");
        telemetry.addLine("No movement commands are issued.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        // Get the current debug information.
        Map<String, Object> drivetrainData =
                follower.debug().drivetrain();

        Map<String, Object> localizerData =
                follower.debug().localizer();

        // Rebuild the telemetry display once per loop.
        telemetry.clearAll();

        telemetry.addLine("=== DRIVETRAIN DEBUG ===");
        addEntries("Drive", drivetrainData);

        telemetry.addLine("=== LOCALIZER DEBUG ===");
        addEntries("Localizer", localizerData);

        // Three blank lines separating the debug data and live readout.
        telemetry.addLine("");
        telemetry.addLine("");
        telemetry.addLine("");

        telemetry.addLine("=== LIVE ODOMETRY READOUT ===");
        addEntries("Live", localizerData);

        telemetry.update();
    }

    private void addEntries(
            String prefix,
            Map<String, Object> data) {

        if (data == null || data.isEmpty()) {
            telemetry.addData(prefix, "No data available");
            return;
        }

        for (Map.Entry<String, Object> entry : data.entrySet()) {
            telemetry.addData(
                    prefix + " | " + entry.getKey(),
                    "%s",
                    String.valueOf(entry.getValue())
            );
        }
    }

    @Override
    public void stop() {
        // No movement commands are issued by this OpMode.
    }
}
