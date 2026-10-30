package org.firstinspires.ftc.teamcode.Auto.Important;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.Auto.Procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.Auto.Procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.Auto.Procedures.Tests;

public class Tuning {
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }


    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap ->
                        new PinpointLocalizer(
                                hardwareMap,
                                Constants.localizerConfig
                        ),
                hardwareMap ->
                        Constants.createDrivetrain(hardwareMap)
        );
    }

    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> Constants.createDrivetrain(hardwareMap),
                hardwareMap -> new PinpointLocalizer(
                        hardwareMap,
                        Constants.localizerConfig
                ),
                () -> new Foresight(Constants.foresightConfig)
        );
    }

}
