package org.firstinspires.ftc.teamcode.Auto.Important;








import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.CoaxialPod;
import com.pedropathing.revhub.drivetrains.CoaxialPodConfig;
import com.pedropathing.revhub.drivetrains.Swerve;
import com.pedropathing.revhub.drivetrains.SwerveConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Constants {

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                // Translational control
                Controller primaryTranslationalForward =
                        Controller.proportional(0.2937911100233925);
                Controller secondaryTranslationalForward =
                        Controller.proportional(0.10854796491060369);

                Controller primaryTranslationalLateral =
                        Controller.proportional(0.21);
                Controller secondaryTranslationalLateral =
                        Controller.proportional(0.1);

                c.forwardTranslational.set(
                        Controller.piecewise(secondaryTranslationalForward)
                                .put(2.5, primaryTranslationalForward)
                );

                c.strafeTranslational.set(
                        Controller.piecewise(secondaryTranslationalLateral)
                                .put(2.5, primaryTranslationalLateral)
                );

                // Feedforward
                c.coast.set(
                        Controller.proportionalFeedforward(0.011446126365118612)
                );

                c.brake.set(
                        Controller.proportionalFeedforward(0.00972920741035082)
                );

                // Heading control
                c.headingFeedback.set(
                        Controller.proportional(3.700332938780753)
                );

                c.headingBrakeCoefficients.set(
                        Vector2D.cartesian(
                                0.05695487078431248,
                                0.02734299141375188
                        )
                );

                // Braking
                c.linearBrakeCoefficients.set(
                        Matrix.diag(0.10, 0.08)
                );

                c.quadraticBrakeCoefficients.set(
                        Matrix.diag(0.0015, 0.0014)
                );

                // Measured forward performance
                c.maxAchievableForwardVelocity.set(
                        85.03270881936757
                );

                c.naturalForwardDeceleration.set(
                        65.38195380152824
                );

                // Retained until lateral performance is measured
                c.maxAchievableStrafeVelocity.set(50.0);
                c.naturalStrafeDeceleration.set(80.0);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.6122540151040385);
        c.yPodOffset.set(4.352876858448419);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static CoaxialPodConfig leftFront = new CoaxialPodConfig(
            c -> {
                c.name.set("leftFront");
                c.motorName.set("frontLeftDrive");
                c.servoName.set("frontLeftSteer");
                c.servoEncoderName.set("frontLeftEncoder");
                c.turnController.set(Controller.pid(0.55, 0, 0.0095)
                        .plus(Controller.proportionalFeedforward(0.0130)));


                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.analogMinVoltage.set(0.008);
                c.analogMaxVoltage.set(3.269);
                c.podOffset.set(Vector2D.cartesian(151.10, -109.344));


                c.angleOffsetRad.set(0.17);
            }
    );


    public static CoaxialPodConfig rightFront = new CoaxialPodConfig(
            c -> {
                c.name.set("rightFront");
                c.motorName.set("frontRightDrive");
                c.servoName.set("frontRightSteer");
                c.servoEncoderName.set("frontRightEncoder");
                c.turnController.set(Controller.pid(0.55, 0, 0.0095)
                        .plus(Controller.proportionalFeedforward(0.0130)));


                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.analogMinVoltage.set(0.0007);
                c.analogMaxVoltage.set(3.255);
                c.podOffset.set(Vector2D.cartesian(151.10, 109.344));


                c.angleOffsetRad.set(1.482);
            }
    );


    public static CoaxialPodConfig leftBack = new CoaxialPodConfig(
            c -> {
                c.name.set("leftBack");
                c.motorName.set("backLeftDrive");
                c.servoName.set("backLeftSteer");
                c.servoEncoderName.set("backLeftEncoder");
                c.turnController.set(Controller.pid(0.55, 0, 0.0095)
                        .plus(Controller.proportionalFeedforward(0.0130)));


                c.driveDirection.set(DcMotorSimple.Direction.REVERSE);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.analogMinVoltage.set(0.006);
                c.analogMaxVoltage.set(3.26);
                c.podOffset.set(Vector2D.cartesian(-151.10, -109.344));


                c.angleOffsetRad.set(3.251);
            }
    );


    public static CoaxialPodConfig rightBack = new CoaxialPodConfig(
            c -> {
                c.name.set("rightBack");
                c.motorName.set("backRightDrive");
                c.servoName.set("backRightSteer");
                c.servoEncoderName.set("backRightEncoder");
                c.turnController.set(Controller.pid(0.55, 0, 0.0095)
                        .plus(Controller.proportionalFeedforward(0.0130)));


                c.driveDirection.set(DcMotorSimple.Direction.FORWARD);
                c.servoDirection.set(DcMotorSimple.Direction.REVERSE);
                c.encoderReversed.set(false);
                c.analogMinVoltage.set(0.007);
                c.analogMaxVoltage.set(3.255);
                c.podOffset.set(Vector2D.cartesian(-151.10, 109.344));


                c.angleOffsetRad.set(0.869);
            }
    );


    public static SwerveConfig driveConfig = new SwerveConfig(
            c -> {
                c.zeroPowerBehavior.set(SwerveConfig.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
                c.manualBrakeMode.set(true);
                c.voltageCompensation.set(false);
            }
    );


    public static Follower create(HardwareMap h) {
        CoaxialPod leftFrontPod = new CoaxialPod(h, leftFront);
        CoaxialPod rightFrontPod = new CoaxialPod(h, rightFront);
        CoaxialPod leftBackPod = new CoaxialPod(h, leftBack);
        CoaxialPod rightBackPod = new CoaxialPod(h, rightBack);
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Swerve(h, driveConfig,
                leftBackPod, leftFrontPod, rightBackPod, rightFrontPod), new Foresight(foresightConfig));
    }


    //foresight tuner stuff

    public static Swerve createDrivetrain(HardwareMap h) {
        CoaxialPod leftFrontPod = new CoaxialPod(h, leftFront);
        CoaxialPod rightFrontPod = new CoaxialPod(h, rightFront);
        CoaxialPod leftBackPod = new CoaxialPod(h, leftBack);
        CoaxialPod rightBackPod = new CoaxialPod(h, rightBack);

        return new Swerve(
                h,
                driveConfig,
                leftBackPod,
                leftFrontPod,
                rightBackPod,
                rightFrontPod
        );
    }
}

/*
 * TURN CONTROLLER TUNING GUIDE
 *
 * Current controller:
 * c.turnController.set(
 *     Controller.pid(0.6, 0, 0.007)
 *         .plus(Controller.proportionalFeedforward(0))
 * );
 *
 * P = 0.6 (Proportional gain)
 * - Controls how strongly the controller responds to steering-angle error.
 * - If the pods steer too slowly or lag behind their target angles,
 *   try increasing P slightly.
 * - If the pods overshoot their target angles or oscillate,
 *   try decreasing P.
 *
 * I = 0 (Integral gain)
 * - Accumulates error over time to correct persistent errors.
 * - Keep at 0 initially.
 * - Only try a small positive value if a pod consistently fails to
 *   reach its target despite adequate P.
 * - Too much I can cause overshoot and oscillation.
 *
 * D = 0.007 (Derivative gain)
 * - Damps the response based on how quickly the error changes.
 * - If the pods overshoot or oscillate, try increasing D slightly.
 * - If steering becomes sluggish or noisy, try decreasing D.
 * - Change D in small increments.
 *
 * Proportional feedforward = 0
 * - Adds output proportional to the target signal.
 * - Leave at 0 initially.
 * - Only experiment with it after tuning PID and identifying a
 *   consistent need for additional steering response.
 *
 *
 * TUNING PROCEDURE
 *
 * 1. Run the same test after each change so results are comparable.
 * 2. Change only ONE PID value at a time.
 * 3. Start with P. Increase it gradually until steering responds
 *    quickly, but stop if the pods begin to oscillate or overshoot.
 * 4. Adjust D to help reduce overshoot and oscillation.
 * 5. Leave I at 0 unless persistent error remains.
 * 6. Test again at different movement powers once steering is smooth.
 *
 *
 * WHAT TO OBSERVE
 *
 * - Slow steering:
 *   Try increasing P slightly.
 *
 * - Overshooting or wobbling around the target:
 *   Reduce P or cautiously increase D.
 *
 * - Jittering when nearly aligned:
 *   Try reducing P. Also check encoder readings, angle offsets,
 *   mechanical friction, and steering linkage before increasing D.
 *
 * - One pod behaves differently from the others:
 *   Check that pod's encoder direction, angle offset, servo direction,
 *   and mechanical friction. Do not use PID to hide a hardware issue.
 *
 * - Smooth steering with minimal overshoot:
 *   Keep the settings and test again at a slightly higher power.
 *
 *
 * REPEATABLE MOVEMENT TEST
 *
 * Test in this order:
 * 1. Drive forward.
 * 2. Pause and observe the pods settling.
 * 3. Strafe sideways.
 * 4. Pause and observe the pods settling.
 * 5. Rotate in place.
 * 6. Pause and repeat.
 *
 * Forward and sideways movement mainly test pod positioning and
 * drivetrain behavior. Rotation is especially useful for observing
 * how the pods respond to changing steering targets.
 *
 * IMPORTANT:
 * This procedure does not automatically tune the PID values.
 * Change the values manually, rebuild, and rerun the test.
 * Keep movement power low and make sure the robot has clear space.
 */