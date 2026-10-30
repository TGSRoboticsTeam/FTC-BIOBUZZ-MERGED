
package org.firstinspires.ftc.teamcode.Auto.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Analog Min Max Tuner", group = "Pedro Pathing")
@Disabled
public class AnalogMinMaxTuner extends OpMode {

    private final String[] servoNames = {
            "frontLeftSteer",
            "frontRightSteer",
            "backLeftSteer",
            "backRightSteer"
    };

    private final String[] encoderNames = {
            "frontLeftEncoder",
            "frontRightEncoder",
            "backLeftEncoder",
            "backRightEncoder"
    };

    private CRServo[] servos;
    private AnalogInput[] encoders;
    private double[] minVoltage;
    private double[] maxVoltage;

    @Override
    public void init() {
        servos = new CRServo[servoNames.length];
        encoders = new AnalogInput[encoderNames.length];
        minVoltage = new double[encoderNames.length];
        maxVoltage = new double[encoderNames.length];

        for (int i = 0; i < servoNames.length; i++) {
            servos[i] = hardwareMap.get(
                    CRServo.class, servoNames[i]);

            encoders[i] = hardwareMap.get(
                    AnalogInput.class, encoderNames[i]);

            minVoltage[i] = Double.POSITIVE_INFINITY;
            maxVoltage[i] = Double.NEGATIVE_INFINITY;

            // Do not rotate the servo automatically.
            servos[i].setPower(0);
        }

        telemetry.addLine("Analog Min Max Tuner");
        telemetry.addLine("Continuous-rotation servos set to zero power.");
        telemetry.addLine("Rotate modules manually to check feedback.");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addLine("Feedback voltage readings:");

        for (int i = 0; i < encoders.length; i++) {
            double voltage = encoders[i].getVoltage();

            minVoltage[i] = Math.min(minVoltage[i], voltage);
            maxVoltage[i] = Math.max(maxVoltage[i], voltage);

            telemetry.addData(
                    servoNames[i] + " current (V)", voltage);

            telemetry.addData(
                    encoderNames[i] + " min (V)", minVoltage[i]);

            telemetry.addData(
                    encoderNames[i] + " max (V)", maxVoltage[i]);
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (servos != null) {
            for (CRServo servo : servos) {
                servo.setPower(0);
            }
        }
    }
}
