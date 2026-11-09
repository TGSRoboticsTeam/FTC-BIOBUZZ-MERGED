package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Memory;

@TeleOp(name = "HiveDrive", group = "Swerve")
//@Disabled
public class HiveDrive extends LinearOpMode {

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private DcMotor intake1, intake2;
    private DcMotor launcher1, launcher2;
    private Servo blocker;
    private Servo turret;
    private Servo aimingFlap;

    private GoBildaPinpointDriver pinpoint;
    private IMU imu;
    private VoltageSensor voltageSensor;


    // SWERVE GEOMETRY
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);


    final double FRONT_LEFT_OFFSET  = 0.05;
    final double FRONT_RIGHT_OFFSET = 1.39;
    final double BACK_LEFT_OFFSET   = 3.16;
    final double BACK_RIGHT_OFFSET  = 0.82;


    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;

    // SPEED CONTROL
    final double MAX_SPEED_DEFAULT = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    private boolean lockWheels = false;

    // Normalize motors
    final double MAX_BATTERY_VOLTAGE = 14.5;


    private boolean launching = false;
    private boolean wasLauncherTriggerPressed = false;
    private boolean turretDefaultPosition = true;
    private boolean wasTurretTriggerPressed = false;

    // Poses
    private final Pose2D gardenCornerPosition = new Pose2D(DistanceUnit.INCH, 8, 8, AngleUnit.RADIANS, 90);
    final double CLOSE_HIVE_Y = 54.8;
    final double FAR_HIVE_Y = 91.8;
    final double HIVE_X = 57.8;

    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        //imu.resetYaw();
        pinpoint.setPosition(Memory.storedPosition);

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;


        while (opModeIsActive()) {

            double batteryVoltage = (voltageSensor != null) ? voltageSensor.getVoltage() : Double.NaN;

            double speedMultiplier = MAX_SPEED_DEFAULT;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }


            // FIELD-CENTRIC RESETTING
            if (gamepad1.dpad_up) { // Reset heading
                imu.resetYaw();
                pinpoint.setHeading(0.0, AngleUnit.RADIANS);
            }

            if (gamepad1.dpad_down) { // Reset location
                pinpoint.setPosX(gardenCornerPosition.getX(DistanceUnit.INCH), DistanceUnit.INCH);
                pinpoint.setPosY(gardenCornerPosition.getY(DistanceUnit.INCH), DistanceUnit.INCH);
            }

            if (gamepad1.x) {
                lockWheels = true;
            }

            // PINPOINT
            Pose2D pinpointPosition = pinpoint.getPosition();
            double pinpointPositionX = pinpointPosition.getX(DistanceUnit.INCH);
            double pinpointPositionY = pinpointPosition.getY(DistanceUnit.INCH);
            double pinpointHeading = pinpointPosition.getHeading(AngleUnit.RADIANS);

            // SWERVE-DRIVE CODE
            // Raw inputs
            double rawY = -gamepad1.left_stick_y * speedMultiplier; // Forward/Backward
            double rawX = gamepad1.left_stick_x * speedMultiplier;  // Strafe Left/Right
            double rot = gamepad1.right_stick_x * speedMultiplier;  // Rotation

            // Field-centric inputs
            double robotX = rawX * Math.cos(pinpointHeading) + rawY * Math.sin(pinpointHeading);
            double robotY = -rawX * Math.sin(pinpointHeading) + rawY * Math.cos(pinpointHeading);

            // Swerve math
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);
            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed;
                speedBackRight /= maxSpeed;
            }

            // Steering Logic
            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                lockWheels = false;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Lock wheels
            if (lockWheels) {
                targetAngleFL = -Math.PI / 4; targetAngleFR = Math.PI / 4;
                targetAngleBL = Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Run swerve modules
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);

            // TURRET
            double hiveY = Math.min(Math.max(
                    CLOSE_HIVE_Y,
                    pinpointPositionY),
                    FAR_HIVE_Y
            );
            turret.setPosition(
                    getAngleToPosition(pinpointPositionX, pinpointPositionY, HIVE_X, hiveY)
            );

            /*
            boolean toggleTurret = gamepad1.a;
            if (toggleTurret && !wasTurretTriggerPressed) {
                turretDefaultPosition = !turretDefaultPosition;
            }
            if (turretDefaultPosition) {
                // Backward aiming
                turret.setPosition(0.293);
                aimingFlap.setPosition(0.575);
            } else {
                // Forward aiming
                turret.setPosition(0.87);
                aimingFlap.setPosition(0.585);
            }
            wasTurretTriggerPressed = toggleTurret;
            //*/

            // INTAKE AND BLOCKER
            if (gamepad1.left_trigger > 0.5) {
                blocker.setPosition(0.5); // Open
                intake1.setPower(0.55);
                intake2.setPower(0.55);
            } else {
                blocker.setPosition(0.35); // Closed
                intake1.setPower(0.0);
                intake2.setPower(0.0);
            }
            if (gamepad1.right_trigger > 0.5) {
                intake1.setPower(1.0); // Fast intake
                intake2.setPower(1.0);
            }

            // LAUNCHER
            boolean toggleLauncher = gamepad1.left_bumper;
            if (toggleLauncher && !wasLauncherTriggerPressed) {
                launching = !launching;
            }
            double launchPower = normalizeVoltage(batteryVoltage, 11.0, 1.1);
            telemetry.addData("Power Feathering", launchPower);
            telemetry.addData("Feathered Voltage", batteryVoltage * launchPower);
            if (launching) {
                launcher1.setPower(launchPower);
                launcher2.setPower(launchPower);
            } else {
                launcher1.setPower(0.0);
                launcher2.setPower(0.0);
            }
            wasLauncherTriggerPressed = toggleLauncher;

            // TELEMETRY
            telemetry.addData("Battery Voltage:", voltageSensor.getVoltage());
            telemetry.update();

            // UPDATE PINPOINT
            pinpoint.update();
        }
    }

    private void initializeHardware() {
        // Swerve Drive Hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive"); // Motor Port 3
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive"); // Motor Port 2
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive"); // Motor Port 1
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive"); // Motor Port 0
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer"); // Servo Port 0
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer"); // Servo Port 1
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer"); // Servo Port 5
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer"); // Servo Port 4
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder"); // Analog Input Devices Port 0
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder"); // Analog Input Devices Port 1
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder"); // Analog Input Devices Port 3
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder"); // Analog Input Devices Port 2

        intake1  = hardwareMap.get(DcMotor.class, "intake1"); // Expansion Motor Port 0
        intake2 = hardwareMap.get(DcMotor.class, "intake2"); // Expansion Motor Port 1
        launcher1   = hardwareMap.get(DcMotor.class, "launcher1"); // Expansion Motor Port 2
        launcher2  = hardwareMap.get(DcMotor.class, "launcher2"); // Expansion Motor Port 3
        turret = hardwareMap.get(Servo.class, "turret"); // Expansion Servo Port 5
        blocker = hardwareMap.get(Servo.class, "blocker"); // Expansion Servo Port 3
        aimingFlap = hardwareMap.get(Servo.class, "aimingFlap"); // Expansion Servo Port 0

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);

        // Pinpoint Odometry
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-1.6122540151040385, 4.352876858448419, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                                      GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();

        // Motor Directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset RunMode for all DC motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1; // IMPORTANT Invert servo power

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double normalizeVoltage(double batteryVoltage, double targetVoltage, double slopeGain) {
        return Math.min(Math.max(targetVoltage * (MAX_BATTERY_VOLTAGE / batteryVoltage) * slopeGain, 0.0), 1.0);
    }

    private double getAngleToPosition(double robotX, double robotY, double targetX, double targetY) {
        double forwardAngle = 0.87;
        double backwardAngle = 0.293;

        double angle = Math.atan2(targetY - robotY, targetX - robotX);
        return forwardAngle - (angle / (2.0 * Math.PI)) * (forwardAngle - backwardAngle);
    }
}