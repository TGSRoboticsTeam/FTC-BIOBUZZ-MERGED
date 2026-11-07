package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
// IMU is no longer strictly needed for drive, but kept for initialization if mechanisms use it.
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "SwerveConfigTest", group = "Tools")
//@Disabled
public class SwerveConfigTest extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {

            frontRightDrive.setPower(0.0);
            frontRightSteer.setPower(0.0);
            frontLeftDrive.setPower(0.0);
            frontLeftSteer.setPower(0.0);
            backRightDrive.setPower(0.0);
            backRightSteer.setPower(0.0);
            backLeftDrive.setPower(0.0);
            backLeftSteer.setPower(0.0);

            if (gamepad1.y) { // Front Right
                frontRightDrive.setPower(0.1);
                frontRightSteer.setPower(0.1);
                telemetry.addData("CURRENT SWERVE POD:", "FRONT RIGHT");
            }
            if (gamepad1.x) { // Front Left
                frontLeftDrive.setPower(0.1);
                frontLeftSteer.setPower(0.1);
                telemetry.addData("CURRENT SWERVE POD:", "FRONT LEFT");
            }
            if (gamepad1.b) { // Back Right
                backRightDrive.setPower(0.1);
                backRightSteer.setPower(0.1);
                telemetry.addData("CURRENT SWERVE POD:", "BACK RIGHT");
            }
            if (gamepad1.a) { // Back Left
                backLeftDrive.setPower(0.1);
                backLeftSteer.setPower(0.1);
                telemetry.addData("CURRENT SWERVE POD:", "BACK LEFT");
            }

            //*
            telemetry.addData("", "");
            telemetry.addData("ENCODER DATA", "");
            telemetry.addData("Front Left:", ( (DcMotorEx) frontLeftDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Front Right:", ( (DcMotorEx) frontRightDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Back Left:", ( (DcMotorEx) backLeftDrive).getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Back Right:", ( (DcMotorEx) backRightDrive).getCurrent(CurrentUnit.AMPS));
            //*/
            telemetry.update();
        }
    }

    private void initializeHardware() {
        // --- Swerve Drive Hardware --- //
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

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}