package org.firstinspires.ftc.teamcode.teamcode;

import android.hardware.Sensor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "Decode2025RobotCode_TeleOp", group = "Robot")
public class Decode2025RobotCode_TeleOp extends OpMode {

    // Drive Motors
    private DcMotor frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive;

    // can be used to determine robot orientation (gyro)
    private IMU imu;

    // Servos (carousel, kicker)
    private Servo carousel, kicker;

    // Shooter & Intake Motors
    private DcMotor shooterMotor, intakeMotor;

    // Color & Touch Sensors
    private ColorSensor colorSensor0, colorSensor1;
    private TouchSensor touchSensor0;

    // Lift Motors
    private DcMotor liftMotor0, liftMotor1;

    @Override
    public void init() {
        initDriveMotors();
        initImu();
        initSensors();
        initServos();
        initShooterMotors();
        initLiftMotors();
    }

    private void initLiftMotors() {
        liftMotor0 = hardwareMap.get(DcMotor.class, "liftMotor0");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
    }

    private void initShooterMotors() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    private void initServos() {
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");

    }

    private void initSensors() {
        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
        touchSensor0 = hardwareMap.get(TouchSensor.class, "touchSensor0");
    }

    private void initImu() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    private void initDriveMotors() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft_motor");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight_motor");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "rearLeft_motor");
        rearRightDrive = hardwareMap.get(DcMotor.class, "rearRight_motor");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double rearLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double rearRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0; // this is a throttle that we can set lower for demos. Do no set higher than 1.0

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(rearLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(rearRightPower));

        this.frontLeftDrive.setPower(maxSpeed * (frontLeftPower/ maxPower));
        this.rearRightDrive.setPower(maxSpeed * (rearRightPower/ maxPower));
        this.rearLeftDrive.setPower(maxSpeed * (rearLeftPower/ maxPower));
        this.frontRightDrive.setPower(maxSpeed * (frontRightPower/ maxPower));
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive(forward, strafe, rotate);

        if (gamepad1.right_bumper){
            moveCarouselClockwise();
        } else if (gamepad1.left_bumper){
            moveCarouselCounterClockwise();
        }
        carouselPosition();
        telemetry.update();
    }

    // create Intake and Shooting positions for the carousel A, B, and C positions (2 each)
    // they should be of type "double".  Example:  double someName = 1.0;
    double carouselPositionAintake = 0.968;
    double carouselPositionBintake = 0.482;
    double carouselPositionCintake = 0.016;

    double carouselPositionBshooter = 0.000;
    double carouselPositionCshooter = 0.929;
    double carouselPositionAshooter = 0.454;

    private void carouselPosition() {
        if (gamepad1.b) {
            carousel.setPosition(carouselPositionAintake);
        } else if (gamepad1.x) {
            carousel.setPosition(carouselPositionCintake);
        }
        // we need to add code here to slowly manually set the carousel's position using the gamepad
        // so we know what values to set our positions to.
        // this will always return the last value that you gave it for carousel.setPosition(double value)
        telemetry.addData("carousel position", "%.3f", carousel.getPosition());
    }

    double currentCarouselPosition = 0.5;
    public void moveCarouselClockwise() {
        currentCarouselPosition += 0.001;
        currentCarouselPosition = Math.min(currentCarouselPosition, 1.0);
        carousel.setPosition(currentCarouselPosition);
        telemetry.addData("Current Carousel Position", currentCarouselPosition);
    }

    public void moveCarouselCounterClockwise() {
        currentCarouselPosition -= 0.001;
        currentCarouselPosition = Math.max(currentCarouselPosition, 0.0);
        carousel.setPosition(currentCarouselPosition);
        telemetry.addData("Current Carousel Position", currentCarouselPosition);
    }

}
