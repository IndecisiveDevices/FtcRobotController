package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDrive {
    private HardwareMap hardwareMap;

    // Drive Motors
    public DcMotor frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive;

    // can be used to determine robot orientation (gyro)
    private IMU imu;

    public void initialize(HardwareMap hardwareMap, String frontLeftMotorName, String frontRightMotorName, String rearLeftMotorName, String rearRightMotorName) {
        this.hardwareMap = hardwareMap;
        initDriveMotors(frontLeftMotorName, frontRightMotorName, rearLeftMotorName, rearRightMotorName);
        initImu();
    }

    // defaults to motor names "frontLeft_motor" "frontRight_motor" "rearLeft_motor"
    //"rearRight_motor"
    public void initialize(HardwareMap hardwareMap) {
        this.initialize(hardwareMap, "front_left_drive", "front_right_drive", "back_left_drive", "back_right_drive");
    }

    private void initDriveMotors(String frontLeftMotorName, String frontRightMotorName, String rearLeftMotorName, String rearRightMotorName) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, frontLeftMotorName);
        frontRightDrive = hardwareMap.get(DcMotor.class, frontRightMotorName);
        rearLeftDrive = hardwareMap.get(DcMotor.class, rearLeftMotorName);
        rearRightDrive = hardwareMap.get(DcMotor.class, rearRightMotorName);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initImu() {
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
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

    public void powerFrontLeftMotor(double power) {
        frontLeftDrive.setPower(power);
    }
    public void powerFrontRightMotor(double power) {
        frontRightDrive.setPower(power);
    }
    public void powerRearLeftMotor(double power) {
        rearLeftDrive.setPower(power);
    }
    public void powerRearRightMotor(double power) {
        rearRightDrive.setPower(power);
    }
}
