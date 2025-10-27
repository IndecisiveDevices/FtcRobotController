package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDrive {
    private HardwareMap hardwareMap;

    // Drive Motors
    private DcMotor frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive;

    // can be used to determine robot orientation (gyro)
    private IMU imu;

    public void initialize(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initDriveMotors();
        initImu();
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
}
