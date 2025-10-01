package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;

public class MecanumDriveMechanism {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    private IMU imu;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft_motor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight_motor");
        backLeftMotor = hardwareMap.dcMotor.get("rearLeft_motor");
        backRightMotor = hardwareMap.dcMotor.get("rearRight_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        Arrays.asList(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)
              .forEach(motor -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        );

        imu.initialize(new IMU.Parameters(hubOrientation));
    }

    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double setSpeed = 1.0;

        maxPower = Math.max(Math.abs(frontLeftPower), maxPower);
        maxPower = Math.max(Math.abs(backLeftPower), maxPower);
        maxPower = Math.max(Math.abs(frontRightPower), maxPower);
        maxPower = Math.max(Math.abs(backRightPower), maxPower);

        frontLeftMotor.setPower(setSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(setSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(setSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(setSpeed * (backRightPower / maxPower));

    }

    public void driveFieldOriented(double forward, double strafe, double rotate) {
        double joystickAngle = Math.atan2(forward, strafe);
        double joystickSpeed = Math.hypot(strafe, forward);

        double robotCurrentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double correctedAngle = AngleUnit.normalizeRadians(joystickAngle - robotCurrentAngle);

        double correctedForward = joystickSpeed * Math.sin(correctedAngle);
        double correctedStrafe = joystickSpeed * Math.cos(correctedAngle);

        drive(correctedForward, correctedStrafe, rotate);
    }
}
