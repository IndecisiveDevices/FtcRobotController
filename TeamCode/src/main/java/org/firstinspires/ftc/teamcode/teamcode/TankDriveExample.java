package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Tank Drive Example", group = "Workshop")
public class TankDriveExample extends OpMode {
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor rearLeftDrive;
    private DcMotor rearRightDrive;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "FL");
        rearLeftDrive = hardwareMap.get(DcMotor.class, "RL");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FR");
        rearRightDrive = hardwareMap.get(DcMotor.class, "RR");

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        //----------------------------
        // Tank Drive Controls
        //----------------------------
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        frontLeftDrive.setPower(leftPower);
        rearLeftDrive.setPower(leftPower);
        frontRightDrive.setPower(rightPower);
        rearRightDrive.setPower(rightPower);

        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
    }
}
