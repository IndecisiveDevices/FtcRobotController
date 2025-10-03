package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;


@TeleOp(name="MotorTestByButton", group="TestCode")
public class MotorTestByButton extends OpMode {

    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft_motor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight_motor");
        backLeftMotor = hardwareMap.dcMotor.get("rearLeft_motor");
        backRightMotor = hardwareMap.dcMotor.get("rearRight_motor");

        Arrays.asList(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor)
                .forEach(motor -> motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER));
    }

    @Override
    public void loop() {

        if (gamepad1.x) {
            frontLeftMotor.setPower(gamepad1.left_stick_y);
        }
        else if (gamepad1.a) {
            backLeftMotor.setPower(gamepad1.left_stick_y);
        }
        else if ( gamepad1.y) {
            frontRightMotor.setPower(gamepad1.left_stick_y);
        }
        else if (gamepad1.b) {
            backRightMotor.setPower(gamepad1.left_stick_y);
        }

    }
}
