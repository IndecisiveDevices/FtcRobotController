package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "ServoTest2025RobotCode_TeleOp", group = "Robot")
public class Ed_Test_TeleOp extends OpMode {

    ServoImplEx carousel;
    Servo kicker;

    @Override
    public void init() {
        carousel = hardwareMap.get(ServoImplEx.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");

        telemetry.addData("", kicker.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        carousel.setPosition(normalizePosition(-gamepad1.left_stick_y));
        kicker.setPosition(normalizePosition(-gamepad1.right_stick_y));

        telemetry.addData("kicker.getPosition", kicker.getPosition());
        telemetry.addData("carousel.getPosition", carousel.getPosition());
        telemetry.addData("left stick y", normalizePosition(-gamepad1.left_stick_y));
        telemetry.addData("right stick y", normalizePosition(-gamepad1.right_stick_y));
        telemetry.update();
    }

    private double normalizePosition(double input) {
        return (input + 1) / 2;
    }
}
