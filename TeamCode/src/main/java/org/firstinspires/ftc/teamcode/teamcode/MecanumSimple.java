package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;

@TeleOp(name = "Drive With Mecanum", group = "Workshop")
public class MecanumSimple extends OpMode {
    MecanumDrive driver = new MecanumDrive();

    @Override
    public void init() {
        driver.initialize(hardwareMap, "FL","FR","RL","RR");
    }

    @Override
    public void loop() {

        if (gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x == 0) {
            driver.powerFrontLeftMotor(gamepad1.square ? 0.5 : 0.0);
            driver.powerFrontRightMotor(gamepad1.triangle ? 0.5 : 0.0);
            driver.powerRearLeftMotor(gamepad1.cross ? 0.5 : 0.0);
            driver.powerRearRightMotor(gamepad1.circle ? 0.5 : 0.0);
            return;
        }

        //----------------------------
        // Drive Controls (Done)
        //----------------------------
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0) {
            forward = forward / 2;
            strafe = strafe / 2;
            rotate = rotate / 2;
        }

        driver.drive(forward, strafe, rotate);


    }

}