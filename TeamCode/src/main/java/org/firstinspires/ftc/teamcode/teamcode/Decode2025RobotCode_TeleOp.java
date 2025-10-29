package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;

@TeleOp(name = "Decode2025RobotCode_TeleOp", group = "Robot")
public class Decode2025RobotCode_TeleOp extends OpMode {
    MecanumDrive driver = new MecanumDrive();
    Lift lifter = new Lift();
    Carousel carousel = new Carousel();

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap);
        carousel.initialize(hardwareMap);
    }


    @Override
    public void loop() {
        //----------------------------
        // Drive Controls (Done)
        // uses:
        //   - driver.drive()
        //   - gamepad1 (left stick x/y, right stick x)
        //----------------------------
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driver.drive(forward, strafe, rotate);

        //----------------------------
        // Intake Controls (IN PROGRESS)
        // uses:
        //   - carousel
        //      .nextRightIntakePosition()
        //      .nextLeftIntakePosition()
        //      .toggleIntakeOnOff()
        //   - gamepad2
        //      .x: counter-clockwise to next slot
        //      .b: clockwise to next slot
        //      .y: intake motor on/off
        //----------------------------
        if (gamepad2.a){
            carousel.nextRightIntakePosition();
        } else if (gamepad2.x){
            carousel.nextLeftIntakePosition(); // TODO
        }

        //----------------------------
        // Shooting Controls (TODO)
        // uses:
        //   - carousel
        //      .nextRightShootingPosition()
        //      .nextLeftShootingPosition()
        //      .setShootingPower(double type)
        //      .shoot()
        //   - gamepad2
        //      .left_bumper: counter-clockwise (left) to next slot
        //      .right_bumper: clockwise (right) to next slot
        //      .left_trigger: set shooter power
        //      .right_trigger: shoot ball
        //----------------------------


        //----------------------------
        // Lift Controls (TODO)
        // uses:
        //   - lifter (Lift)
        //   - gamepad2 (dpad up/down)
        // behavior:
        //   - stop drive
        //   - dpad up lifts motor until sensor is pressed
        //   - dpad down lowers motor until motors are at "0" position
        //----------------------------

        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
        telemetry.update();
    }
}
