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
        //      .toggleIntakeOnOff()
        //   - gamepad2
        //      .x: counter-clockwise to next slot
        //      .b: clockwise to next slot
        //      .y: intake motor on/off
        //----------------------------
        if (gamepad2.a) {
            carousel.nextRightIntakePosition();
        } else if (gamepad2.x) {
            carousel.nextLeftIntakePosition();
        }

        //----------------------------
        // Shooting Controls (DONE)
        //   - gamepad2
        //      .left_bumper: counter-clockwise (left) to next slot
        //      .right_bumper: clockwise (right) to next slot
        //      .left_trigger: set shooter wheel power
        //      .right_trigger: kick ball (shoot ball) - shooter wheel must be going
        //----------------------------
        else if (gamepad2.left_bumper) {
            carousel.nextLeftShootingPosition();
        } else if (gamepad2.right_bumper) {
            carousel.nextRightShootingPosition();
        } else if (gamepad2.left_trigger > 0) {
            carousel.setShootingPower(gamepad2.left_trigger);

            if (gamepad2.right_trigger > 0) {
                carousel.kick();
            }
        }

        //----------------------------
        // Lift Controls (TODO)
        //   - gamepad2
        //      .dpad_up: lifter lifts up
        //      .dpad_down: lifter lowers down
        //----------------------------
        if (gamepad2.dpad_up) {
            driver.drive(0,0,0);
            lifter.liftUp();
        } else if (gamepad2.dpad_down) {
            driver.drive(0,0,0);
            lifter.liftDown();
        }

        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
        carousel.showCarouselData();
        telemetry.update();
    }
}
