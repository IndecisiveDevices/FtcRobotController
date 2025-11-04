package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
    }


    @Override
    public void loop() {
        //----------------------------
        // Drive Controls (Done)
        //----------------------------
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driver.drive(forward, strafe, rotate);

        //-------------------
        // Carousel Controls
        //-------------------
        // Left trigger start Shooting Mode.
        // IF gamepad2 left trigger is pressed
        //  - start the shooter  wheels motor
        //  - buttons X, B, A will go to shooter positions
        //  - right trigger will set ball kicker to 1.0 (it "shoots").
        // ELSE
        //  - buttons X, B, A will go to intake positions
        //  - kicker will be set to 0.0
        //----------------------------
        carousel.setShootingPower(gamepad2.left_trigger);

        if (gamepad2.left_trigger > 0) {
            if (gamepad2.b) {
                carousel.gotoShootingB();
            } else if (gamepad2.x) {
                carousel.gotoShootingX();
            } else if (gamepad2.a) {
                carousel.gotoShootingA();
            } else {
                if (gamepad2.right_trigger > 0) {
                    carousel.kick(1.0);
                } else {
                    carousel.kick(0.0);
                }
            }
        }
        else {
            carousel.kick(0.0);

            if (gamepad2.b) {
                carousel.gotoIntakeB();
            } else if (gamepad2.x) {
                carousel.gotoIntakeX();
            } else if (gamepad2.a) {
                carousel.gotoIntakeA();
            }
        }

        if (gamepad2.y) {
            carousel.turnIntakeMotorOnOff();
        }

//        //----------------------------
//        // Lift Controls (DONE)
//        //   - gamepad2
//        //      .dpad_up: lifter lifts up
//        //      .dpad_down: lifter lowers down
//        //----------------------------
//        if (gamepad2.dpad_up) {
//            driver.drive(0,0,0);
//            lifter.liftUp();
//        } else if (gamepad2.dpad_down) {
//            driver.drive(0,0,0);
//            lifter.liftDown();
//        }

        // Coach Ed's code block for troubleshooting
        if (gamepad1.a) {
            lifter.liftLeft(-gamepad1.left_trigger);
            lifter.liftRight(-gamepad1.right_trigger);
        } else {
            lifter.liftLeft(gamepad1.left_trigger);
            lifter.liftRight(gamepad1.right_trigger);
        }

        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
        lifter.displayLiftPositions();
        carousel.showCarouselData();
        telemetry.update();
    }
}
