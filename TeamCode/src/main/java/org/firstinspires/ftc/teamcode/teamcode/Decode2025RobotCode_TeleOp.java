package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Decode2025RobotCode_TeleOp", group = "Robot")
public class Decode2025RobotCode_TeleOp extends OpMode {
    MecanumDrive driver = new MecanumDrive();
    Lift lifter = new Lift();
    Carousel carousel = new Carousel();
    AprilTagsWebCam aprilTagsWebCam = new AprilTagsWebCam();


    // GAME MATCH QUICK SETTINGS
    double SHOOTING_POWER_CROSS_FIELD = 0.65;
    double SHOOTING_POWER_CLOSE_RANGE = 0.25;

    // DEFAULT SETTINGS
    double currentShooterSpeed = SHOOTING_POWER_CROSS_FIELD;

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagsWebCam.update();
        AprilTagDetection red = aprilTagsWebCam.getTagBySpecificId(24);
        AprilTagDetection blue = aprilTagsWebCam.getTagBySpecificId(20);

        aprilTagsWebCam.displayDetectionTelemetry(red);
        aprilTagsWebCam.displayDetectionTelemetry(blue);


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
        // gamepad2.back: turns shooter on/off
        // gamepad2.dpad_up/down: sets shooter speed
        //----------------------------
        // gamepad2.left_trigger: Shooting Mode.
        // IF gamepad2 left trigger is pressed
        //  - buttons X, B, A will go to shooter positions
        //  - right trigger will set ball kicker to up
        // ELSE
        //  - buttons X, B, A will go to intake positions
        //  - kicker will be set to down
        //----------------------------
        if (gamepad2.backWasPressed()) {
            carousel.turnShooterOnOff(currentShooterSpeed);
        }

        if (gamepad2.dpadUpWasPressed()) {
            currentShooterSpeed = SHOOTING_POWER_CROSS_FIELD;
            carousel.setShootingPower(currentShooterSpeed);
        } else if (gamepad2.dpadDownWasPressed()) {
            currentShooterSpeed = SHOOTING_POWER_CLOSE_RANGE;
            carousel.setShootingPower(currentShooterSpeed);
        }

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

        //----------------------------
        // Lift Controls (DONE)
        // Lift control only engaged when robot it not moving
        // - gamepad1.right_trigger + a: lower lift
        // - gamepad1.right_trigger: raise lift
        //----------------------------
        boolean robotIsStopped = isRobotStopped(forward, strafe, rotate);

        if (robotIsStopped) {
            // use for resetting between matches
            if (gamepad1.b) {
                lifter.liftRight(gamepad1.right_trigger);
                lifter.liftLeft(gamepad1.left_trigger);
            } else if (gamepad1.x) {
                lifter.liftRight(-gamepad1.right_trigger);
                lifter.liftLeft(-gamepad1.left_trigger);
            }
            // otherwise, do normal lift control
            else if (gamepad1.right_trigger > 0) {
                if (gamepad1.a) {
                    lifter.liftDown(gamepad1.right_trigger / 2);
                } else {
                    lifter.liftUp(gamepad1.right_trigger / 2);
                }
            } else {
                lifter.stopLift();
            }

        }

        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
        lifter.displayLiftPositions();
        carousel.showCarouselData();
        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // returns true if forward, strafe, and rotate are all at 0 (not moving)
    // ---------------------------------------------------------------------
    private boolean isRobotStopped(double forward, double strafe, double rotate) {
        return (forward == 0 && strafe == 0 && rotate == 0);
    }
}
