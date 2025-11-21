package org.firstinspires.ftc.teamcode.teamcode.bypower;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Decode2025RobotCode_TeleOp", group = "Robot")
@Disabled
public class Decode2025RobotCode_TeleOp extends OpMode {
    MecanumDrive driver = new MecanumDrive();
    Lift lifter = new Lift();
    Carousel carousel = new Carousel();
    AprilTagsWebCam aprilTagsWebCam = new AprilTagsWebCam();

    // April Tag IDs for Red and Blue goals/targets
    final int RED_TAG_ID = 24;
    final int BLUE_TAG_ID = 20;

    // TODO: Verify cross-field shooting distance from center of target
    //  AprilTag to back of shooting wheel.
    final double CAMERA_TO_WHEEL_INCHES = 10.0; // inches between camera face and back of shooter wheel
    final double TESTED_CAMERA_TO_TARGET_INCHES = 119.1; // inches between camera face and target
    final double TESTED_SHOOTING_DISTANCE_FROM_WHEEL = (CAMERA_TO_WHEEL_INCHES + TESTED_CAMERA_TO_TARGET_INCHES);

    double TESTED_SHOOTING_POWER_CROSS_FIELD = 0.67; // <<----- Tested

    // GAME MATCH QUICK SETTINGS
    final int SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    // DEFAULT SETTINGS
    double currentShooterSpeed = TESTED_SHOOTING_POWER_CROSS_FIELD;
    boolean shooterWheelStarted = false; // set true if you want to start w/o shooting wheels on.

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
    }

    double SHOOTING_POWER_PER_INCH = (TESTED_SHOOTING_POWER_CROSS_FIELD / TESTED_SHOOTING_DISTANCE_FROM_WHEEL);

    @Override
    public void loop() {
        SHOOTING_POWER_PER_INCH = (TESTED_SHOOTING_POWER_CROSS_FIELD / TESTED_SHOOTING_DISTANCE_FROM_WHEEL);
        // Turn on shooter wheel at start of game.
        if (!shooterWheelStarted) {
            carousel.turnShooterOnOff(currentShooterSpeed);
            shooterWheelStarted = true;
        }

        aprilTagsWebCam.update();
        AprilTagDetection targetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

        double distanceToTarget = 0;

        if (targetTag == null) {
            telemetry.addData("No Tag Detected", "");
        } else {
            if (targetTag.ftcPose != null) {
                distanceToTarget = targetTag.ftcPose.range;
            }
            aprilTagsWebCam.displayDetectionTelemetry(targetTag);
        }

        telemetry.addData("Shooter Power", currentShooterSpeed);
        telemetry.addData("power per inch", SHOOTING_POWER_PER_INCH );

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
        if (gamepad2.dpadUpWasReleased()) {
            TESTED_SHOOTING_POWER_CROSS_FIELD += 0.01;
        } else if (gamepad2.dpadDownWasReleased()) {
            TESTED_SHOOTING_POWER_CROSS_FIELD -= 0.01;
        }

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

        // Set shooter speed based on distance to target
        currentShooterSpeed = calculateShooterPower(distanceToTarget);
        carousel.setShootingPower(currentShooterSpeed);

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
            // otherwise, do normal lift control
            if (gamepad1.right_trigger > 0) {
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
        carousel.showCarouselData(false);
        telemetry.update();
    }

    public double calculateShooterPower(double inches) {
        if (inches < 24) {
            return currentShooterSpeed;
        }
        double newPower = (inches + TESTED_CAMERA_TO_TARGET_INCHES) * SHOOTING_POWER_PER_INCH;

        return Math.min(newPower, .97);
    }

    // ---------------------------------------------------------------------
    // returns true if forward, strafe, and rotate are all at 0 (not moving)
    // ---------------------------------------------------------------------
    private boolean isRobotStopped(double forward, double strafe, double rotate) {
        return (forward == 0 && strafe == 0 && rotate == 0);
    }
}
