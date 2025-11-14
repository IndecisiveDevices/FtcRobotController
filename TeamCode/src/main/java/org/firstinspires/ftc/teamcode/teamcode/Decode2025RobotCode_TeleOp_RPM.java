package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Decode2025RobotCode_TeleOp_RPM", group = "Robot")
public class Decode2025RobotCode_TeleOp_RPM extends OpMode {
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
    final double RPM_OF_SHOT_WHEN_TESTED = 5057.14; // <<----- CHANGE THIS POTENTIALLY
    final double MAX_RPM = 5800;
    final double RPM_NEEDED_PER_INCH = (RPM_OF_SHOT_WHEN_TESTED / TESTED_CAMERA_TO_TARGET_INCHES);
    double currentRpm = RPM_OF_SHOT_WHEN_TESTED;

    // GAME MATCH QUICK SETTINGS
    final int SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    // DEFAULT SETTINGS
    boolean shooterWheelStarted = false; // set true if you want to start w/o shooting wheels on.
    private boolean useCalculatedVelocity = true;

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // Turn on shooter wheel at start of game.
        if (!shooterWheelStarted || gamepad2.startWasReleased()) {
            carousel.turnShooterOnOffByRpm(currentRpm);
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

        // Set shooter speed based on distance to target
        if (gamepad2.startWasReleased()) {
            if (useCalculatedVelocity) {
                useCalculatedVelocity = false;
            } else {
                useCalculatedVelocity = true;
            }
        }

        if (useCalculatedVelocity) {
            currentRpm = calculateShooterRPM(distanceToTarget);
        } else {
            // gamepad2.back: turns shooter on/off
            // gamepad2.dpad_up/down: sets shooter speed
            if (gamepad2.dpadUpWasReleased()) {
                currentRpm += 50;
            } else if (gamepad2.dpadDownWasReleased()) {
                currentRpm -= 50;
            }
        }
        carousel.setShooterRPM(currentRpm);

        //----------------------------
        // gamepad2.left_trigger: Shooting Mode.
        // IF gamepad2 left trigger is pressed
        //  - buttons X, B, A will go to shooter positions
        //  - right trigger will set ball kicker to up
        // ELSE
        //  - buttons X, B, A will go to intake positions
        //  - kicker will be set to down
        //----------------------------
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
        carousel.showCarouselData();
        telemetry.update();
    }

    public double calculateShooterRPM(double inches) {
        if (inches < 16) {
            return currentRpm;
        }
        double newRPM = (inches + TESTED_CAMERA_TO_TARGET_INCHES) * RPM_NEEDED_PER_INCH;

        return Math.min(newRPM, MAX_RPM);
    }

    // ---------------------------------------------------------------------
    // returns true if forward, strafe, and rotate are all at 0 (not moving)
    // ---------------------------------------------------------------------
    private boolean isRobotStopped(double forward, double strafe, double rotate) {
        return (forward == 0 && strafe == 0 && rotate == 0);
    }
}
