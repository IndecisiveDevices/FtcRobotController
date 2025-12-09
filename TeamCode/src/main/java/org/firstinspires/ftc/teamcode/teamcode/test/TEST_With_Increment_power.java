package org.firstinspires.ftc.teamcode.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TEST_With_Increment_power", group = "TEST")
public class TEST_With_Increment_power extends OpMode {
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
    final double TESTED_CAMERA_TO_TARGET_INCHES = 120.1; // inches between camera face and target
    final double RPM_OF_SHOT_WHEN_TESTED = 5057.14; // <<----- CHANGE THIS POTENTIALLY
    final double MAX_RPM = 5800;
    final double RPM_NEEDED_PER_INCH = (RPM_OF_SHOT_WHEN_TESTED / TESTED_CAMERA_TO_TARGET_INCHES);

    final static double CROSS_FIELD_SHOT_RPM = 3450;
    final static double CLOSE_SHOT_RPM = 2500;

    double currentRpm = CLOSE_SHOT_RPM; //<-- Hard-coded instead of RPM_OF_SHOT_WHEN_TESTED;

    // GAME MATCH QUICK SETTINGS
    int SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    // DEFAULT SETTINGS
    boolean shooterWheelStarted = true; // set true if you want to start w/o shooting wheels on.
    private boolean useCalculatedVelocity = false;

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.autoSetExposure(5000);
    }

    ElapsedTime targetReachedTimer = new ElapsedTime();
    int currentTimerSetting = 0;

    @Override
    public void loop() {
        //carousel.saveSlotColor();

        // Turn on shooter wheel at start of game.
        if (!shooterWheelStarted) {
            carousel.turnShooterOnOffByRpm(currentRpm);
            shooterWheelStarted = true;
        }

        telemetry.addData("Shooter Set To: ", currentRpm);

        ////////////////////////////////////////////////////////
        // TESTING NEXT/PREV LOGIC
        ////////////////////////////////////////////////////////
        if (gamepad2.rightBumperWasPressed()) {
            carousel.nextRightIntakePosition();
        } else if (gamepad2.leftBumperWasPressed()) {
            carousel.nextLeftIntakePosition();
        }

        if (gamepad2.backWasReleased()){
            carousel.turnShooterOnOffByRpm(currentRpm);
        }

        if (gamepad1.xWasReleased()) {
            SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID;
        }
        else if (gamepad1.bWasReleased()) {
            SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
        }
        if (SHOOTING_TARGET_TAG_ID == BLUE_TAG_ID) {
            telemetry.addData("Target tag: ", "Blue");
        }
        else if (SHOOTING_TARGET_TAG_ID == RED_TAG_ID) {
            telemetry.addData("Target tag: ", "Red");
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

//        if (gamepad1.left_trigger > 0) {
//            if (gamepad1.dpad_down) {
//                forward = 0;
//                strafe = 0;
//                rotate = centerOnTarget();
//            } else {
//                forward = forward / 2;
//                strafe = strafe / 2;
//                rotate = rotate / 2;
//            }
//        }

        driver.drive(forward, strafe, rotate);

        if (gamepad1.dpadLeftWasReleased()) {
            centerOffSet -= 0.5;
        } else if (gamepad1.dpadRightWasReleased()) {
            centerOffSet += 0.5;
        }
        telemetry.addData("Center Offset: ", centerOffSet);


        //-------------------
        // Carousel Controls
        //-------------------

//        // Set shooter speed based on distance to target
//        if (gamepad2.startWasReleased()) {
//            if (useCalculatedVelocity) {
//                useCalculatedVelocity = false;
//            } else {
//                useCalculatedVelocity = true;
//            }
//        }

        telemetry.addData("Calculate RPM: " , useCalculatedVelocity ? "✅" : "❌");

//        if (useCalculatedVelocity) {
//            currentRpm = calculateShooterRPM(distanceToTarget);
//        } else {
        // gamepad2.back: turns shooter on/off
        // gamepad2.dpad_up/down: sets shooter speed

//        }

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

            carousel.showCarouselData(false);

            telemetry.addLine("Shooter activated...");

            if (gamepad2.dpadUpWasReleased()) {
                currentRpm += 100;
                carousel.setShooterRPM(currentRpm);
            } else if (gamepad2.dpadDownWasReleased()) {
                currentRpm -= 100;
                carousel.setShooterRPM(currentRpm);
            }

            if (gamepad2.bWasPressed()) {
                carousel.gotoShootingB();
            } else if (gamepad2.xWasPressed()) {
                carousel.gotoShootingX();
            } else if (gamepad2.aWasPressed()) {
                carousel.gotoShootingA();
            } else {
                if (gamepad2.right_trigger > 0) {
                    if (carousel.targetRpmReached()) {
                        carousel.kick(1.0);
                    }
                } else {
                    carousel.kick(0.0);
                }
            }
        }
        else {

            carousel.showCarouselData(false);

            carousel.kick(0.0);

            if (gamepad2.dpadUpWasReleased()) {
                currentRpm = CROSS_FIELD_SHOT_RPM;
                carousel.setShooterRPM(currentRpm);
            } else if (gamepad2.dpadDownWasReleased()) {
                currentRpm = CLOSE_SHOT_RPM;
                carousel.setShooterRPM(currentRpm);
            }

            if (gamepad2.bWasPressed()) {
                carousel.gotoIntakeB();
            } else if (gamepad2.xWasPressed()) {
                carousel.gotoIntakeX();
            } else if (gamepad2.aWasPressed()) {
                carousel.gotoIntakeA();
            }
        }

        if (gamepad2.yWasPressed()) {
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
//            // otherwise, do normal lift control
//            if (gamepad1.right_trigger > 0) {
//                if (gamepad1.a) {
//                    lifter.liftDown(gamepad1.right_trigger / 2);
//                } else {
//                    lifter.liftUp(gamepad1.right_trigger / 2);
//                }
//            } else {
//                lifter.stopLift();
//            }

            if (gamepad1.a) {
                lifter.liftLeft(-gamepad1.left_trigger);
                lifter.liftRight(-gamepad1.right_trigger);
            } else {
                lifter.liftLeft(gamepad1.left_trigger);
                lifter.liftRight(gamepad1.right_trigger);
            }

        }

        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
        telemetry.addData("liftRatio: ", liftRatio);
        lifter.displayLiftPositions();
        telemetry.update();
    }

    double liftRatio = 0.5;

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


    // Adjust these numbers to suit your robot.
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 1;   //  Clip the turn speed to this max value (adjust for your robot)

    double distanceToTarget = 0;

    private void goToTarget(double desiredDistance) {
        boolean targetFound = false;

        double drive;
        double turn;
        double strafe;

        aprilTagsWebCam.update();
        AprilTagDetection targetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

//        if (targetTag == null) {
//            telemetry.addData("Tag NOt Detected", SHOOTING_TARGET_TAG_ID);
//        } else {
//            if (targetTag.ftcPose != null) {
//                distanceToTarget = targetTag.ftcPose.range;
//                targetFound = true;
//                aprilTagsWebCam.displayDetectionTelemetry(targetTag);
//            }
//        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (targetTag.ftcPose.range - desiredDistance);
            double  headingError    = targetTag.ftcPose.bearing;
            double  yawError        = targetTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // rangeError is less than .8", stop and exit method
            if (Math.abs(rangeError) < 0.8) {
                driver.drive(0,0,0);
                return;
            }

            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else {
            // rotate until we see the tag
            turn   = -0.0;
            drive  = -0.05;
            strafe = 0;
            telemetry.addLine("Looking for tag ...");
        }
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        driver.drive(drive, -strafe, -turn);
    }

    /**
     * Rotates the robot to center on the specified AprilTag without driving forward or backward.
     * The robot will turn until the tag is directly in front of it (headingError is minimal).
     */
    double centerOffSet = 0;
    private double centerOnTarget() {
        // First, get the most recent tag data
        aprilTagsWebCam.update();
        AprilTagDetection targetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

        // If the tag isn't found, we can't do anything.
        if (targetTag == null || targetTag.ftcPose == null) {
            telemetry.addLine("Cannot center: Tag not visible.");
            // Stop any previous movement
            driver.drive(0, 0, 0);
            return 0; // Exit the method
        }

        // The 'bearing' or 'headingError' is the angle we need to correct.
        // A positive value means the tag is to our right, so we need to turn right.
        // A negative value means the tag is to our left, so we need to turn left.
        double headingError = targetTag.ftcPose.bearing - centerOffSet; // blue

        // Stop turning if we are centered (e.g., within 2 degrees).
        // This prevents the robot from jittering back and forth.
        final double HEADING_TOLERANCE = 0.5; // degrees
        if (Math.abs(headingError) <= HEADING_TOLERANCE) {
            telemetry.addLine("Centered on Target!");
            driver.drive(0, 0, 0); // Stop turning
            return 0; // We are centered, so we're done.
        }

        telemetry.addData("Centering...", "Heading Error: %.2f", headingError);
//        aprilTagsWebCam.displayDetectionTelemetry(targetTag);

        // Calculate the turning power. The 'TURN_GAIN' slows down the rotation
        // as the robot gets closer to being centered, providing smoother control.
        double turnPower = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        return -turnPower;
//        // Apply only the turning motion. Drive and strafe are set to 0.
//        // Note: The 'turn' from the original method corresponds to a negative 'turnPower'
//        // if you want to maintain the same rotational direction.
//        driver.drive(0, 0, -turnPower);

    }

}
