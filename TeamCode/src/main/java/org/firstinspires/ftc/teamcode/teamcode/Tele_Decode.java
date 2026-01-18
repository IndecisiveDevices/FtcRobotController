package org.firstinspires.ftc.teamcode.teamcode;

import static android.os.SystemClock.sleep;

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

@TeleOp(name = "Tele_Decode", group = "Decode")
public class Tele_Decode extends OpMode {
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
    final static double CLOSE_SHOT_RPM = 2700;

    double currentRpm = CLOSE_SHOT_RPM; //<-- Hard-coded instead of RPM_OF_SHOT_WHEN_TESTED;

    // GAME MATCH QUICK SETTINGS
    int SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    // DEFAULT SETTINGS
    boolean shooterWheelStarted = false; // set true if you want to start w/o shooting wheels on.
    private boolean useCalculatedVelocity = false;

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.autoSetExposure(3000);
    }

    @Override
    public void loop() {
        // Turn on shooter wheel at start of game.
        if (!shooterWheelStarted) {
            carousel.turnShooterOnOffByRpm(currentRpm);
            shooterWheelStarted = true;
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
            telemetry.addLine("\uD83D\uDFE6 \uD83D\uDFE6  BLUE TARGET \uD83D\uDFE6 \uD83D\uDFE6");
        }
        else if (SHOOTING_TARGET_TAG_ID == RED_TAG_ID) {
            telemetry.addLine("\uD83D\uDFE5 \uD83D\uDFE5  RED TARGET\uD83D\uDFE5 \uD83D\uDFE5");
        }

        //----------------------------
        // Drive Controls (Done)
        //----------------------------
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0) {
            if (gamepad1.dpad_down) {
                forward = 0;
                strafe = 0;
                rotate = centerOnTarget();
            } else {
                forward = forward / 2;
                strafe = strafe / 2;
                rotate = rotate / 2;
            }
        }

        driver.drive(forward, strafe, rotate);

        if (gamepad1.dpadLeftWasReleased()) {
            centerOffSet -= 1.0;
        } else if (gamepad1.dpadRightWasReleased()) {
            centerOffSet += 1.0;
        }
        telemetry.addLine("CENTER OFFSET: " + centerOffSet + "   )))");


        //-------------------
        // Carousel Controls
        //-------------------

        //telemetry.addData("useCalculatedVelocity RPM: " , useCalculatedVelocity);


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
            carousel.showCarouselData(true);

            if (gamepad2.dpadUpWasReleased()) {
                currentRpm += 100;
                carousel.setShooterRPM(currentRpm);
            } else if (gamepad2.dpadDownWasReleased()) {
                currentRpm -= 100;
                carousel.setShooterRPM(currentRpm);
            } else if (gamepad2.leftBumperWasPressed()) {
                carousel.nextLeftShootingPosition();
            }
            else if (gamepad2.rightBumperWasPressed()) {
                carousel.nextRightShootingPosition();
            }
            else if (gamepad2.bWasPressed()) {
                carousel.gotoShootingB();
            } else if (gamepad2.xWasPressed()) {
                carousel.gotoShootingX();
            } else if (gamepad2.aWasPressed()) {
                carousel.gotoShootingA();
            } else {
                if (gamepad2.right_trigger > 0) {
                   // if (carousel.targetRpmReached()) {
                        carousel.kick(1.0);
                    //}
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


            // if left bumper, got to intake left position
            if (gamepad2.leftBumperWasPressed()) {
                carousel.nextLeftIntakePosition();
            }
            else if (gamepad2.rightBumperWasPressed()) {
                carousel.nextRightIntakePosition();
            }
            else if (gamepad2.bWasPressed()) {
                carousel.gotoIntakeB();
            } else if (gamepad2.xWasPressed()) {
                carousel.gotoIntakeX();
            } else if (gamepad2.aWasPressed()) {
                carousel.gotoIntakeA();
            }
        }

        if (gamepad2.y) {
            carousel.turnIntakeMotorOn();
        }
        else if (gamepad2.yWasReleased()) {
            carousel.turnIntakeMotorOff();
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
                    lifter.liftToPosition(7206, 7086, gamepad1.right_trigger / 2);
                } else {
                    lifter.liftToPosition(100, 100, gamepad1.right_trigger / 2);
                }
            } else {
                lifter.stopLift();
            }
        }

        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
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

        lifter.displayLiftPositions();
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


    // Adjust these numbers to suit your robot.
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double TURN_GAIN   =  0.02  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_TURN  = 1;   //  Clip the turn speed to this max value (adjust for your robot)


    /**
     * Rotates the robot to center on the specified AprilTag without driving forward or backward.
     * The robot will turn until the tag is directly in front of it (headingError is minimal).
     */
    double centerOffSet = 0;
    boolean disableGetRefresh = false;

    private double centerOnTarget() {
        // First, get the most recent tag data
        aprilTagsWebCam.update();
        AprilTagDetection targetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

        // If the tag isn't found,
        if (targetTag == null) {
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
        aprilTagsWebCam.displayDetectionTelemetry(targetTag);

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
