package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Decode2025RobotCode_Auto", group = "Robot")
public class Decode2025RobotCode_Auto extends LinearOpMode {
    MecanumDrive driver = new MecanumDrive();
    Carousel carousel = new Carousel();
    AprilTagsWebCam aprilTagsWebCam = new AprilTagsWebCam();

    // Adjust these numbers to suit your robot.
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    final double MAX_SHOOTING_POWER = 0.68;

    // April Tag IDs to look for on the Obelisk
    final int GREEN_PURPLE_PURPLE_TAG_ID = 21;
    final int PURPLE_GREEN_PURPLE_TAG_ID = 22;
    final int PURPLE_PURPLE_GREEN_TAG_ID = 23;

    // April Tag IDs for Red and Blue goals/targets
    final int RED_TAG_ID = 24;
    final int BLUE_TAG_ID = 20;

    // GAME MATCH QUICK SETTINGS
    final int SHOOTING_TARGET_TAG_ID = RED_TAG_ID; // <<----- CHANGE THIS POTENTIALLY
    final double DESIRED_DISTANCE_TO_TARGET = 48.0; //  this is how close the camera should get to the target (inches)
    final double SHOOTING_POWER_CROSS_FIELD = 0.67;
    final double SHOOTING_POWER_CLOSE_RANGE = 0.40;

    int classificationTagId = 0; // This will be set when detected by webcam

    @Override public void runOpMode()
    {
        AprilTagDetection shootingTargetTag = null;     // Used to hold the data for a detected AprilTag

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.setManualExposure(6, 250, isStopRequested());

        driver.initialize(hardwareMap);
        carousel.initialize(hardwareMap, telemetry);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        // start the shooter wheel (TODO)


        // Move forward for a few seconds so we can move off wall (if close)
        driver.drive(0.5, 0, 0);
        sleep(4000);

        driver.drive(0, 0, 0);

        boolean targetReached = false;

        // Will rotate until the SHOOTING_TARGET_TAG_ID is found.
        // Once found, will drive to target until reaching DESIRED_DISTANCE_TO_TARGET
        while (opModeIsActive() && !targetReached)
        {
            aprilTagsWebCam.update();
            targetFound = false;
            shootingTargetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);
            classificationTagId = getClassificationTagId();

            if (shootingTargetTag == null) {
                targetFound = false;
            } else {
                targetFound = true;
            }

            // If we have found the desired target, Drive to target Automatically. Otherwise, we will rotate
            if (targetFound ) {
                telemetry.addData("\n>","Driving to Target\n");
                telemetry.addData("Found", "ID %d (%s)", shootingTargetTag.id, shootingTargetTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", shootingTargetTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", shootingTargetTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", shootingTargetTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (shootingTargetTag.ftcPose.range - DESIRED_DISTANCE_TO_TARGET);
                double  headingError    = shootingTargetTag.ftcPose.bearing;
                double  yawError        = shootingTargetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                if (Math.abs(rangeError) < 1.2) {
                    targetReached = true;
                }
            } else {
                telemetry.addData("\n>","Locating Target\n");
                drive = 0;
                strafe = 0;
                turn = MAX_AUTO_TURN;
            }

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            driver.drive(drive, -strafe, -turn);
            sleep(10);
        }

        // DO WE SHOOT HERE? (TODO)
        shootPattern();
        sleep(500);

        // NOW, GET OFF A LAUNCH LINE.
        // strafe (slide) left/back for a bit.
        driver.drive(-0.5, -0.5, 0);
        sleep(1500);

        // THE END, SHUTTING DOWN
        carousel.setShootingPower(0);
        //aprilTagsWebCam.stop();
    }

    private int getClassificationTagId() {
        if (classificationTagId > 0) {
            return classificationTagId;
        }

        List<AprilTagDetection> allTags = aprilTagsWebCam.getDetectedTags();
        for (AprilTagDetection tag : allTags) {
            if (tag.id == GREEN_PURPLE_PURPLE_TAG_ID) {
                return GREEN_PURPLE_PURPLE_TAG_ID;
            } else if (tag.id == PURPLE_GREEN_PURPLE_TAG_ID) {
                return PURPLE_GREEN_PURPLE_TAG_ID;
            } else if (tag.id == PURPLE_PURPLE_GREEN_TAG_ID) {
                return PURPLE_PURPLE_GREEN_TAG_ID;
            }
        }
        return classificationTagId;
    }

    private void shootPattern() {
        if (classificationTagId == GREEN_PURPLE_PURPLE_TAG_ID) {
            shootPatternGPP();
        } else if (classificationTagId == PURPLE_GREEN_PURPLE_TAG_ID) {
            shootPatternPGP();
        } else if (classificationTagId == PURPLE_PURPLE_GREEN_TAG_ID) {
            shootPatternPPG();
        } else {
            telemetry.addLine("Unable to determine pattern. Shooting PPG pattern.");
            shootPatternPPG();
        }
    }

    final int SLEEP_AFTER_POSITIONS = 500;
    final int SLEEP_AFTER_KICK = 800;

    // NOTE: Determine which color we are loading into the slots. Then we can implement.
    // Example if the colors are loaded as
    // - X: Purple
    // - A: Green
    // - B: Purple
    // Then this next method would be implemented as such...
    private void shootPatternGPP() {
        carousel.gotoShootingA();
        sleep(SLEEP_AFTER_POSITIONS);
        useKicker();

        carousel.gotoShootingB();
        sleep(SLEEP_AFTER_POSITIONS);
        useKicker();

        carousel.gotoShootingA();
        sleep(SLEEP_AFTER_POSITIONS);
        useKicker();
    }

    // (TODO)
    private void shootPatternPGP() {

    }

    // (TODO)
    private void shootPatternPPG() {

    }

    // Sets carousel.kick position to 1.0 for 1 second.
    // then sets kick position to 0.0 and sleeps for 1 second
    private void useKicker() {
        carousel.kick(1.0);
        sleep(SLEEP_AFTER_KICK);
        carousel.kick(0.0);
        sleep(SLEEP_AFTER_KICK);
    }

}
