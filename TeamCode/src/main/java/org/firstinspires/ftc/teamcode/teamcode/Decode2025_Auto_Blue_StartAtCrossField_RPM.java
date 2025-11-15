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
@Autonomous(name = "Decode2025_Auto_Blue_StartAtCrossField_RPM", group = "Robot")
public class Decode2025_Auto_Blue_StartAtCrossField_RPM extends LinearOpMode {
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

    // If using auto "drive to target", change this distance in front of the target you want the bot
    // to drive to in inches. example: goToTargetTagDistance(DESIRED_DISTANCE_TO_TARGET);
    final double DESIRED_DISTANCE_TO_TARGET = 62.235;

    // April Tag IDs to look for on the Obelisk
    final int GREEN_PURPLE_PURPLE_TAG_ID = 21;
    final int PURPLE_GREEN_PURPLE_TAG_ID = 22;
    final int PURPLE_PURPLE_GREEN_TAG_ID = 23;

    // April Tag IDs for Red and Blue goals/targets
    final int RED_TAG_ID = 24;
    final int BLUE_TAG_ID = 20;

    // GAME MATCH QUICK SETTINGS
    final int SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    // TODO: Verify cross-field shooting distance from center of target
    //  AprilTag to back of shooting wheel.
    final double CAMERA_TO_WHEEL_INCHES = 10.0; // inches between camera face and back of shooter wheel
    final double TESTED_CAMERA_TO_TARGET_INCHES = 119.1; // inches between camera face and target
    final double RPM_OF_SHOT_WHEN_TESTED = 5057.14; // <<----- CHANGE THIS POTENTIALLY
    final double MAX_RPM = 5800;
    final double RPM_NEEDED_PER_INCH = (RPM_OF_SHOT_WHEN_TESTED / TESTED_CAMERA_TO_TARGET_INCHES);
    double SHOOT_FROM_DISTANCE = 48.0; // <--- Change this if/when we know how far we are shooting from.

    // DEFAULT SETTINGS
    double currentRpm = 4000; //RPM_NEEDED_PER_INCH * SHOOT_FROM_DISTANCE;
    int classificationTagId = 0; // This will be set when detected by webcam
    AprilTagDetection shootingTargetTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        initialize();
        waitForStart();

        // start the shooter wheel
        carousel.turnShooterOnOffByRpm(currentRpm);

        moveRobot(0.1,0,0.0);

        /// /////////////////////////////////////
        // GET CLASSIFICATION TAG ID
        /// ////////////////////////////////////
        // read the classification tag ID so we know which order to shoot the balls
        sleep(500); // allow a bit of time for camera to capture tag
        classificationTagId = getClassificationTagId();

        /// /////////////////////////////////////
        // FACE TARGET
        /// ////////////////////////////////////
        moveRobot(0,0,-0.1);
        sleep(1000);
        moveRobot(0, 0, 0);

        /// /////////////////////////////////////
        // SHOOTING
        /// ////////////////////////////////////
        shootAtTargetTag();
        carousel.setShootingPower(0);

        /// /////////////////////////////////////
        // GO BACK TO LOADING ZONE OR NEARBY
        /// ////////////////////////////////////
        // slide right
        moveRobot(0.0, 1, 0);
        sleep(1500);

        moveRobot(0,0,0);

        carousel.turnShooterOnOffByRpm(0);

        // if we want to use april tags to move to or away from the robot
        // goToTargetTagDistance(DESIRED_DISTANCE_TO_TARGET);
        // or
        // goToTargetTagDistance(121.0);

        // THE END, SHUTTING DOWN
        //aprilTagsWebCam.stop();
    }

    public void moveRobot(double forward, double strafe, double turn) {
        // We created movements based on BLUE side of the field. So, if we switch
        // to Red, we need to flip the strafe and turn values. Forward will
        // still remain the same.
        if (SHOOTING_TARGET_TAG_ID == RED_TAG_ID) {
            strafe = -strafe;
            turn = -turn;
        }
        driver.drive(forward, strafe, turn);
    }

    public void shootAtTargetTag() {
        aprilTagsWebCam.update();
        shootingTargetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

//        if (shootingTargetTag != null && shootingTargetTag.ftcPose != null) {
//            double newShooterPower = calculateShooterRPM(shootingTargetTag.ftcPose.range);
//            carousel.setShootingPower(newShooterPower);
//        }

        // Now that we set the classification tag id, we can shoot.

        if (classificationTagId == GREEN_PURPLE_PURPLE_TAG_ID) {
            shootGreen();
            shootPurpleOne();
            shootPurpleTwo();
        } else if (classificationTagId == PURPLE_GREEN_PURPLE_TAG_ID) {
            shootPurpleOne();
            shootGreen();
            shootPurpleTwo();
        } else { // is either PURPLE_PURPLE_GREEN_TAG_ID or the default if classificationId not found
            shootPurpleOne();
            shootPurpleTwo();
            shootGreen();
        }
    }

    private void waitForShooterWheel() {
//        // create a while loop that checks the carousel.getShooterRpm()
//        // and if it hasn't reached currentRpm yet, sleep(200) until it does
//        // getShooterRPM returns values in increments of 85.7
//        while (Math.abs(carousel.getShooterRPM() - currentRpm) > 85.0) {
//            sleep(200);
//        }
    }

    private void initialize() {
        // Initialize the Apriltag Detection process
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.setManualExposure(6, 250, isStopRequested());

        driver.initialize(hardwareMap);
        carousel.initialize(hardwareMap, telemetry);

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to begin");
        telemetry.update();
    }

    private int getClassificationTagId() {
        if (classificationTagId > 0) {
            return classificationTagId;
        }

        List<AprilTagDetection> allTags = aprilTagsWebCam.getDetectedTags();
        if (allTags != null) {
            for (AprilTagDetection tag : allTags) {
                if (tag.id == GREEN_PURPLE_PURPLE_TAG_ID) {
                    return GREEN_PURPLE_PURPLE_TAG_ID;
                } else if (tag.id == PURPLE_GREEN_PURPLE_TAG_ID) {
                    return PURPLE_GREEN_PURPLE_TAG_ID;
                } else if (tag.id == PURPLE_PURPLE_GREEN_TAG_ID) {
                    return PURPLE_PURPLE_GREEN_TAG_ID;
                }
            }
        }
        return classificationTagId;
    }

    final int SLEEP_AFTER_POSITIONS = 1000;
    final int SLEEP_AFTER_KICK = 800;

    // Green Is A
    private void shootGreen() {
        carousel.gotoShootingA();
        sleep(SLEEP_AFTER_POSITIONS);
        waitForShooterWheel();
        useKicker();
    }

    // Purple One is B
    private void shootPurpleOne() {
        carousel.gotoShootingB();
        sleep(SLEEP_AFTER_POSITIONS);
        waitForShooterWheel();
        useKicker();
    }

    // Purple Two is X
    private void shootPurpleTwo() {
        carousel.gotoShootingX();
        sleep(SLEEP_AFTER_POSITIONS);
        waitForShooterWheel();
        useKicker();
    }

    // Sets carousel.kick position to 1.0 for 1 second.
    // then sets kick position to 0.0 and sleeps for 1 second
    private void useKicker() {
        carousel.kick(1.0);
        sleep(SLEEP_AFTER_KICK);
        carousel.kick(0.0);
        sleep(SLEEP_AFTER_KICK);
    }

    public double calculateShooterRPM(double inches) {
        if (inches < 16) {
            return currentRpm;
        }
        double newRPM = (inches + TESTED_CAMERA_TO_TARGET_INCHES) * RPM_NEEDED_PER_INCH;

        return Math.min(newRPM, MAX_RPM);
    }

    // Use april tag to auto move the bot to target until you reach distanceToTagInches.
    // For example if you want the robot to go to the position 4ft in front of the target
    // use:  goToTargetTagDistance(48);
    private void goToTargetTagDistance(double distanceToTagInches) {
        double strafe;
        double drive;
        double turn;

        // Will rotate until the SHOOTING_TARGET_TAG_ID is found.
        // Once found, will drive to target until reaching DESIRED_DISTANCE_TO_TARGET
        while (opModeIsActive())
        {
            aprilTagsWebCam.update();
            shootingTargetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);
            classificationTagId = getClassificationTagId();

            // If we have found the desired target, Drive to target Automatically. Otherwise, we will rotate
            if (shootingTargetTag != null ) {
                telemetry.addData("\n>","Driving to Target\n");
                telemetry.addData("Found", "ID %d (%s)", shootingTargetTag.id, shootingTargetTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", shootingTargetTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", shootingTargetTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", shootingTargetTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (shootingTargetTag.ftcPose.range - distanceToTagInches);
                double  headingError    = shootingTargetTag.ftcPose.bearing;
                double  yawError        = shootingTargetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                if (Math.abs(rangeError) < 1.2) {
                    return; // we exit out of the method when we are at the target distance.
                }
            } else {
                telemetry.addData("\n>","Locating Target\n");
                drive = 0;
                strafe = 0;
                turn = MAX_AUTO_TURN;
            }

            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, -strafe, -turn);
            sleep(10);
        }
    }

}
