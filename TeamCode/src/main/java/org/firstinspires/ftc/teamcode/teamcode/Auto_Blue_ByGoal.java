package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Auto_Blue_ByGoal", group = "Auto Blue")
public class Auto_Blue_ByGoal extends LinearOpMode {
    protected MecanumDrive driver = new MecanumDrive();
    protected Carousel carousel = new Carousel();
    protected AprilTagsWebCam aprilTagsWebCam = new AprilTagsWebCam();

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
    final double DESIRED_DISTANCE_TO_TARGET = 45.5;

    // April Tag IDs to look for on the Obelisk
    final protected int GREEN_PURPLE_PURPLE_TAG_ID = 21;
    final protected int PURPLE_GREEN_PURPLE_TAG_ID = 22;
    final protected int PURPLE_PURPLE_GREEN_TAG_ID = 23;

    // April Tag IDs for Red and Blue goals/targets
    final protected int RED_TAG_ID = 24;
    final protected int BLUE_TAG_ID = 20;

    // GAME MATCH QUICK SETTINGS
    protected int SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    // TODO: Verify cross-field shooting distance from center of target
    //  AprilTag to back of shooting wheel.
    final double CAMERA_TO_WHEEL_INCHES = 10.0; // inches between camera face and back of shooter wheel
    final double TESTED_CAMERA_TO_TARGET_INCHES = 119.1; // inches between camera face and target
    final double RPM_OF_SHOT_WHEN_TESTED = 5057.14; // <<----- CHANGE THIS POTENTIALLY
    final double MAX_RPM = 5800;
    final double RPM_NEEDED_PER_INCH = (RPM_OF_SHOT_WHEN_TESTED / TESTED_CAMERA_TO_TARGET_INCHES);
    public double currentRpm = 2700; // RPM_OF_SHOT_WHEN_TESTED;

    // DEFAULT SETTINGS
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    int classificationTagId = 0; // This will be set when detected by webcam
    AprilTagDetection shootingTargetTag = null;     // Used to hold the data for a detected AprilTag

    protected void initialize() {
        // Initialize the Apriltag Detection process
        aprilTagsWebCam.initialize(hardwareMap, telemetry);
        aprilTagsWebCam.setManualExposure(6, 250);

        driver.initialize(hardwareMap);
        carousel.initialize(hardwareMap, telemetry);

        // Wait for driver to press start
//        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to begin now");
        telemetry.update();
    }

    @Override public void runOpMode()
    {
        initialize();
        waitForStart();
        carousel.gotoShootingA();
        // start the shooter wheel
        carousel.turnShooterOnOffByRpm(currentRpm - 100);

        /// /////////////////////////////////////
        // BACK AWAY FROM GOAL
        /// ////////////////////////////////////
        double backMovePower = -0.6;
        moveRobot(backMovePower, 0, 0.0);
        sleep(850);

        int slowdownCounter = 200;
        while (slowdownCounter > 0) {
            backMovePower *= 0.75;
            moveRobot(backMovePower, 0, 0);
            sleep(20);
            slowdownCounter -= 20;
        }

        moveRobot(0, 0, 0);
        sleep(2000);


        carousel.turnIntakeMotorOn();
        /// /////////////////////////////////////
        // SHOOTING GOAL
        /// ////////////////////////////////////
        shootAtTargetTag();
        carousel.setShootingPower(0);

        /// /////////////////////////////////////
        // GO BACK TO LOADING ZONE OR NEARBY
        /// ////////////////////////////////////
        // rotate a bit, toward the loading zone
        moveRobot(0.0, -0.3, 0.0);
        sleep(1000);

        // then move backward to the loading zone
        moveRobot(0.4, 0, -0.3);
        sleep(500);

        // then move backward to the loading zone
        moveRobot(0.2, 0, 0);
        sleep(780);

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

        // Now that we set the classification tag id, we can shoot.
        telemetry.addLine("Getting ready to shoot in 3 seconds");
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
            carousel.setShooterRPM(currentRpm);
            shootGreen();
        }
    }

    final int SLEEP_AFTER_POSITIONS = 2000; // 1500;
    final int SLEEP_AFTER_KICK = 1000; // 1200;
    final int SLEEP_AFTER_SHOOT = 1000; // 1200;

    // Green Is A
    private void shootGreen() {
        carousel.gotoShootingA();
        sleep(SLEEP_AFTER_POSITIONS);
        telemetry.addLine(" RPM:" + carousel.getShooterRPM());
        useKicker();
        sleep(SLEEP_AFTER_SHOOT);
    }

    // Purple One is B
    private void shootPurpleOne() {
        carousel.gotoShootingB();
        sleep(SLEEP_AFTER_POSITIONS);
        telemetry.addLine(" RPM:" + carousel.getShooterRPM());
        useKicker();
        carousel.setShooterRPM(currentRpm - 100); // TEMP
        sleep(SLEEP_AFTER_SHOOT);
    }

    // Purple Two is X
    private void shootPurpleTwo() {
        carousel.gotoShootingX();
        sleep(SLEEP_AFTER_POSITIONS);
        telemetry.addLine(" RPM:" + carousel.getShooterRPM());
        useKicker();
        sleep(SLEEP_AFTER_SHOOT);
    }

    // Sets carousel.kick position to 1.0 for 1 second.
    // then sets kick position to 0.0 and sleeps for 1 second
    private void useKicker() {
        carousel.kick(1.0);
        sleep(SLEEP_AFTER_KICK);
        carousel.kick(0.0);
        sleep(SLEEP_AFTER_KICK);
    }


    double distanceToTarget = 0;

    private void goToTarget(double desiredDistance) {
        while (opModeIsActive())
        {
            boolean targetFound = false;

            double drive;
            double turn;
            double strafe;

//            aprilTagsWebCam.update();
            AprilTagDetection targetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

            if (targetTag == null) {
                telemetry.addData("Tag NOt Detected", SHOOTING_TARGET_TAG_ID);
            } else {
                if (targetTag.ftcPose != null) {
                    distanceToTarget = targetTag.ftcPose.range;
                    targetFound = true;
                    aprilTagsWebCam.displayDetectionTelemetry(targetTag);
                }
            }

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
            sleep(10);
        }
    }
}
