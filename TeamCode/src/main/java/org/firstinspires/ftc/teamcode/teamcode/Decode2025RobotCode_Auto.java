package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Decode2025RobotCode_Auto", group = "Robot")
public class Decode2025RobotCode_Auto extends LinearOpMode {
    MecanumDrive driver = new MecanumDrive();
    AprilTagsWebCam aprilTagsWebCam = new AprilTagsWebCam();

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 48.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final int RED_TAG_ID = 24;
    private static final int BLUE_TAG_ID = 20;
    private static final int SHOOTING_TARGET_TAG_ID = RED_TAG_ID; // <<----- CHANGE THIS POTENTIALLY

    private static final int CLASS_GREEN_PURPLE_PURPLE = 21; // 21 (GPP), 22 (PGP), 23 (PPG)
    private static final int CLASS_PURPLE_GREEN_PURPLE = 22;
    private static final int CLASS_PURPLE_PURPLE_GREEN = 23;


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

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        // Move forward for a few seconds so we can move off wall (if close)
        driver.drive(0.5, 0, 0);
        sleep(4000);

        driver.drive(0, 0, 0);

        boolean targetReached = false;

        while (opModeIsActive() && !targetReached)
        {
            aprilTagsWebCam.update();
            targetFound = false;
            shootingTargetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

            if (shootingTargetTag == null) {
                targetFound = false;
            } else {
                targetFound = true;
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound ) {  // && gamepad1.left_bumper

                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", shootingTargetTag.id, shootingTargetTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", shootingTargetTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", shootingTargetTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", shootingTargetTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (shootingTargetTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = shootingTargetTag.ftcPose.bearing;
                double  yawError        = shootingTargetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                if (Math.abs(rangeError) < 1.0) {
                    targetReached = true;
                }
            } else {
                telemetry.addData("\n>","Locating Target\n");
                drive = 0;
                strafe = 0;
                turn = MAX_AUTO_TURN;

//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            driver.drive(drive, -strafe, -turn);
            sleep(10);
        }

        sleep(1000);

        targetReached = false;

        while (opModeIsActive())
        {
            aprilTagsWebCam.update();
            targetFound = false;
            shootingTargetTag = aprilTagsWebCam.getTagBySpecificId(SHOOTING_TARGET_TAG_ID);

            if (shootingTargetTag == null) {
                targetFound = false;
            } else {
                targetFound = true;
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound ) {  // && gamepad1.left_bumper

                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", shootingTargetTag.id, shootingTargetTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", shootingTargetTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", shootingTargetTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", shootingTargetTag.ftcPose.yaw);

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (shootingTargetTag.ftcPose.range - 96.0); // move back 2 feet
                double  headingError    = shootingTargetTag.ftcPose.bearing;
                double  yawError        = shootingTargetTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                if (Math.abs(rangeError) < 1.0) {
                    targetReached = true;
                }
            } else {
                telemetry.addData("\n>","Locating Target\n");
                drive = 0;
                strafe = 0;
                turn = MAX_AUTO_TURN;

//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            driver.drive(drive, -strafe, -turn);
            sleep(10);
        }

        // shimmy left, then backwards
        driver.drive(-0.5, -0.5, 0);
        sleep(1500);
        driver.drive(0, 0, 0);

        //aprilTagsWebCam.stop();
    }
}
