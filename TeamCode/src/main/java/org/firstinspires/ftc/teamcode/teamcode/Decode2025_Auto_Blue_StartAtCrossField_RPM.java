package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Decode2025_Auto_Blue_StartAtCrossField_RPM", group = "Auto Blue")
public class Decode2025_Auto_Blue_StartAtCrossField_RPM extends Decode2025_Auto_Blue_ByGoal_RPM {
    public Decode2025_Auto_Blue_StartAtCrossField_RPM() {
        SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID;
        currentRpm = 3300;
    }

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
        // FACE TARGET
        /// ////////////////////////////////////
//        moveRobot(0,0,-0.4);
//        sleep(800);
        moveRobot(0, 0, 0);

        sleep(3500);

        /// /////////////////////////////////////
        // SHOOTING
        /// ////////////////////////////////////
        carousel.turnIntakeMotorOnOff();
        shootAtTargetTag();
        carousel.setShootingPower(0);

        carousel.turnIntakeMotorOnOff();
        /// /////////////////////////////////////
        // Move away from launch line
        /// ////////////////////////////////////
        moveRobot(0.5, 0, 0);
        sleep(1000);

        moveRobot(0.0, 0.0, -0.3);
        sleep(500);

        moveRobot(0,0,0);

        carousel.turnShooterOnOffByRpm(0);
    }
}
