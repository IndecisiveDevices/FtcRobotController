package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Auto_Blue_CF", group = "Auto Blue")
public class Auto_Blue_CF extends Auto_Blue_ByGoal {
    public Auto_Blue_CF() {
        SHOOTING_TARGET_TAG_ID = BLUE_TAG_ID;
            currentRpm = 3300;
            centerOffSet = -3.0;
    }

    @Override public void runOpMode()
    {
        initialize();
        waitForStart();
        // start the shooter wheel
        carousel.turnShooterOnOffByRpm(currentRpm + 100);
        carousel.gotoShootingA(); // NOT SHOOTING...just setting the carousel position

        // move robot off the wall
        moveRobot(0.1, 0, 0);
        sleep(700);

        // waiting for shooter motor to reach rpm
        moveRobot(0, 0, 0);

        findClassificationIdTag();
        centerOnTarget();

        sleep(3100);

        telemetry.addData("Classification Tag ID: ", classificationTagId);
        telemetry.update();

        moveRobot(0, 0,0);

        /// /////////////////////////////////////
        // SHOOTING
        /// ////////////////////////////////////
        // carousel.turnIntakeMotorOnOff();
        // shootAtTargetTag();
        // carousel.setShootingPower(0);

        /// /////////////////////////////////////
        // Move away from launch line
        /// ////////////////////////////////////
        moveRobot(0.05, -0.1, 0);
        sleep(1500);

        moveRobot(0.0, 0.0, 0.3);
        sleep(500);

        moveRobot(0,0,0);

        carousel.turnShooterOnOffByRpm(0);
    }
}
