package org.firstinspires.ftc.teamcode.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.teamcode.Decode2025_Auto_Blue_ByGoal_RPM;

@Autonomous(name = "Auto_Red_CF", group = "Auto Red")
public class Auto_Red_CF_V2 extends Decode2025_Auto_Blue_ByGoal_RPM {
    public Auto_Red_CF_V2() {
        SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
        currentRpm = 3300;
    }

    @Override public void runOpMode()
    {
        initialize();
        waitForStart();
        // start the shooter wheel
        carousel.turnShooterOnOffByRpm(currentRpm + 200);
        carousel.gotoShootingA();

        moveRobot(0, 0, 0);
        sleep(3500);

        /// /////////////////////////////////////
        // SHOOTING
        /// ////////////////////////////////////
        carousel.turnIntakeMotorOnOff();
        shootAtTargetTag();
        carousel.setShootingPower(0);

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

