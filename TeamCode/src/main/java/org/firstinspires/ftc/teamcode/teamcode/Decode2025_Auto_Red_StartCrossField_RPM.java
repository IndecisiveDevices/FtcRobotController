package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Auto_Red_CF", group = "Auto Red")
@Disabled
public class Decode2025_Auto_Red_StartCrossField_RPM extends Decode2025_Auto_Blue_ByGoal_RPM {
    Decode2025_Auto_Red_StartCrossField_RPM() {
        SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
        currentRpm = 3300;
    }
// DO NOT USE
    @Override public void runOpMode()
    {
        initialize();
        waitForStart();
        // start the shooter wheel
        carousel.turnShooterOnOffByRpm(currentRpm + 100);
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
