package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Auto_Red_ByGoal", group = "Robot")
public class Auto_Red_ByGoal extends Auto_Blue_ByGoal {
    public Auto_Red_ByGoal() {
        SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
    }
}
