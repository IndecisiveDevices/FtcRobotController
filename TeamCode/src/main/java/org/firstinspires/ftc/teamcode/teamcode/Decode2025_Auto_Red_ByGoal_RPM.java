package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@Autonomous(name = "Auto_Red_ByGoal", group = "Robot")
public class Decode2025_Auto_Red_ByGoal_RPM extends Decode2025_Auto_Blue_ByGoal_RPM {
    public Decode2025_Auto_Red_ByGoal_RPM() {
        SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
    }
}
