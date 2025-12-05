package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto_Red_CF_NEW", group = "Auto Red")
public class Auto_Red_CF extends Auto_Blue_CF {
    public Auto_Red_CF() {
        SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
        currentRpm = 3300;
    }
}

