package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.AprilTagsWebCam;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Decode2025_Auto_Red_StartCrossField_RPM", group = "Auto Red")
public class Decode2025_Auto_Red_StartCrossField_RPM extends Decode2025_Auto_Blue_StartAtCrossField_RPM {
    Decode2025_Auto_Red_StartCrossField_RPM() {
        SHOOTING_TARGET_TAG_ID = RED_TAG_ID;
    }
}
