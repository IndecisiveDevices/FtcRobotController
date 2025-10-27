package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Lift {
    // Lift Motors
    private DcMotor liftMotor0, liftMotor1;
    private TouchSensor touchSensor0;


    public void initialize(HardwareMap hardwareMap) {
        liftMotor0 = hardwareMap.get(DcMotor.class, "liftMotor0");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");
        touchSensor0 = hardwareMap.get(TouchSensor.class, "touchSensor0");

        // lift is expected to be off (down) when initialized
        // We only have 1 touch sensor to use to tell us when the robot is at the top.
        // But how can the program tell when the robot is at the bottom?

    }

    // Motors + Sensor
    public void liftUp() {

    }

    // Motors only. We
    public void liftDown() {

    }
}
