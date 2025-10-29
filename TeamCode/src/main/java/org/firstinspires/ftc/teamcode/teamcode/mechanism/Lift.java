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

    // Have motors move with positive power until
    // touch sensor is pressed.
    // uses:
    //  - liftMotor0/1
    //      .setPower()
    //  - touchSensor0
    //      .isPressed()
    public void liftUp() {

    }

    // Have motor move down until liftMotor0 or liftMotor1 is at start position.
    // uses:
    //  - liftMotor0/1
    //      .setPower()
    //
    public void liftDown() {
        // set liftMotor0 and liftMotor1
    }
}
