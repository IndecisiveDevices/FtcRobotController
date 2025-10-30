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

        liftMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        if (touchSensor0.isPressed()) { return; }

        liftMotor0.setPower(0.1);
        liftMotor1.setPower(0.1);
    }

    // Have motor move down until liftMotor0 or liftMotor1 is at start position.
    // uses:
    //  - liftMotor0/1
    //      .setPower()
    //
    public void liftDown() {
        liftMotor0.setTargetPosition(0);
        liftMotor1.setTargetPosition(0);
        liftMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor0.setPower(0.1);
        liftMotor1.setPower(0.1);
    }
}
