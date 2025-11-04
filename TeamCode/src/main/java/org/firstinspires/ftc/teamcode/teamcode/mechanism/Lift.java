package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    // Lift Motors
    private DcMotor liftMotor0, liftMotor1;

    // Sensors
    private TouchSensor touchSensor0;
    private Telemetry telemetry;


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        touchSensor0 = hardwareMap.get(TouchSensor.class, "touchSensor0");

        liftMotor0 = hardwareMap.get(DcMotor.class, "liftMotor0");
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor1");

        liftMotor0.setDirection(DcMotor.Direction.REVERSE);

        liftMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        // if the touchSensor is pressed, we are at the top
        // and need to stop the motors so they don't cause damage.
        if (touchSensor0.isPressed()) {
            liftMotor0.setPower(0);
            liftMotor1.setPower(0);
            return;
        }

        // Need to make sure we are not still in RUN_TO_POSITION mode.
        liftMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor0.setPower(0.5);
        liftMotor1.setPower(0.5);
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

        liftMotor0.setPower(0.5);
        liftMotor1.setPower(0.5);
    }

    public void liftRight(double power) {
        liftMotor0.setPower(power);
    }

    public void liftLeft(double power) {
        liftMotor1.setPower(power);
    }

    public void displayLiftPositions() {
        telemetry.addData("Left Lift Position", liftMotor0.getCurrentPosition());
        telemetry.addData("Right Lift Position", liftMotor1.getCurrentPosition());

    }
}
