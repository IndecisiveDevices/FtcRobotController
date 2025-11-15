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

        liftMotor0 = hardwareMap.get(DcMotor.class, "liftMotor1"); // (right) port: 2?
        liftMotor1 = hardwareMap.get(DcMotor.class, "liftMotor0"); // (left) port: 1

        liftMotor0.setDirection(DcMotor.Direction.REVERSE);

        // makes sure that the motor encoder value starts at 0
        liftMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Have motors move with positive power until
    // touch sensor is pressed.
    public void liftUp(double power) {
        // if the touchSensor is pressed, we are at the top
        // and need to stop the motors so they don't cause damage.
        touchSensor0.getValue();
        if (touchSensor0.isPressed()) {
            stopLift();
            return;
        }

        // Need to make sure we are not still in RUN_TO_POSITION mode.
        liftMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor0.setPower(power);
        liftMotor1.setPower(power);
    }


    // Left motors to a target position
    // 7260 was the last logged.
    public void liftToPosition(int position, double power) {
        liftMotor0.setTargetPosition(position);
        liftMotor1.setTargetPosition(position);

        liftMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor0.setPower(power);
        liftMotor1.setPower(power);
    }

    // Left motors
    public void liftDown(double power) {
//        liftMotor0.setTargetPosition(0);
//        liftMotor1.setTargetPosition(0);
//
//        liftMotor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor0.setPower(-power);
        liftMotor1.setPower(-power);
    }

    public void stopLift() {
        liftMotor0.setPower(0);
        liftMotor1.setPower(0);
    }

    // ----------------------------------------------
    // Troubleshooting methods below here. Not used
    // for competition.
    // ----------------------------------------------
    public void liftRight(double power) {
        liftMotor1.setPower(power);
    }

    public void liftLeft(double power) {
        liftMotor0.setPower(power);
    }

    public void displayLiftPositions() {
        telemetry.addData("Left Lift Position", liftMotor0.getCurrentPosition());
        telemetry.addData("Right Lift Position", liftMotor1.getCurrentPosition());
        telemetry.addData("Touch Sensor is pressed", touchSensor0.isPressed());
    }
}
