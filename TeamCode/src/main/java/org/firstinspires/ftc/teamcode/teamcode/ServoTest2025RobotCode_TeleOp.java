package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "ServoTest2025RobotCode_TeleOp", group = "Robot")
public class ServoTest2025RobotCode_TeleOp extends OpMode {

    NormalizedColorSensor colorSensor0;
    Servo carousel;
    Servo kicker;
    TouchSensor touchSensor;

    @Override
    public void init() {
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");

        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensor0.setGain(7);

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        telemetry.addData("Status", "Initialized says Teammate");
        telemetry.update();
    }

    private void testTouchSensor() {
        telemetry.addData("Touch Sensor get value", touchSensor.getValue());
        telemetry.addData("Touch Sensor ", touchSensor.isPressed());
    }

    @Override
    public void loop() {
        testTouchSensor();
        testColorSensors();

        if (gamepad1.x) {
            carousel.setPosition(0.5);
        }
        if (gamepad1.y) {
            carousel.setPosition(0.0);
        }

        if (gamepad1.a) {
            kicker.setPosition(0.5);
        }
        kicker.setPosition(0.0);
        telemetry.update();
    }

    public void testColorSensors() {
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor0).getLightDetected());
        NormalizedRGBA colors = colorSensor0.getNormalizedColors();

        //Determining the amount of red, green, and blue
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        //Determining HSV and alpha
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));

        double hue = JavaUtil.colorToHue(colors.toColor());
        String color;
        if(hue < 30) color = "Red";
        else if (hue < 60) color = "Orange";
        else if (hue < 90) color = "Yellow";
        else if (hue < 150) color = "Green";
        else if (hue < 225) color = "Blue";
        else if (hue < 350) color = "Purple";
        else color = "Red";

        telemetry.addData("Color Hue", color);

        double greenDistance = Math.abs(hue - 150);
        double purpleDistance = Math.abs(hue - 225);

        // now we can compare the two distances and determine which is the smallest
        if (greenDistance < purpleDistance){
            telemetry.addData("Color is closer to", "Green");
        }
        else{
            telemetry.addData("Color is closer to", "Purple");
        }

    }
}


