package org.firstinspires.ftc.teamcode.mechanisms;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/*
    Shooter class controls the carousel and kicker servos
    as well as the shooting wheel's motor.
 */
public class Shooter {
    /*************************************
     * Servos
     ************************************/
    Servo carousel, kicker;
    // These values will most likely need to be changed after we program the carousel
    // servo LEFT and RIGHT ranges using the REV Servo Programmer
    static final double SLOT_X_INTAKE_POS = 0.0;
    static final double SLOT_Y_INTAKE_POS = 0.5;
    static final double SLOT_B_INTAKE_POS = 1.0;

    // Same as above, but for the kicker.
    static final double KICKER_UP_POS = 1.0;
    static final double KICKER_DOWN_POS = 0.0;

    public enum CarouselSlot {
        X, Y, B
    }

    /*************************************
     * Color Sensors
     ************************************/
    NormalizedColorSensor colorSensor0, colorSensor1;
    double hue;


    /*************************************
     * Telemetry
     ************************************/
    Telemetry telemetry = null;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");

        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensor0.setGain(7);

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor1.setGain(7);

        carousel.setPosition(0.5); // we'll start in middle position.
        kicker.setPosition(KICKER_DOWN_POS);
    }

    public void moveSlotToIntake(CarouselSlot slot) {
        if (slot == CarouselSlot.X) {
            carousel.setPosition(SLOT_X_INTAKE_POS);
        }
        if (slot == CarouselSlot.Y) {
            carousel.setPosition(SLOT_Y_INTAKE_POS);
        }
        if (slot == CarouselSlot.B) {
            carousel.setPosition(SLOT_B_INTAKE_POS);
        }

    }

    // Shooting position is going to be the same
    // value as the slot to the right of the intake
    public void moveSlotToShoot(CarouselSlot slot) {
        if (slot == CarouselSlot.X) {
            carousel.setPosition(SLOT_Y_INTAKE_POS);
        }
        if (slot == CarouselSlot.Y) {
            carousel.setPosition(SLOT_B_INTAKE_POS);
        }
        if (slot == CarouselSlot.B) {
            carousel.setPosition(SLOT_X_INTAKE_POS);
        }
    }

    public void shoot() {
        kicker.setPosition(1.0);
        sleep(500);
        kicker.setPosition(0.0);
    }

    public void displayActiveSlots() {
        // determine if the current carousel position is closer to the
        // SLOT_X_INTAKE_POS, SLOT_B_INTAKE_POS, or SLOT_Y_INTAKE_POS value
        double currentPosition = carousel.getPosition();
        double xDistance = Math.abs(currentPosition - SLOT_X_INTAKE_POS);
        double yDistance = Math.abs(currentPosition - SLOT_Y_INTAKE_POS);
        double bDistance = Math.abs(currentPosition - SLOT_B_INTAKE_POS);
        // add the distance values to a list or array and sort it to find the smallest value
        double[] distances = {xDistance, yDistance, bDistance};
        Arrays.sort(distances);
        // now set a double for the smallest distance from the sorted array
        double minDistance = distances[0];

        CarouselSlot intakeSlot = CarouselSlot.X;
        CarouselSlot shootingSlot = CarouselSlot.Y;

        if (minDistance == xDistance) {
            intakeSlot = CarouselSlot.X;
            shootingSlot = CarouselSlot.Y;
        }
        else if (minDistance == yDistance) {
            intakeSlot = CarouselSlot.Y;
            shootingSlot = CarouselSlot.B;
        }
        else {
            intakeSlot = CarouselSlot.B;
            shootingSlot = CarouselSlot.X;
        }

        telemetry.addData("Intake Slot", intakeSlot.toString());
        telemetry.addData("Shooting Slot", shootingSlot.toString());
        telemetry.addData("Current Position", currentPosition);
        telemetry.update();
    }

    private void readColor(NormalizedColorSensor colorSensor) {
        // TODO: use the 2 color sensors
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor()); // <------ New code

        //Determining the amount of red, green, and blue
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        //Determining HSV and alpha
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        String color;
        if      (hue < 30) color = "Red";
        else if (hue < 60)  color = "Orange";
        else if (hue < 90) color = "Yellow";
        else if (hue < 150) color = "Green";
        else if (hue < 225) color = "Blue";
        else if (hue < 350) color = "Purple";
        else color = "Red";

        telemetry.addData("Color Hue", color);

        telemetry.addData("Color is closest to", getClosestClassificationColor());
    }

    public String getClosestClassificationColor() {
        // Determine if colorSensor0 or colorSensor1 should be used for classification
        // based on which one has the highest light detected
        NormalizedRGBA color1 = colorSensor0.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor1.getNormalizedColors();
        double hue;
        if (color1.alpha > color2.alpha) {
            hue = JavaUtil.colorToHue(color1.toColor());
        } else {
            hue = JavaUtil.colorToHue(color2.toColor());
        }

        // Define the target hue values
        double greenTargetHue = 150;
        double purpleTargetHue = 225;

        // Calculate the shortest distance on a circle for Green
        double shortestGreenDistance = getShortest360Distance(hue, greenTargetHue);

        // Calculate the shortest distance on a circle for Purple
        double shortestPurpleDistance = getShortest360Distance(hue, purpleTargetHue);

        telemetry.addData("Shortest Green Dist", "%.2f", shortestGreenDistance);
        telemetry.addData("Shortest Purple Dist", "%.2f", shortestPurpleDistance);

        // now we can compare the two shortest distances
        if (shortestGreenDistance < shortestPurpleDistance){
            return "Green";
        }

        return "Purple";
    }

    private double getShortest360Distance(double readingHue, double targetHue) {
        double dist1 = Math.abs(readingHue - targetHue);
        double dist2 = 360 - dist1; // The wrap-around distance
        return Math.min(dist1, dist2);
    }

}
