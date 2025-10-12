package org.firstinspires.ftc.teamcode.mechanisms;


import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public class Shooter {
    static final double SLOT_X_INTAKE_POS = 0.0;
    static final double SLOT_Y_INTAKE_POS = 0.5;
    static final double SLOT_B_INTAKE_POS = 1.0;

    public enum CarouselSlot {
        X, Y, B
    }

    Servo carousel, kicker;

    public void init(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");
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

        CarouselSlot intakeSlot = null;
        CarouselSlot shootingSlot = null;

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
        telemetry.update();
    }
}
