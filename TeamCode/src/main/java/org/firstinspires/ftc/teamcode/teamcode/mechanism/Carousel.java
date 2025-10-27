package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Carousel {
    // Servos (carousel)
    private Servo carousel;

    // Color & Touch Sensors
    private ColorSensor colorSensor0, colorSensor1;

    // create Intake and Shooting positions for the carousel A, B, and C positions (2 each)
    private final double carouselPositionAintake = 0.968;
    private final double carouselPositionAshooter = 0.454;

    private final double carouselPositionBintake = 0.482;
    private final double carouselPositionBshooter = 0.000;

    private final double carouselPositionCintake = 0.016;
    private final double carouselPositionCshooter = 0.929;

    /*****************************************************************************************
     * NEW CONCEPT: Enums (sounds like "ee-num")
     *
     * Imagine you're creating a character for a video game and you need to choose their
     * special power: Flying, Super-Speed, or Invisibility. You want to make sure you can
     * ONLY pick from that specific list.
     *
     * That's exactly what an Enum does! It lets us create our own custom "type" with a
     * fixed list of possible values. Here, we've created a type called
     * `ClassificationColor` that can ONLY be `Purple`, `Green`, or `None`.
     *
     * Why is this cool?
     * 1. It prevents typos! You can't accidentally type "Purpul".
     * 2. It makes the code super easy to read. `if (pixelColor == ClassificationColor.Green)`
     *    is much clearer than `if (pixelColor == 2)`.
     *
     * Think of it as creating a multiple-choice question for our code!
     ****************************************************************************************/
    public enum ClassificationColor { Purple, Green, None}

    /*****************************************************************************************
     * NEW CONCEPT: Inner Classes (like a mini-class!)
     *
     * Imagine you're building a LEGO car. You need wheels, right? Each wheel is made of
     * a tire and a rim. Instead of having a messy pile of tires and rims, it's way
     * more organized to build each wheel first and then attach them to the car.
     *
     * An inner class is like that! We're building a "Carousel," and it has three "Slots"
     * (A, B, and C). Each slot has its own data: a name and an intake position, for example.
     *
     * This `Slot` class is our blueprint for making a perfect, organized "wheel."
     * We can create an "instance" (a real object) of this class for each of our carousel
     * slots. This keeps all the information for one slot bundled together neatly.
     *
     * It makes our main code much cleaner and easier to read. Instead of a jumble of
     * variables, we get to use organized objects like `slotA.intakePosition`!
     *
     * Better yet, we can add more fields to the class as we need. Are intake position and
     * name enough information about a Slot for our robot?
     ****************************************************************************************/
    private class Slot {
        public final double intakePosition;
        public final String name;

        public Slot(double intakePosition, String name) {
            this.intakePosition = intakePosition;
            this.name = name;
        }
    }
    // This is how you can create an "instance" of a Slot
    private Slot slotA = new Slot(carouselPositionAintake, "A");
    
    public void initialize(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(Servo.class, "carousel");
        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
    }

    public void nextIntakePosition() {
        double currentPosition = carousel.getPosition();

        // if (gamepad2.b) {
        //     carousel.setPosition(slotA.intakePosition);
        // } else if (gamepad2.x) {
        //     carousel.setPosition(slotB.intakePosition);
        // }

    }

    public void previousIntakePosition() { }

    public void showCarouselData() {
        telemetry.addData("carousel position", "%.3f", carousel.getPosition());
    }

    double currentCarouselPosition = 0.5;
    public void moveCarouselClockwise() {
        currentCarouselPosition += 0.001;
        currentCarouselPosition = Math.min(currentCarouselPosition, 1.0);

        carousel.setPosition(currentCarouselPosition);
        telemetry.addData("Current Carousel Position", currentCarouselPosition);
    }

    public void moveCarouselCounterClockwise() {
        currentCarouselPosition -= 0.001;
        currentCarouselPosition = Math.max(currentCarouselPosition, 0.0);
        carousel.setPosition(currentCarouselPosition);
        telemetry.addData("Current Carousel Position", currentCarouselPosition);
    }

}
