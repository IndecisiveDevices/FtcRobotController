package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Carousel {
    // Servos (carousel, kicker)
    private Servo carousel, kicker;

    // Color & Touch Sensors
    private ColorSensor colorSensor0, colorSensor1;

    // Shooter & Intake Motors
    private DcMotor shooterMotor, intakeMotor;

    // create Intake and Shooting positions for the carousel A, B, and C positions (2 each)
    private final double A_INTAKE_POSITION = 0.968;
    private final double B_INTAKE_POSITION = 0.482;
    private final double C_INTAKE_POSITION = 0.016;

    private final double A_SHOOTING_POSITION = 0.454;
    private final double B_SHOOTING_POSITION = 0.000;
    private final double C_SHOOTING_POSITION = 0.929;

    // This is how you can create an "instance" of a Slot
    private Slot slotA = new Slot(A_INTAKE_POSITION, A_SHOOTING_POSITION, "A");
    private Slot slotB = new Slot(B_INTAKE_POSITION, B_SHOOTING_POSITION, "B");
    private Slot slotC = new Slot(C_INTAKE_POSITION, C_SHOOTING_POSITION, "C");

    public void initialize(HardwareMap hardwareMap) {
        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");

        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    /**
     * Turns on/off the intake motor.
     */
    boolean intakeMotorIsOn = false; // starts as off
    public void turnIntakeMotorOnOff() {

    }

    /**
     * Rotates carousel clockwise to the next Slot intake position.
     * The order from lower (0.016) to higher (0.968)
     *  Slot C  --> Slot B --> Slot A
     * Intake positions are
     * - 0.016 (C): if current servo position is less than this, move to this position C
     * - 0.482 (B): if it is less than this, move to this position B
     * - 0.968 (A): if it is less than this, move to this position A
     */
    public void nextRightIntakePosition() {
        double currentPosition = carousel.getPosition();
        Slot nextSlot = null;

        // We want to move clockwise (right) to the next position
        // so that means we need to order our if/else condition to
        // evaluate the smallest intake positions first.

        if (currentPosition < slotC.intakePosition) {
            nextSlot = slotC;
        } else if (currentPosition < slotB.intakePosition) {
            nextSlot = slotB;
        } else { // Slot A is the right-most we can go to.
            nextSlot = slotA;
        }

        carousel.setPosition(nextSlot.intakePosition);
        telemetry.addData("Intake Set to Slot: ", nextSlot.name);
    }

    /**
     * Rotates carousel counter-clockwise to the next Slot intake position.
     * The order from lower (0.000) to higher (0.929)
     *  Slot B <--  Slot A <-- Slot C
     */
    public void nextLeftIntakePosition() {

        double currentPosition = carousel.getPosition();
        Slot nextSlot = null;

        if (currentPosition > slotB.intakePosition) {
            nextSlot = slotB;
        } else if (currentPosition > slotA.intakePosition) {
            nextSlot = slotA;
        } else if (currentPosition > slotC.intakePosition) {
            nextSlot = slotC;
        }

        carousel.setPosition(nextSlot.intakePosition);
        telemetry.addData( "Intake Set to Slot: ",  nextSlot.name);
    }


    /**
     * Rotates carousel counter-clockwise to the next Slot intake position.
     * The order from lower (0.000) to higher (0.929)
     *  Slot B <--  Slot A <-- Slot C
     */
    public void nextRightShootingPosition() {
        double currentPosition = carousel.getPosition();
        Slot nextSlot = null;

        if (currentPosition < slotB.shootingPosition) {
            nextSlot = slotB;
        } else if (currentPosition < slotA.shootingPosition) {
            nextSlot = slotA;
        } else { // Slot A is the right-most we can go to.
            nextSlot = slotC;
        }

        carousel.setPosition(nextSlot.shootingPosition);
        telemetry.addData("Shooting Set to Slot: ", nextSlot.name);

    }

    public void nextLeftShootingPosition() {
        double currentPosition = carousel.getPosition();
        Slot nextSlot = null;

        if (currentPosition > slotC.shootingPosition) {
            nextSlot = slotC;
        } else if (currentPosition > slotA.shootingPosition) {
            nextSlot = slotA;
        } else { // (currentPosition > slotB.shootingPosition)
            nextSlot = slotB;
        }

        carousel.setPosition(nextSlot.shootingPosition);
        telemetry.addData("Shooting Set to Slot: ", nextSlot.name);
    }

    public void setShootingPower(double shootingPower) {
        shooterMotor.setPower(shootingPower);
    }

    public void kick() {
        kicker.setPosition(1.00);
    }

    public void showCarouselData() {
        //array of slots
        Slot[] slots = {slotA, slotB, slotC};

        for (Slot slot : slots) {
            telemetry.addLine("Slot: " + slot.name);
            telemetry.addLine("  Color: " + slot.color.toString());
            telemetry.addLine("  Ready for Intake: " + slot.readyForIntake());
            telemetry.addLine("  Ready for Shot: " + slot.readyForShot());
        }

        // display if intake motor is on or off
        telemetry.addLine("Intake Motor: " + (intakeMotorIsOn ? "ON" : "OFF"));

    }

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
        public final double shootingPosition;
        public final String name;
        public ClassificationColor color;

        public Slot(double intakePosition, double shootingPosition, String name) {
            this.intakePosition = intakePosition;
            this.shootingPosition = shootingPosition;
            this.name = name;
            this.color = ClassificationColor.None;
        }

        public String readyForShot() {
            if (carousel.getPosition() == this.shootingPosition) {
                return "✅ Ready";
            }
            return "❌ No";
        }

        public String readyForIntake() {
            if (carousel.getPosition() == this.intakePosition) {
                return "✅ Ready";
            }
            return "❌ No";
        }
    }


    /*----------------------------------------------------
    The code below here is used to help determine the
    shooting and intake positions when calibrating the
    robot if the carousel hardware position changes.
    ------------------------------------------------------ */
    double testCarouselPosition = 0.5;
    public void moveCarouselClockwiseSlowly() {
        testCarouselPosition += 0.001;
        testCarouselPosition = Math.min(testCarouselPosition, 1.0);

        carousel.setPosition(testCarouselPosition);
        telemetry.addData("Current Carousel Position", testCarouselPosition);
    }

    public void moveCarouselCounterClockwiseSlowly() {
        testCarouselPosition -= 0.001;
        testCarouselPosition = Math.max(testCarouselPosition, 0.0);
        carousel.setPosition(testCarouselPosition);
        telemetry.addData("Current Carousel Position", testCarouselPosition);
    }

}
