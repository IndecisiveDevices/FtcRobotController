package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    private Telemetry telemetry;

    // Servos (carousel, kicker)
    private Servo carousel, kicker;

    // Color & Touch Sensors
    private ColorSensor colorSensor0, colorSensor1;

    // Shooter & Intake Motors
    private DcMotor shooterMotor, intakeMotor;

    // create Intake and Shooting positions for the carousel A, B, and C positions (2 each)
    private final double A_INTAKE_POSITION = 0.968;
    private final double B_INTAKE_POSITION = 0.482;
    private final double X_INTAKE_POSITION = 0.016;

    private final double A_SHOOTING_POSITION = 0.434;
    private final double B_SHOOTING_POSITION = 0.000;
    private final double X_SHOOTING_POSITION = 0.900;

    // This is how you can create an "instance" of a Slot
    public final Slot slotA = new Slot(A_INTAKE_POSITION, A_SHOOTING_POSITION, "A");
    public final Slot slotB = new Slot(B_INTAKE_POSITION, B_SHOOTING_POSITION, "B");
    public final Slot slotX = new Slot(X_INTAKE_POSITION, X_SHOOTING_POSITION, "X");

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setDirection(Servo.Direction.REVERSE);

        colorSensor0 = hardwareMap.get(ColorSensor.class, "colorSensor0");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setTargetPosition(1);
    }

    public void gotoIntakeA() {
        carousel.setPosition(slotA.intakePosition);
    }

    public void gotoIntakeB() {
        carousel.setPosition(slotB.intakePosition);
    }

    public void gotoIntakeX() {
        carousel.setPosition(slotX.intakePosition);
    }

    public void gotoShootingA() {
        carousel.setPosition(slotA.shootingPosition);
    }

    public void gotoShootingB() {
        carousel.setPosition(slotB.shootingPosition);
    }
    public void gotoShootingX() {
        carousel.setPosition(slotX.shootingPosition);
    }

    /**
     * Turns on/off the intake motor.
     */
    boolean intakeMotorIsOn = false; // starts as off
    public void turnIntakeMotorOnOff() {

    }

    public void setShootingPower(double shootingPower) {
        shooterMotor.setPower(shootingPower);
    }

    public void kick(double power) {
        kicker.setPosition(power);
    }

    public void showCarouselData() {
        //array of slots
        Slot[] slots = {slotA, slotB, slotX};

        for (Slot slot : slots) {
            if (slot == null ) {
                continue;
            }
            telemetry.addLine("Slot: " + slot.name);
            telemetry.addLine("  Color: " + slot.color.toString());
            telemetry.addLine("  Ready for Intake: " + slot.readyForIntake());
            telemetry.addLine("  Ready for Shot: " + slot.readyForShot());
        }

        // display if intake motor is on or off
        telemetry.addLine("Intake Motor: " + (intakeMotorIsOn ? "ON" : "OFF"));
        telemetry.addLine("Carousel Position: " + carousel.getPosition());
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
    public class Slot {
        public final double intakePosition;
        public final double shootingPosition;
        public final String name;
        public ClassificationColor color;

        // this is called a "constructor". This say that when you create a new Slot, you have to
        // give it an intake position, shooting position, and a name. Example:
        //   Slot slotA = new Slot(0.968, 0.434, "A");
        public Slot(double intakePosition, double shootingPosition, String name) {
            this.intakePosition = intakePosition;
            this.shootingPosition = shootingPosition;
            this.name = name;
            this.color = ClassificationColor.None;
        }

        // Define a small tolerance for our comparison.
        // 0.001 is a good starting point since servo positions are usually between 0.0 and 1.0.
        private static final double tolerance = 0.001;

        public String readyForShot() {
            // Check if the absolute difference between the two positions is within our tolerance.
            if (Math.abs(carousel.getPosition() - this.shootingPosition) < tolerance) {
                return "✅ Ready";
            }
            return "❌ No";
        }

        public String readyForIntake() {
            // Check if the absolute difference between the two positions is within our tolerance.
            if (Math.abs(carousel.getPosition() - this.intakePosition) < tolerance) {
                return "✅ Ready";
            }
            return "❌ No";
        }
    }
}
