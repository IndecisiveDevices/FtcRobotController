package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Carousel {
    private Telemetry telemetry;

    // Servos (carousel, kicker)
    private Servo carousel, kicker;

    // Color & Touch Sensors
    private NormalizedColorSensor colorSensor0, colorSensor1;

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

    public final Slot[] allSlots = {slotA, slotB, slotX};

    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        carousel = hardwareMap.get(Servo.class, "carousel");
        kicker = hardwareMap.get(Servo.class, "kicker");
        kicker.setDirection(Servo.Direction.REVERSE);

        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensor0.setGain(7);

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor1.setGain(7);

        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setTargetPosition(1);
    }

    public void gotoIntakeA() {
        saveSlotColor();
        carousel.setPosition(slotA.intakePosition);
    }

    public void gotoIntakeB() {
        saveSlotColor();
        carousel.setPosition(slotB.intakePosition);
    }

    public void gotoIntakeX() {
        saveSlotColor();
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
        // TODO: Replace with code
    }

    public void setShootingPower(double shootingPower) {
        shooterMotor.setPower(shootingPower);
    }

    public void kick(double power) {
        kicker.setPosition(power);
    }

    public void showCarouselData() {
        for (Slot slot : allSlots) {
            if (slot == null ) {
                continue;
            }
            telemetry.addLine("Slot: " + slot.name);
            telemetry.addLine("  Color: " + slot.color.toString());

            if (slot.isReadyForIntake()) {
                telemetry.addLine("  Ready for Intake: " + "✅");
            } else {
                telemetry.addLine("  Ready for Intake: " + "❌");
            }

            if (slot.isReadyForShot()) {
                telemetry.addLine("  Ready for Shot: " + "✅");
            } else {
                telemetry.addLine("  Ready for Shot: " + "❌");
            }
        }

        // display if intake motor is on or off
        telemetry.addLine("Intake Motor: " + (intakeMotorIsOn ? "ON" : "OFF"));
        telemetry.addLine("Carousel Position: " + carousel.getPosition());
        telemetry.addLine("Color at Intake: " + getClassificationColor().toString());
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

        public boolean isReadyForShot() {
            // Check if the absolute difference between the two positions is within our tolerance.
            if (Math.abs(carousel.getPosition() - this.shootingPosition) < tolerance) {
                return true;
            }
            return false;
        }

        public boolean isReadyForIntake() {
            // Check if the absolute difference between the two positions is within our tolerance.
            if (Math.abs(carousel.getPosition() - this.intakePosition) < tolerance) {
                return true;
            }
            return false;
        }
    }


    private void readColor(NormalizedColorSensor colorSensor) {
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
        telemetry.addData("Color is closest to", getClassificationColor().toString());
    }

    private void saveSlotColor() {
        for (Slot slot : allSlots) {
            if (slot.isReadyForIntake()) {
                slot.color = getClassificationColor();
            }
        }
    }

    private void removeSlotColor() {
        for (Slot slot : allSlots) {
            if (slot.isReadyForShot()) {
                slot.color = ClassificationColor.None;
            }
        }
    }

    private ClassificationColor getClassificationColor() {
        // Determine if colorSensor0 or colorSensor1 should be used for classification
        // based on which one has the highest light detected
        NormalizedRGBA color1 = colorSensor0.getNormalizedColors();
        NormalizedRGBA color2 = colorSensor1.getNormalizedColors();

        // determine if we have enough light to detect a color. if not, then return None
        double lightDetected1 = ((OpticalDistanceSensor) colorSensor0).getLightDetected();
        double lightDetected2 = ((OpticalDistanceSensor) colorSensor1).getLightDetected();
        if (Math.max(lightDetected1, lightDetected2) < 0.060) {
            return ClassificationColor.None;
        }

        double hue;
        if (lightDetected1 > lightDetected2) {
            hue = JavaUtil.colorToHue(color1.toColor());
        } else {
            hue = JavaUtil.colorToHue(color2.toColor());
        }

        double greenDistance = Math.abs(hue - 150);
        double purpleDistance = Math.abs(hue - 225);

        // now we can compare the two distances and determine which is the smallest
        if (greenDistance < purpleDistance){
            return ClassificationColor.Green;
        }
        else {
            return ClassificationColor.Purple;
        }
    }

}
