package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo carousel, kicker;

    // Color & Touch Sensors
    public NormalizedColorSensor colorSensor0, colorSensor1;

    // Shooter & Intake Motors
    public DcMotorEx shooterMotor;
    public DcMotor intakeMotor;

    private final double SHOOTER_TICKS_PER_REVOLUTION = 14; // or 28 * 0.5 gear ratio
    private double MAX_SHOOTER_RPM = 6000;

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

        carousel = hardwareMap.get(Servo.class, "carousel"); // expansion port: 0
        kicker = hardwareMap.get(Servo.class, "kicker"); // expansion port: 2
        kicker.setDirection(Servo.Direction.REVERSE);

        colorSensor0 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor0");
        colorSensor0.setGain(7);

        colorSensor1 = hardwareMap.get(NormalizedColorSensor.class, "colorSensor1");
        colorSensor1.setGain(7);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor"); // port 3.
        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Set the motor to use its encoder to maintain a target velocity.

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); // port: 0
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Rotates carousel clockwise to the next Slot intake position.
     * The order from lower (0.016) to higher (0.968)
     *  Slot C  --> Slot B --> Slot A
     */
    public void nextRightIntakePosition() {
        saveSlotColor();
        double currentPosition = carousel.getPosition() + 0.001;
        Slot nextSlot = null;

        // We want to move clockwise (right) to the next position
        // so that means we need to order our if/else condition to
        // evaluate the smallest intake positions first.
        // Intake positions are
        // - 0.016 (X): if current servo position is less than this, move to this position X
        // - 0.482 (B): if it is less than this, move to this position B
        // - 0.968 (A): if it is less than this, move to this position A
        // - if greater than A, go back to X.
        if (currentPosition < slotX.intakePosition) {
            nextSlot = slotX;
        } else if (currentPosition < slotB.intakePosition) {
            nextSlot = slotB;
        } else if (currentPosition < slotA.intakePosition) {
            nextSlot = slotA;
        } else {
            nextSlot = slotX;
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
        saveSlotColor();
        double currentPosition = carousel.getPosition() - 0.001;
        Slot nextSlot = null;

        if (currentPosition > slotB.intakePosition) {
            nextSlot = slotB;
        } else if (currentPosition > slotA.intakePosition) {
            nextSlot = slotA;
        } else if (currentPosition > slotX.intakePosition) {
            nextSlot = slotX;
        } else {
            nextSlot = slotB;
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
        saveSlotColor();
        double currentPosition = carousel.getPosition() + 0.001;
        Slot nextSlot = null;

        if (currentPosition < slotB.shootingPosition) {
            nextSlot = slotB;
        } else if (currentPosition < slotA.shootingPosition) {
            nextSlot = slotA;
        } else if (currentPosition < slotX.shootingPosition) {
            nextSlot = slotX;
        } else {
            nextSlot = slotB;
        }

        carousel.setPosition(nextSlot.shootingPosition);
        telemetry.addData("Shooting Set to Slot: ", nextSlot.name);

    }

    public void nextLeftShootingPosition() {
        saveSlotColor();
        double currentPosition = carousel.getPosition() - 0.001;
        Slot nextSlot = null;

        if (currentPosition > slotX.shootingPosition) {
            nextSlot = slotX;
        } else if (currentPosition > slotA.shootingPosition) {
            nextSlot = slotA;
        } else if (currentPosition > slotB.shootingPosition) {
            nextSlot = slotB;
        } else {
            nextSlot = slotX;
        }

        carousel.setPosition(nextSlot.shootingPosition);
        telemetry.addData("Shooting Set to Slot: ", nextSlot.name);
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
    private boolean intakeMotorIsOn = false; // starts as off
    public void turnIntakeMotorOnOff() {
        if (intakeMotorIsOn) {
            intakeMotorIsOn = false;
        } else {
            intakeMotorIsOn = true;
        }

        intakeMotor.setPower(intakeMotorIsOn ? 1 : 0);
    }

    /*********************************
     * Turns on/off the shooter motor.
     * - create a class field outside of the method to save our decision of on/off (starts as off)
     * - then create a method that updates the decision, then set's the motor on/of based on that
     ********************************/
    private boolean shooterMotorIsOn = false;  // this saves our decision. Starts as off (false)

    public void turnShooterOnOff(double power) {
        if (shooterMotorIsOn) {
            shooterMotorIsOn = false;
        } else {
            shooterMotorIsOn = true;
        }
        setShootingPower(power);
    }

    public void turnShooterOnOffByRpm(double rpm) {
        if (shooterMotorIsOn) {
            shooterMotorIsOn = false;
        } else {
            shooterMotorIsOn = true;
        }
        setShooterRPM(rpm);
    }

    // will set the shooting power above 0 if shooterMotorIsOn is true
    public void setShootingPower(double shootingPower) {
        if ( shootingPower < 0) {
            shootingPower = 0;
        }
        if (shootingPower > 1) {
            shootingPower = 1;
        }

        shooterMotor.setPower(shooterMotorIsOn ? shootingPower : 0);
    }

    /**
     * Calculates and returns the current speed of the shooter motor in Revolutions Per Minute (RPM).
     * @return The current motor speed in RPM.
     */
    public double getShooterRPM() {
        // getVelocity() returns the speed in "encoder ticks per second"
        double ticksPerSecond = shooterMotor.getVelocity();
        // Convert ticks per second to revolutions per minute (RPM)
        double revolutionsPerMinute = (ticksPerSecond / SHOOTER_TICKS_PER_REVOLUTION) * 60;
        return revolutionsPerMinute;
    }

    public void setShooterRPM(double rpm) {
        rpm = Math.min(rpm, MAX_SHOOTER_RPM);
        double ticksPerSecond = (rpm / 60.0) * SHOOTER_TICKS_PER_REVOLUTION;
        //telemetry.addLine("ticketPerSecond" + ticksPerSecond);
        shooterMotor.setVelocity(shooterMotorIsOn ? ticksPerSecond : 0);
    }

    public void kick(double power) {
        kicker.setPosition(power);
    }

    public void showCarouselData() {
        telemetry.addLine("Shooter RPM: " + (int)getShooterRPM());

        for (Slot slot : allSlots) {
            if (slot == null ) {
                continue;
            }

            String colorEmoji = NONE_EMOJI;
            if (slot.color == ClassificationColor.Green) {
                colorEmoji = GREEN_EMOJI;
            } else if (slot.color == ClassificationColor.Purple) {
                colorEmoji = PURPLE_EMOJI;
            }

            telemetry.addLine("Slot: " + slot.name + " " + colorEmoji);
            telemetry.addLine("  Ready for Intake: " + (slot.isReadyForIntake() ? READY_EMOJI : NOT_READY_EMOJI ));
            telemetry.addLine("  Ready for Shot: " + (slot.isReadyForShot() ? READY_EMOJI : NOT_READY_EMOJI));
        }

        // display if intake motor is on or off
        telemetry.addLine("Intake Motor: " + (intakeMotorIsOn ? "\uD83D\uDCA1" : "⚫"));
//        telemetry.addLine("Carousel Position: " + carousel.getPosition());
//        telemetry.addLine("Color at Intake: " + getClassificationColor().toString());
    }

    public void turnIntakeMotorOn() {
        intakeMotor.setPower(1);
    }

    public void turnIntakeMotorOff() {
        intakeMotor.setPower(0);
    }

    /*****************************************************************************************
     * NEW CONCEPT: Enums (sounds like "ee-num")
     *
     * Imagine you're creating a character for a video game and need to pick a special
     * power from a specific list, like Flying, Super-Speed, or Invisibility. An Enum
     * does exactly that! It lets us create our own custom "type" with a fixed list of
     * choices. We've created `ClassificationColor` which can ONLY be `Purple`, `Green`,
     * or `None`. This is awesome because it prevents typos and makes the code super
     * easy to read, turning our code's choices into a simple multiple-choice question!
     ****************************************************************************************/
    public enum ClassificationColor { Purple, Green, None}

    /*****************************************************************************************
     * NEW CONCEPT: Inner Classes (like a mini-class!)
     *
     * Imagine you're building a LEGO car. Instead of having a messy pile of tires and
     * rims, it's more organized to build each wheel first, then attach it to the car.
     *
     * This `Slot` class is our blueprint for an organized "wheel." We are building a
     * "Carousel" that has three "Slots," and each slot has its own data bundled
     * together neatly: a name, an intake position, and a shooting position.
     *
     * This makes our main code much cleaner. Instead of a jumble of variables, we
     * get to use organized objects like `slotA.intakePosition`!
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

        public boolean isReadyForShot(double currPosition) {
            // Check if the absolute difference between the two positions is within our tolerance.
            if (Math.abs(currPosition - this.shootingPosition) < tolerance) {
                return true;
            }
            return false;
        }

        public boolean isReadyForShot() {
            return isReadyForShot(carousel.getPosition());
        }

        public boolean isReadyForIntake(double currPosition) {
            // Check if the absolute difference between the two positions is within our tolerance.
            if (Math.abs(currPosition - this.intakePosition) < tolerance) {
                return true;
            }
            return false;
        }

        public boolean isReadyForIntake() {
            return isReadyForIntake(carousel.getPosition());
        }
    }

    private static final String PURPLE_EMOJI = "🟣";
    private static final String GREEN_EMOJI = "🟢";
    private static final String NONE_EMOJI = "⚪";
    private static final String READY_EMOJI = "✅";
    private static final String NOT_READY_EMOJI = "❌";

    private void readColor(NormalizedColorSensor colorSensor) {
        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
        telemetry.addData("Color is closest to", getClassificationColor().toString());
    }

    private void saveSlotColor() {
        double currPosition = carousel.getPosition();

        for (Slot slot : allSlots) {
            if (slot.isReadyForIntake(currPosition)) {
                slot.color = getClassificationColor();
            }
        }
    }

    private void removeslotXolor() {
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
