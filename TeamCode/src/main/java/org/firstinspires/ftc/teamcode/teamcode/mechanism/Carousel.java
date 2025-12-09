package org.firstinspires.ftc.teamcode.teamcode.mechanism;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public final Slot[] allSlots = {slotB, slotA, slotX, };

    // p, i, d, f coefficients
    public PIDCoefficients pidCoefficients = new PIDCoefficients(
            100,
            1.05,
            3.0);
//    public PIDCoefficients pidCoefficients = new PIDCoefficients(
//            100,
//            0.85,
//            3.0);

    double f = 0.0009434;

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


        shooterMotor.setVelocityPIDFCoefficients(
                pidCoefficients.p,
                pidCoefficients.i,
                pidCoefficients.d,
                f);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor"); // port: 0
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    /**
     * Rotates carousel clockwise to the next Slot intake position.
     * The order from lower (0.016) to higher (0.968)
     *  Slot C  --> Slot B --> Slot A
     */
    public void nextRightIntakePosition() {
        double currentPosition = carousel.getPosition() + 0.001;

        // We want to move clockwise (right) to the next position
        // so that means we need to order our if/else condition to
        // evaluate the smallest intake positions first.
        // Intake positions are
        // - 0.016 (X): if current servo position is less than this, move to this position X
        // - 0.482 (B): if it is less than this, move to this position B
        // - 0.968 (A): if it is less than this, move to this position A
        // - if greater than A, go back to X.
        if (currentPosition < slotX.intakePosition) {
            gotoIntakeX();
        } else if (currentPosition < slotB.intakePosition) {
            gotoIntakeB();
        } else if (currentPosition < slotA.intakePosition) {
            gotoIntakeA();
        } else {
            gotoIntakeX();
        }

    }

    /**
     * Rotates carousel counter-clockwise to the next Slot intake position.
     * The order from lower (0.000) to higher (0.929)
     *  Slot B <--  Slot A <-- Slot C
     */
    public void nextLeftIntakePosition() {
        double currentPosition = carousel.getPosition() - 0.001;

        if (currentPosition > slotA.intakePosition) {
            gotoIntakeA();
        } else if (currentPosition > slotB.intakePosition) {
            gotoIntakeB();
        } else  if (currentPosition > slotX.intakePosition) {
            gotoIntakeX();
        } else {
            gotoIntakeA();
        }
    }


    /**
     * Rotates carousel counter-clockwise to the next Slot intake position.
     * The order from lower (0.000) to higher (0.929)
     *  Slot B <--  Slot A <-- Slot C
     */
    public void nextRightShootingPosition() {
        double currentPosition = carousel.getPosition() + 0.001;

        if (currentPosition < slotB.shootingPosition) {
            gotoShootingB();
        } else if (currentPosition < slotA.shootingPosition) {
            gotoShootingA();
        } else if (currentPosition < slotX.shootingPosition) {
            gotoShootingX();
        } else  {
            gotoShootingB();
        }
    }

    public void nextLeftShootingPosition() {
        double currentPosition = carousel.getPosition() - 0.001;

        if (currentPosition > slotX.shootingPosition) {
            gotoShootingX();
        } else if (currentPosition > slotA.shootingPosition) {
            gotoShootingA();
        } else if (currentPosition > slotB.shootingPosition) {
            gotoShootingB();
        } else {
            gotoShootingX();
        }
    }

    public void gotoIntakeA() {
        setCarouselPosition(slotA.intakePosition, true);
    }

    public void gotoIntakeB() {
        setCarouselPosition(slotB.intakePosition, true);
    }

    public void gotoIntakeX() {
        setCarouselPosition(slotX.intakePosition, true);
    }

    public void gotoShootingA() {
        setCarouselPosition(slotA.shootingPosition, false);
    }

    public void gotoShootingB() {
        setCarouselPosition(slotB.shootingPosition, false);
    }

    public void gotoShootingX() {
        setCarouselPosition(slotX.shootingPosition, false);
    }

    ElapsedTime servoTimer = new ElapsedTime();
    int maxServoTime = 1197; // 1.197 seconds for 280 degrees.
    int currentTimerSetting = 0;

    private void setCarouselPosition(double position, boolean saveSlotColor) {
        if (servoTimer.milliseconds() >= currentTimerSetting) {
            //if (saveSlotColor) {
                saveSlotColor();
            //};
            // get difference between current position (0.020) and incoming position (0.080), represented as milliseconds (80)
            currentTimerSetting = (int)(Math.abs(carousel.getPosition() - position) * maxServoTime);
            carousel.setPosition(position);
            servoTimer.reset();
        }
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

    double targetRpm = 0;
    public void setShooterRPM(double rpm) {
        targetRpm = Math.min(rpm, MAX_SHOOTER_RPM);
        double ticksPerSecond = (targetRpm / 60.0) * SHOOTER_TICKS_PER_REVOLUTION;
        shooterMotor.setVelocity(shooterMotorIsOn ? ticksPerSecond : 0);
    }

    public void kick(double power) {
        kicker.setPosition(power);
        if (power > 0) {
            removeSlotColor();
        }
    }

    // function to determine if the incoming RPM is within the acceptable range
    // to fire the shooter. Acceptable range is within 65 RPM of the currentRpm value.
    // takes arguments "currentRpm" and "measuredRpm"
    public boolean targetRpmReached() {
        final double ACCEPTABLE_RANGE = 75.0;
        if (targetRpm < 100) {
            return false;
        }
        return Math.abs(targetRpm - getShooterRPM()) <= ACCEPTABLE_RANGE;
    }

    public void showCarouselData(boolean isShooting) {
        telemetry.addLine("Ready to Fire: " + (targetRpmReached() ? "✅" : "❌"));
        telemetry.addLine(
                "Intake On: " + (intakeMotorIsOn ? "\uD83D\uDCA1" : "⚫") +
                        "   |    " +
                        "Shooter RPM: " + (int)getShooterRPM()
        );
        telemetry.addLine(" ");

        for (Slot slot : allSlots) {
            if (slot == null ) {
                continue;
            }

            String colorEmoji = "";
            if (slot.color == ClassificationColor.Green) {
                colorEmoji = GREEN_EMOJI;
            } else if (slot.color == ClassificationColor.Purple) {
                colorEmoji = PURPLE_EMOJI;
            } else if (slot.color == ClassificationColor.None) {
                colorEmoji = NONE_EMOJI;
            }

            String shootIntakeMsg = "";

            if (isShooting) {
                shootIntakeMsg = "Shoot: " + (slot.isReadyForShot() ? READY_EMOJI : NOT_READY_EMOJI);
            } else {
                shootIntakeMsg = "Intake: " + (slot.isReadyForIntake() ? READY_EMOJI : NOT_READY_EMOJI);
            }


            telemetry.addLine("Slot: " + slot.name + " " + colorEmoji + "    |    " + shootIntakeMsg);
            telemetry.addLine(" ");
        }
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
        private static final double tolerance = 0.0005;

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

    public void saveSlotColor() {
        double currPosition = carousel.getPosition();
        ClassificationColor color = getClassificationColor();

        for (Slot slot : allSlots) {
            if (slot.isReadyForIntake(currPosition)) {
                slot.color = color;
                //sleep(2500);
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
    public void setLongRangePidCoefficients(){
//        shooterMotor.setPIDFCoefficients(
//                longRangeCoefficients.p,
//                longRangeCoefficients.i,
//                longRangeCoefficients.d,
//                f
//        );
    }

    public void setShortRangePidCoefficients(){

    }
}
