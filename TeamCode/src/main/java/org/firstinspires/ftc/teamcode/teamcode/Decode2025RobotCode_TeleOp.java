package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;

@TeleOp(name = "Decode2025RobotCode_TeleOp", group = "Robot")
public class Decode2025RobotCode_TeleOp extends OpMode {
    MecanumDrive driver = new MecanumDrive();
    Lift lifter = new Lift();
    Carousel carousel = new Carousel();

    // Servos (carousel, kicker)
    private Servo kicker;

    // Shooter & Intake Motors
    private DcMotor shooterMotor, intakeMotor;

    @Override
    public void init() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap);
        carousel.initialize(hardwareMap);

        initServos();
        initShooterMotors();
    }

    private void initShooterMotors() {
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    private void initServos() {
        kicker = hardwareMap.get(Servo.class, "kicker");
    }

    @Override
    public void loop() {
        //----------------------------
        // Drive Code (Done)
        //----------------------------
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driver.drive(forward, strafe, rotate);

        //----------------------------
        // Lift Code (TODO)
        //----------------------------


        //----------------------------
        // Intake Code (TODO)
        //----------------------------
        if (gamepad2.a){
            carousel.nextIntakePosition();
        } else if (gamepad2.x){
            carousel.previousIntakePosition();
        }

        //----------------------------
        // Shooting Code (TODO)
        //----------------------------


        //----------------------------
        // Telemetry Update (DONE)
        //----------------------------
        telemetry.update();
    }
}
