package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;


@TeleOp(name="Ed's Decode Example", group="TeleOp Drive")
public class DecodeCompExample extends OpMode {
    MecanumDrive drive = new MecanumDrive();
    Shooter shooter = new Shooter();

    double forward, strafe, rotate;
    boolean fieldOriented = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
        shooter.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Left stick gives us a negative value when pushed forward (up) and positive when pulled
        // back (down). We want to flip this so pressing forward makes the bot move forward
        // (positive value) instead of backwards (negative value).
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // toggles field oriented on/off without having to hold it down
        if (gamepad1.a) {
            fieldOriented = !fieldOriented;
        }
        if (gamepad1.dpad_up) {
            drive.bumpSpeed(0.1);
        }
        if (gamepad1.dpad_down) {
            drive.bumpSpeed(-0.1);
        }

        telemetry.addData("fieldOriented", String.valueOf(fieldOriented));
        telemetry.addData("current speed", String.valueOf(drive.getSpeed()));
        telemetry.update();

        if (fieldOriented) {
            drive.driveFieldOriented(forward, strafe, rotate);
        } else {
            drive.drive(forward, strafe, rotate);
        }

        if (gamepad1.left_bumper) {
            readyIntakeSlot();
        } else {
            readyShot();
        }

        shooter.displayActiveSlots();

        if (gamepad1.right_trigger >= 0.03) {
            shooter.shoot();
        }
    }

    public void readyIntakeSlot() {
        if (gamepad1.x) {
            shooter.moveSlotToIntake(Shooter.CarouselSlot.X);
        }
        else if (gamepad1.y) {
            shooter.moveSlotToIntake(Shooter.CarouselSlot.Y);
        }
        else if (gamepad1.b) {
            shooter.moveSlotToIntake(Shooter.CarouselSlot.B);
        }
    }

    public void readyShot() {
        if (gamepad1.x) {
            shooter.moveSlotToShoot(Shooter.CarouselSlot.X);
        }
        else if (gamepad1.y) {
            shooter.moveSlotToShoot(Shooter.CarouselSlot.Y);
        }
        else if (gamepad1.b) {
            shooter.moveSlotToShoot(Shooter.CarouselSlot.B);
        }
    }
}
