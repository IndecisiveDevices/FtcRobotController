package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveMechanism;


@TeleOp(name="Mecanum Drive, field or bot oriented", group="TeleOp Drive")
public class MecanumDriveExample extends OpMode {
    MecanumDriveMechanism drive = new MecanumDriveMechanism();;
    double forward, strafe, rotate;
    boolean fieldOriented = false;
    boolean inverseStrafe = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
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

        telemetry.addData("fieldOriented", String.valueOf(fieldOriented));
        telemetry.addData("inverseStrafe", String.valueOf(inverseStrafe));
        telemetry.update();

        if (fieldOriented) {
            drive.driveFieldOriented(forward, strafe, rotate);
        } else {
            drive.drive(forward, strafe, rotate);
        }
    }
}
