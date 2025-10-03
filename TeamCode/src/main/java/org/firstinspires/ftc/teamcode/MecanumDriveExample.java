package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveMechanism;


@TeleOp(name="Mecanum Drive, field or bot oriented", group="TeleOp Drive")
public class MecanumDriveExample extends OpMode {
    MecanumDriveMechanism drive = new MecanumDriveMechanism();;
    double forward, strafe, rotate;
    boolean fieldOriented = false;

    @Override
    public void init() {
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // toggles field oriented on/off without having to hold it down
        if (gamepad1.a) {
            fieldOriented = !fieldOriented;
        }

        if (fieldOriented) {
            drive.driveFieldOriented(forward, strafe, rotate);
        } else {
            drive.drive(forward, strafe, rotate);
        }
    }
}
