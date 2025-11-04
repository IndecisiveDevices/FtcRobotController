package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamcode.mechanism.Carousel;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.Lift;
import org.firstinspires.ftc.teamcode.teamcode.mechanism.MecanumDrive;

// Do a search for "RobotAutoDriveToAprilTagOmni.java" to see what we can copy
// and paste it here. We have a webcam to use.
@TeleOp(name = "Decode2025RobotCode_Auto", group = "Robot")
public class Decode2025RobotCode_Auto extends LinearOpMode {
    MecanumDrive driver = new MecanumDrive();
    Lift lifter = new Lift();
    Carousel carousel = new Carousel();



    public void initialize() {
        driver.initialize(hardwareMap);
        lifter.initialize(hardwareMap, telemetry);
        carousel.initialize(hardwareMap, telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        while (opModeIsActive()) {

            // go forward for 2 seconds using driver.drive(forward, strafe, rotate);
            driver.drive(0.3, 0, 0);
            sleep(2000);

            // rotate for 2 seconds
            driver.drive(0, 0, 0.5);
            sleep(2000);

            // slide left for 2 seconds
            driver.drive(0, -0.3, 0);
            sleep(2000);

            telemetry.addData("End of program", "true");
        }

    }
}
