package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Telemetry")

public class Telemetry extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    @Override public void runOpMode() {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        drivetrain.initEncoders();

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();

            telemetry.addData("FR: %d", drivetrain.frontRightDrive.getCurrentPosition());
            telemetry.addData("FL: %d", drivetrain.frontLeftDrive.getCurrentPosition());
            telemetry.addData("BR: %d", drivetrain.backRightDrive.getCurrentPosition());
            telemetry.addData("BL: %d\n", drivetrain.backLeftDrive.getCurrentPosition());

            telemetry.addData("Arm Motor: %d", ILC.armMotor.getCurrentPosition());

        }
    }
}