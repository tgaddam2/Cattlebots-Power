package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

            telemetry.addData("Left Arm Motor: %d", ILC.leftArmMotor.getCurrentPosition());
            telemetry.addData("Right Arm Motor: %d\n", ILC.rightArmMotor.getCurrentPosition());

            telemetry.addData("Extend Spool Motor: %d", ILC.ExtendSpoolMotor.getCurrentPosition());
            telemetry.addData("Retract Spool Motor: %d", ILC.RetractSpoolMotor.getCurrentPosition());
        }
    }
}