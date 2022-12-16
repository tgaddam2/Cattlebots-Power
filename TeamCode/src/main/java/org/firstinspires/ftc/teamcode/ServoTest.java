package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ServoTest")

public class ServoTest extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime intakeButtonTimer = new ElapsedTime();
    private ElapsedTime dPadTimer = new ElapsedTime();

    int newLeftArmPos;
    int newRightArmPos;

    @Override public void runOpMode() {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        intakeButtonTimer.reset();
        boolean intakeButton = false;

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();

            // turn intake on and off
            if(gamepad2.right_bumper && intakeButtonTimer.milliseconds() >= 50) {
                intakeButtonTimer.reset();
                intakeButton = !intakeButton;
            }
            if(intakeButton) {
                ILC.intake();
            }
            else {
                ILC.outtake();
            }
        }
    }
}

