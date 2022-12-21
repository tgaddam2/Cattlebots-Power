package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMain")

public class TeleOpMain extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime clawButtonTimer = new ElapsedTime();
    private ElapsedTime dPadTimer = new ElapsedTime();

    int newLeftArmPos;
    int newRightArmPos;

    @Override public void runOpMode() {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        drivetrain.initEncoders();

        clawButtonTimer.reset();
        dPadTimer.reset();
        boolean intakeButton = false;

        double speedScale = 0.7;

        int currentExtendTargetHeight = 30;
        int currentRetractTargetHeight = -30;

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();
            // Control Robot Movement
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double FLPower = speedScale * (r * Math.cos(robotAngle) + rightX);
            final double BLPower = speedScale * (r * Math.sin(robotAngle) + rightX);
            final double FRPower = speedScale * (r * Math.sin(robotAngle) - rightX);
            final double BRPower = speedScale * (r * Math.cos(robotAngle) - rightX);

            drivetrain.frontLeftDrive.setPower(FLPower);
            drivetrain.backLeftDrive.setPower(BLPower);
            drivetrain.frontRightDrive.setPower(FRPower);
            drivetrain.backRightDrive.setPower(BRPower);

            // turn intake on and off
            if(gamepad2.right_bumper && clawButtonTimer.milliseconds() >= 250) {
                clawButtonTimer.reset();
                intakeButton = !intakeButton;
            }
            if(intakeButton) {
                ILC.intake();
            }
            else {
                ILC.outtake();
            }

            if(gamepad2.dpad_up) {
                currentExtendTargetHeight += 1;
                currentRetractTargetHeight += 1;
            }
            if(gamepad2.dpad_down) {
                currentExtendTargetHeight -= 1;
                currentRetractTargetHeight -= 1;
            }
            if(gamepad2.right_stick_y > 0.5) {
                currentExtendTargetHeight += 1;
            }
            if(gamepad2.right_stick_y < -0.5) {
                currentExtendTargetHeight -= 1;
            }
            if(gamepad2.left_stick_y > 0.5) {
                currentRetractTargetHeight += 1;
            }
            if(gamepad2.left_stick_y < -0.5) {
                currentRetractTargetHeight -= 1;
            }


            ILC.updateExtendSpool(currentExtendTargetHeight);
            ILC.updateRetractSpool(currentRetractTargetHeight);
//
//            // move arm to level one, two and three as well as all the way down
//            if(gamepad2.a) {
//                ILC.liftToPos(ILC.groundJunctionPos);
//            }
//            else if(gamepad2.b) {
//                ILC.liftToPos(ILC.lowJunctionPos);
//            }
//            else if(gamepad2.x) {
//                ILC.liftToPos(ILC.mediumJunctionPos);
//            }
//            else if(gamepad2.y) {
//                ILC.liftToPos(ILC.highJunctionPos);
//            }
//
//            // micro adjust slide height
//            if(gamepad2.dpad_up) {
//                newLeftArmPos = ILC.leftArmMotor.getCurrentPosition() - 100;
//                newRightArmPos = ILC.leftArmMotor.getCurrentPosition() - 100;
//
//                int [] newPos = {newLeftArmPos, newRightArmPos};
//
//                ILC.liftToPos(newPos);
//                dPadTimer.reset();
//            }
//            if(gamepad2.dpad_down) {
//                newLeftArmPos = ILC.leftArmMotor.getCurrentPosition() - 100;
//                newRightArmPos = ILC.leftArmMotor.getCurrentPosition() - 100;
//
//                int [] newPos = {newLeftArmPos, newRightArmPos};
//
//                ILC.liftToPos(newPos);
//                dPadTimer.reset();
//            }
        }
    }
}

