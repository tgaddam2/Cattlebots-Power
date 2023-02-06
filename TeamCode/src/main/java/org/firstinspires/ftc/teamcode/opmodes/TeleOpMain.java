package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpMain")

public class TeleOpMain extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime clawButtonTimer = new ElapsedTime();
    private ElapsedTime dPadTimer = new ElapsedTime();
    private ElapsedTime headingFixTimer = new ElapsedTime();

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
        boolean clawButton = false;

        double speedScale = 0.4;

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

            drivetrain.FLMotor.setPower(FLPower);
            drivetrain.BLMotor.setPower(BLPower);
            drivetrain.FRMotor.setPower(FRPower);
            drivetrain.BRMotor.setPower(BRPower);

            // turn intake on and off
            if(gamepad2.right_bumper && clawButtonTimer.milliseconds() >= 250) {
                clawButtonTimer.reset();
                clawButton = !clawButton;
            }
            if(clawButton) {
                ILC.closeClaw();
            }
            else {
                ILC.openClaw();
            }

            if(gamepad2.b) {
                ILC.liftMove(1);
            }
            else if(gamepad2.x) {
                ILC.liftMove(2);
            }
            else if(gamepad2.y) {
                ILC.liftMove(3);
            }
            else if(gamepad2.a) {
                ILC.liftMove(0);
            }

            if(gamepad1.left_trigger > 0.75) {
                speedScale = 0.7;

                drivetrain.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else {
                speedScale = 0.4;

                drivetrain.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if(gamepad2.dpad_up) {
                ILC.dPadMove("up");
            }
            else if (gamepad2.dpad_down) {
                ILC.dPadMove("down");
            }

            if(gamepad1.y && headingFixTimer.milliseconds() >= 500) {
                clawButtonTimer.reset();
                drivetrain.imu.resetYaw();
            }

            if(gamepad2.left_trigger > 0.75) {
                ILC.ArmSpeed = 0.8;
            }
            else {
                ILC.ArmSpeed = 0.5;
            }
        }
    }
}

