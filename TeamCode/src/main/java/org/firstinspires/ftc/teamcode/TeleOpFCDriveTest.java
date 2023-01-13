package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOpFC")

public class TeleOpFCDriveTest extends LinearOpMode
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

        IMU imu;
        IMU.Parameters parameters;
        YawPitchRollAngles angles;

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

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
            angles = imu.getRobotYawPitchRollAngles();
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double driveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 - angles.getYaw(AngleUnit.RADIANS);
            double rightX = gamepad1.right_stick_x;
            final double FLPower = speedScale * (r * Math.cos(driveAngle) + rightX);
            final double BLPower = speedScale * (r * Math.sin(driveAngle) + rightX);
            final double FRPower = speedScale * (r * Math.sin(driveAngle) - rightX);
            final double BRPower = speedScale * (r * Math.cos(driveAngle) - rightX);

            drivetrain.frontLeftDrive.setPower(FLPower);
            drivetrain.backLeftDrive.setPower(BLPower);
            drivetrain.frontRightDrive.setPower(FRPower);
            drivetrain.backRightDrive.setPower(BRPower);

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

                drivetrain.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                drivetrain.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else {
                speedScale = 0.4;

                drivetrain.frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                drivetrain.backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if(gamepad2.dpad_up) {
                ILC.dPadMove("up");
            }
            else if (gamepad2.dpad_down) {
                ILC.dPadMove("down");
            }
        }
    }
}

