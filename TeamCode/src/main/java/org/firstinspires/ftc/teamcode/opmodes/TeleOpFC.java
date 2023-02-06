package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "TeleOpFC")

public class TeleOpFC extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime clawButtonTimer = new ElapsedTime();
    private ElapsedTime dPadTimer = new ElapsedTime();
    private ElapsedTime headingFixTimer = new ElapsedTime();

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
        int armPosition = 0;
        int armZero = 0;
        double armSpeed = 0.5;

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
                armPosition = armZero + ILC.lowJunctionPos;
            }
            else if(gamepad2.x) {
                armPosition = armZero + ILC.midJunctionPos;
            }
            else if(gamepad2.y) {
                armPosition = armZero +  ILC.highJunctionPos;
            }
            else if(gamepad2.a) {
                armPosition = armZero + ILC.groundJunctionPos;
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
                armPosition += 10;
            }
            else if (gamepad2.dpad_down) {
                armPosition -= 10;
            }

            if(gamepad1.y && headingFixTimer.milliseconds() >= 500) {
                headingFixTimer.reset();
                drivetrain.imu.resetYaw();
            }

            if(gamepad2.left_trigger > 0.75) {
                armSpeed = 0.9;
            }
            else {
                armSpeed = 0.6;
            }

            if(gamepad2.left_stick_button) {
                armZero = ILC.armMotor.getCurrentPosition();
            }

            ILC.encoderLiftMove(armPosition, armSpeed);
        }
    }
}

//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//
//@TeleOp(name = "TeleOpFC")
//
//public class TeleOpFCDriveTest extends LinearOpMode
//{
//    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
//    Drivetrain drivetrain = new Drivetrain(this);
//
//    private ElapsedTime clawButtonTimer = new ElapsedTime();
//    private ElapsedTime dPadTimer = new ElapsedTime();
//    private ElapsedTime headingFixTimer = new ElapsedTime();
//
//    int position = 0;
//
//    @Override public void runOpMode() {
//
//        ILC.initIntakeLiftCamera(hardwareMap);
//        drivetrain.initDrivetrain(hardwareMap);
//        drivetrain.initGyro(hardwareMap);
//
//        IMU imu;
//        IMU.Parameters parameters;
//        YawPitchRollAngles angles;
//
//        parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
//                        RevHubOrientationOnRobot.UsbFacingDirection.UP
//                )
//        );
//
//
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(parameters);
//
//        // Wait until we're told to go
//        waitForStart();
//
//        drivetrain.initEncoders();
//
//        clawButtonTimer.reset();
//        dPadTimer.reset();
//        boolean clawButton = false;
//
//        double speedScale = 0.4;
//
//        // Loop and update the dashboard
//        while (opModeIsActive()) {
//            telemetry.update();
//            // Control Robot Movement
//            angles = imu.getRobotYawPitchRollAngles();
//            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//            double driveAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 - angles.getYaw(AngleUnit.RADIANS);
//            double rightX = gamepad1.right_stick_x;
//            final double FLPower = speedScale * (r * Math.cos(driveAngle) + rightX);
//            final double BLPower = speedScale * (r * Math.sin(driveAngle) + rightX);
//            final double FRPower = speedScale * (r * Math.sin(driveAngle) - rightX);
//            final double BRPower = speedScale * (r * Math.cos(driveAngle) - rightX);
//
//            drivetrain.FLMotor.setPower(FLPower);
//            drivetrain.BLMotor.setPower(BLPower);
//            drivetrain.FRMotor.setPower(FRPower);
//            drivetrain.BRMotor.setPower(BRPower);
//
//            // turn intake on and off
//            if(gamepad2.right_bumper && clawButtonTimer.milliseconds() >= 250) {
//                clawButtonTimer.reset();
//                clawButton = !clawButton;
//            }
//            if(clawButton) {
//                ILC.closeClaw();
//            }
//            else {
//                ILC.openClaw();
//            }
//
//            if(gamepad2.b) {
//                ILC.liftMove(1);
//            }
//            else if(gamepad2.x) {
//                ILC.liftMove(2);
//            }
//            else if(gamepad2.y) {
//                ILC.liftMove(3);
//            }
//            else if(gamepad2.a) {
//                ILC.liftMove(0);
//            }
//
//            if(gamepad1.left_trigger > 0.75) {
//                speedScale = 0.7;
//
//                drivetrain.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                drivetrain.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                drivetrain.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                drivetrain.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//            }
//            else {
//                speedScale = 0.4;
//
//                drivetrain.FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                drivetrain.FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                drivetrain.BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                drivetrain.BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
//
//            if(gamepad2.dpad_up) {
//                ILC.dPadMove("up");
//            }
//            else if (gamepad2.dpad_down) {
//                ILC.dPadMove("down");
//            }
//
//            if(gamepad1.y && headingFixTimer.milliseconds() >= 500) {
//                headingFixTimer.reset();
//                drivetrain.imu.resetYaw();
//            }
//
//            if(gamepad2.left_trigger > 0.75) {
//                ILC.ArmSpeed = 0.8;
//            }
//            else {
//                ILC.ArmSpeed = 0.5;
//            }
//        }
//    }
//}
//
