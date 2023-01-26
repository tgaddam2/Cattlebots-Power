package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "GyroscopeTest")

public class GyroscopeTest extends LinearOpMode
{
    Drivetrain drivetrain = new Drivetrain(this);
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);

    @Override public void runOpMode() {
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
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Loop and update the dashboard
        while (opModeIsActive()) {

            angles = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Yaw: ", angles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch: ", angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll: ", angles.getRoll(AngleUnit.DEGREES));
            telemetry.update();

            if(gamepad1.a) {
                drivetrain.turn(0.2, 89, "left");
            }
        }
    }
}

