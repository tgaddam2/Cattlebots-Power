package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "GyroscopeTest")

public class GyroscopeTest extends LinearOpMode
{
    Drivetrain DT = new Drivetrain(this);
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);

    @Override public void runOpMode() {
        // Wait until we're told to go
        waitForStart();
        DT.initDrivetrain(hardwareMap);
        DT.initGyro(hardwareMap);

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.addData("Yaw: ", DT.angles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch: ", DT.angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll: ", DT.angles.getRoll(AngleUnit.DEGREES));
            DT.angles = DT.imu.getRobotYawPitchRollAngles();
            telemetry.update();

            DT.turn(0.2, 90, "left");
            DT.turn(0.2, 90, "right");
        }
    }
}

