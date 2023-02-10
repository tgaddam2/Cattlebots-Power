package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "testAuto")

public class testAuto extends LinearOpMode {
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);

    @Override public void runOpMode() throws InterruptedException {
        SampleMecanumDrive DT = new SampleMecanumDrive(hardwareMap);

        ILC.initIntakeLiftCamera(hardwareMap);

        CameraBlueOrange cam = new CameraBlueOrange(hardwareMap);
        cam.initCamera();

        Pose2d startPose = new Pose2d(-16, -60, rad(90));

        DT.setPoseEstimate(startPose);

        waitForStart();

        while(opModeIsActive()) {
            ILC.closeClaw();
            ILC.liftMove(3);

            Trajectory traj = DT.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-15, 0), rad(45))
                    .build();

            DT.followTrajectory(traj);

            break;
        }
    }

    public void wait(int milliseconds) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.startTime();
        while(waitTimer.milliseconds() < milliseconds) {}
    }

    public double rad(double degrees) {
        return Math.toRadians(degrees);
    }
}