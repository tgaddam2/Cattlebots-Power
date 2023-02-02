package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "doubleRightAuto")

public class doubleRightAuto extends LinearOpMode {
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain DT = new Drivetrain(this);

    CameraBlueOrange.SEDPipeline pipeline;

    @Override public void runOpMode() {
        CameraBlueOrange cam = new CameraBlueOrange(hardwareMap);

        ILC.initIntakeLiftCamera(hardwareMap);

        cam.initCamera();
        DT.initDrivetrain(hardwareMap);
        DT.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        // Loop and update the dashboard
        while (opModeIsActive()) {
            String position = cam.getStringPosition().toLowerCase();
            wait(1000);
            position = cam.getStringPosition().toLowerCase();

            //score starting cone
            ILC.armLeft.setPosition(0.4);
            ILC.armRight.setPosition(0.6);

            DT.drive(0.3, 50);

            ILC.closeClaw();

            ILC.liftMove(3);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }
            DT.turnToZero(0.2);

            DT.strafe("left", 0.2, 26);

            DT.strafe2("right", 0.1, 20);
            while(cam.pipeline.getCenterAlignedAnalysis().equals("NO")) {
                telemetry.addData("Aligned: ", cam.pipeline.getCenterAlignedAnalysis().equals("NO"));
                telemetry.addData("Avg Cb: ", cam.pipeline.getCenter_align_avgCb());
                telemetry.addData("Avg Cr: ", cam.pipeline.getCenter_align_avgCr());
                telemetry.addData("Avg Y: ", cam.pipeline.getCenter_align_avgY());
                telemetry.addData("Position: ", position);
                telemetry.update();
            }

            DT.drive(0.2, 4);
            wait(50);

            ILC.encoderLiftMove(3785, 0.8);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            ILC.openClaw();
            ILC.liftMove(3);

            DT.drive(0.2, -4);

            // get another cone
            DT.strafe("right", 0.4, 5);
            DT.turn(0.2, 86, "right");
            ILC.coneMove(5);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            DT.drive(0.5, 35);
            wait(200);

            ILC.closeClaw();
            wait(300);
            ILC.liftMove(3);

            wait(300);

            DT.drive(0.4, -17);
            DT.turn(0.2, 87, "left");

            ILC.liftMove(3);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            DT.strafe("left", 0.3, 27);

            DT.turnToZero(0.2);

            // second cone
            DT.strafe2("right", 0.1, 20);
            while(cam.pipeline.getCenterAlignedAnalysis().equals("NO")) {
                telemetry.addData("Aligned: ", cam.pipeline.getCenterAlignedAnalysis().equals("NO"));
                telemetry.addData("Avg Cb: ", cam.pipeline.getCenter_align_avgCb());
                telemetry.addData("Avg Cr: ", cam.pipeline.getCenter_align_avgCr());
                telemetry.addData("Avg Y: ", cam.pipeline.getCenter_align_avgY());
                telemetry.addData("Position: ", position);
                telemetry.update();
            }

            DT.drive(0.2, 7);
            wait(50);

            ILC.openClaw();

            DT.drive(0.4, -2);

            ILC.liftMove(1);

            DT.turn(0.2, 86, "right");

            ILC.liftMove(0);

            if(position.equals("right")) {
                DT.drive(0.7, 37);
            } else if(position.equals("left")) {
                DT.drive(0.7, -11);
            } else {
                DT.drive(0.7, 8);
            }

//            DT.turn(0.3, 83, "left");
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            break;
        }
    }

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        pipeline = new CameraBlueOrange.SEDPipeline();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public void wait(int milliseconds) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.startTime();
        while(waitTimer.milliseconds() < milliseconds) {}
    }
}