package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "goodRightAuto")

public class TestAuto extends LinearOpMode {
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
            ILC.closeClaw();

            DT.drive(0.3, 44);
            ILC.liftMove(3);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }
            DT.turn(0.2, 0, "left");

//            DT.strafe("left", 0.4, 12);
            DT.strafe2("left", 0.3, 20);
            while(cam.pipeline.getAlignedAnalysis().equals("NO")) {
                telemetry.addData("Aligned: ", cam.pipeline.getAlignedAnalysis().equals("NO"));
                telemetry.addData("Avg Cb: ", cam.pipeline.getAuto_align_avgCb());
                telemetry.addData("Avg Cr: ", cam.pipeline.getAuto_align_avgCr());
                telemetry.addData("Avg Y: ", cam.pipeline.getAuto_align_avgY());
                telemetry.update();
            }
            DT.drive(0.2, 8);

            wait(500);

            ILC.openClaw();

            DT.drive(0.2, -5);
            DT.strafe("right", 0.4, 11);

            // park
            ILC.liftMove(1);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            if(position.equals("right")) {
                DT.strafe("right", 0.2, 27);
            } else if(position.equals("left")) {
                DT.strafe("left", 0.2, 25);
            }

            DT.drive(0.2, -12);

            ILC.liftMove(0);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            telemetry.addData("Position: %s", position);

            telemetry.update();

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