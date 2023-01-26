/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "leftAuto")

public class LeftAuto extends LinearOpMode {
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
            wait(2000);
            position = cam.getStringPosition().toLowerCase();

            //score starting cone
            ILC.closeClaw();

            DT.drive(0.3, 48);
            ILC.liftMove(3);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }
            DT.turn(0.2, 0, "left");

            DT.strafe("right", 0.4, 13);
            DT.drive(0.2, 4);

            wait(500);

            ILC.openClaw();

            DT.drive(0.2, -4);
            DT.strafe("left", 0.2, 11);
            DT.turnToZero(0.2);

            // park
            ILC.liftMove(1);
            while(ILC.armMotor.isBusy()) {
                telemetry.addData("Arm Motor: ", ILC.armMotor.getCurrentPosition());
                telemetry.update();
            }

            if(position.equals("right")) {
                DT.strafe("right", 0.2, 26);
            } else if(position.equals("left")) {
                DT.strafe("left", 0.2, 28);
            }

            DT.drive(0.2, -10);

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