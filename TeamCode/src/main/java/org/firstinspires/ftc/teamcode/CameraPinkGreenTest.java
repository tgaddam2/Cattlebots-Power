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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "CameraPinkGreenTest")

public class CameraPinkGreenTest extends LinearOpMode
{
    HardwarePushbot     robot   = new HardwarePushbot();   // Use a Pushbot's hardware

    static final double     ArmSpeed                = 1;

    // State used for updating telemetry
    Orientation angles;

    int downPos = 0;
    int levelOnePos = 100;
    int levelTwoPos = 200;
    int levelThreePos = 300;

    ElapsedTime wait = new ElapsedTime();

    CameraPinkGreen.SEDPipeline pipeline = new CameraPinkGreen.SEDPipeline();

    @Override public void runOpMode() {

        robot.init(hardwareMap);

        initCamera();
        int avgYValue = pipeline.getAvgY();
        int avgCrValue = pipeline.getAvgCr();
        int avgCbValue = pipeline.getAvgCb();

        // Wait until we're told to go
        waitForStart();
        wait.startTime();

        // Loop and update the dashboard
        while (opModeIsActive()) {
            telemetry.update();

            goToShippingPosition();
            wait.reset();
            while(wait.seconds() < 1) {}
            goToShippingPosition();

            avgYValue = pipeline.getAvgY();
            avgCrValue = pipeline.getAvgCr();
            avgCbValue = pipeline.getAvgCb();

            telemetry.addData("Y", avgYValue);
            telemetry.addData("Cr", avgCrValue);
            telemetry.addData("Cb", avgCbValue);
        }
    }

    public void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvInternalCamera phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

    void goToShippingPosition() {
        String position = pipeline.getStringAnalysis();
        telemetry.addData("here", position);
        if(position.equals("LEFT")) {
            robot.linearMotor.setTargetPosition(levelOnePos);
        }
        else if(position.equals("CENTER")) {
            robot.linearMotor.setTargetPosition(levelTwoPos);
        }
        else if(position.equals("RIGHT")) {
            robot.linearMotor.setTargetPosition(levelThreePos);
        }

        robot.linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.linearMotor.setPower(Math.abs(ArmSpeed));
        sleep(10);
    }
}