package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Servo Testing")

public class ServoTest extends LinearOpMode
{
    IntakeLiftCamera ILC = new IntakeLiftCamera(this);
    Drivetrain drivetrain = new Drivetrain(this);

    private ElapsedTime clawButtonTimer = new ElapsedTime();

    @Override public void runOpMode() {

        ILC.initIntakeLiftCamera(hardwareMap);
        drivetrain.initDrivetrain(hardwareMap);
        drivetrain.initGyro(hardwareMap);

        // Wait until we're told to go
        waitForStart();

        clawButtonTimer.reset();
        boolean clawButton = false;

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // turn intake on and off
//            if(gamepad2.right_bumper && clawButtonTimer.milliseconds() >= 50) {
//                clawButtonTimer.reset();
//                clawButton = !clawButton;
//            }

            for(int i = 0; i < 5; i++) {
                clawButtonTimer.reset();
                while(clawButtonTimer.milliseconds() < 1000){}

                ILC.closeClaw();


                clawButtonTimer.reset();
                while(clawButtonTimer.milliseconds() < 1000){}

                ILC.openClaw();
            }

            while(clawButtonTimer.milliseconds() < 1000){}
            break;

//            String a = "";
//            for(int i = 0; i < 5; i++){
//                telemetry.addData("-1", a);
//                telemetry.update();
//                ILC.armRight.setPosition(-1);
//                ILC.armLeft.setPosition(-1);
//
//                clawButtonTimer.reset();
//                while(clawButtonTimer.milliseconds() < 3000){}
//
//                telemetry.addData("1", a);
//                telemetry.update();
//                ILC.armRight.setPosition(1);
//                ILC.armLeft.setPosition(1);
//
//                clawButtonTimer.reset();
//                while(clawButtonTimer.milliseconds() < 3000){}
//            }
//
//            break;
        }
    }
}

