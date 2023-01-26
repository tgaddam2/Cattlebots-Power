package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class IntakeLiftCamera {
    public DcMotor armMotor;

    public Servo armLeft;
    public Servo armRight;

    public WebcamName webcam;

    CameraBlueOrange camera;

    double ArmSpeed = 0.5;

    int highJunctionPos = 4200;
    int midJunctionPos = 3000;
    int lowJunctionPos = 1800;
    int groundJunctionPos = 0;

    int cone5Pos = 760;
    int cone4Pos = 540;
    int cone3Pos = 415;
    int cone2Pos = 220;
    int cone1Pos = 0;

    LinearOpMode opMode;

    ElapsedTime sleepTimer;

    CameraBlueOrange.SEDPipeline pipeline = new CameraBlueOrange.SEDPipeline();

    public IntakeLiftCamera(LinearOpMode op) {
        opMode = op;
    }

    public String getSignalPos() {
        String position = camera.getStringPosition();
        return position;
    }

    public void closeClaw() {
        armLeft.setPosition(0.35);
        armRight.setPosition(0.65);
    }

    public void openClaw() {
        armLeft.setPosition(0.45);
        armRight.setPosition(0.55);
    }

    public void liftMove(int position) {
        int encoderPos = 0;

        if(position == 0) {
            encoderPos = groundJunctionPos;
        }
        else if(position == 1) {
            encoderPos = lowJunctionPos;
        }
        else if(position == 2) {
            encoderPos = midJunctionPos;
        }
        else if(position == 3) {
            encoderPos = highJunctionPos;
        }

        armMotor.setTargetPosition(encoderPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.8);
    }

    public void coneMove(int position) {
        int encoderPos = 0;

        if(position == 1) {
            encoderPos = cone1Pos;
        }
        else if(position == 2) {
            encoderPos = cone2Pos;
        }
        else if(position == 3) {
            encoderPos = cone3Pos;
        }
        else if(position == 4) {
            encoderPos = cone4Pos;
        }
        else if(position == 5) {
            encoderPos = cone5Pos;
        }

        armMotor.setTargetPosition(encoderPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ArmSpeed);
    }

    public void dPadMove(String direction) {
        int encoderPos = armMotor.getCurrentPosition();

        if(direction.equals("up")) {
            encoderPos -= 100;
        }
        if(direction.equals("down")) {
            encoderPos += 100;
        }

        armMotor.setTargetPosition(encoderPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ArmSpeed);
    }

    public void initIntakeLiftCamera(HardwareMap hwMap) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        armMotor = hwMap.get(DcMotor.class, "ArmMotor");
        armLeft = hwMap.get(Servo.class, "LeftClaw");
        armRight = hwMap.get(Servo.class, "RightClaw");

        armLeft.setPosition(0.45);
        armRight.setPosition(0.55);

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0);
    }

    public void encoderMove(int encoderPos) {
        armMotor.setTargetPosition(encoderPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(ArmSpeed);
    }

    public void initCameraBlueOrange(HardwareMap hwMap) {
        webcam = hwMap.get(WebcamName.class, "Webcam 1");

        camera = new CameraBlueOrange(opMode.hardwareMap);
        camera.initCamera();
    }

    private void sleep(double millisec) {
        sleepTimer.reset();
        while(sleepTimer.milliseconds() < millisec) {}
    }
}
