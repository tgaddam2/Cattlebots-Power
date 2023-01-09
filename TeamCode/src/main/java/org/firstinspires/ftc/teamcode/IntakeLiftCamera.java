package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class IntakeLiftCamera {
    public DcMotorEx armMotor;

    public Servo armLeft;
    public Servo armRight;

    public WebcamName webcam;

    CameraBlueOrange camera;

    static final double ArmSpeed = 0.3;
    static final double SpoolSpeed = 1;

    // need to measure [left, right]
    int [] highJunctionPos = {1300, 1300, 120, 120};
    int [] mediumJunctionPos = {2600, 2600, 120, 120};
    int [] lowJunctionPos = {4751, 4751, 120, 120};
    int [] groundJunctionPos = {4751, 4751, 120, 120};

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
//        armLeft.setPosition(-0.5);
        armRight.setPosition(0.5);
        sleep(500);
        armLeft.setPosition(-0.5);
        sleep(500);
    }

    public void openClaw() {
        armLeft.setPosition(-0.3);
        armRight.setPosition(0.3);
        sleep(500);
    }

    public void initIntakeLiftCamera(HardwareMap hwMap) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        armMotor = hwMap.get(DcMotorEx.class, "ArmMotor");
        armLeft = hwMap.get(Servo.class, "LeftClaw");
        armRight = hwMap.get(Servo.class, "RightClaw");

        armRight.setPosition(0);
        armLeft.setPosition(0);

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0);
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
