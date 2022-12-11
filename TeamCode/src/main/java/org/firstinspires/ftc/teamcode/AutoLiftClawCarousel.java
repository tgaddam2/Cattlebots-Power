package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class AutoLiftClawCarousel {
    public DcMotor linearMotor;
    public DcMotor carouselLeft;
    public DcMotor carouselRight;
    public Servo armLeft;
    public Servo armRight;
    public WebcamName webcam;

    static final double     ArmSpeed                = 1;

    int downPos = 20;
    int levelOnePos = 1300;
    int levelTwoPos = 2600;
    int levelThreePos = 4751;

    LinearOpMode opMode;

    ElapsedTime sleepTimer;

    public AutoLiftClawCarousel(LinearOpMode op) {
        opMode = op;
    }

    public void openClaw() {
        armLeft.setPosition(0.5);
        armRight.setPosition(0.5);
        sleep(500);
    }

    public void closeClaw() {
        armLeft.setPosition(0.1);
        armRight.setPosition(0.9);
        sleep(500);
    }

    public void goToShippingPosition(String position) {
        if(position.equals("LEFT")) {
            linearMotor.setTargetPosition(levelOnePos);
        }
        else if(position.equals("CENTER")) {
            linearMotor.setTargetPosition(levelTwoPos);
        }
        else if(position.equals("RIGHT")) {
            linearMotor.setTargetPosition(levelThreePos);
        }

        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(Math.abs(ArmSpeed));
    }

    public void armDown(){
        linearMotor.setTargetPosition(downPos);
        linearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearMotor.setPower(Math.abs(ArmSpeed));
    }

    public void carouselTurn() {
        carouselLeft.setPower(-1);
        carouselRight.setPower(-0.5);
        sleep(7000);
        carouselLeft.setPower(0);
        carouselRight.setPower(0);
    }

    public void carouselOn() {
        carouselLeft.setPower(-1);
        carouselRight.setPower(1);
    }

    public void carouselOff() {
        carouselLeft.setPower(0);
        carouselRight.setPower(0);
    }

    public void init(HardwareMap hwMap, String clawPos) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        linearMotor = hwMap.get(DcMotor.class, "ArmMotor");
        carouselLeft = hwMap.get(DcMotor.class, "CarouselLeft");
        carouselRight = hwMap.get(DcMotor.class, "CarouselRight");
        armLeft = hwMap.get(Servo.class, "ArmLeft");
        armRight = hwMap.get(Servo.class, "ArmRight");

        webcam = hwMap.get(WebcamName.class, "Webcam 1");

        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        carouselLeft.setDirection(DcMotor.Direction.FORWARD);
        carouselRight.setDirection(DcMotor.Direction.REVERSE);

        linearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linearMotor.setPower(0);
        carouselLeft.setPower(0);
        carouselRight.setPower(0);

        if(clawPos.equals("left")) {
            armLeft.setPosition(0.75);
            armRight.setPosition(1);
        }
        else if(clawPos.equals("right")) {
            armLeft.setPosition(0);
            armRight.setPosition(0.25);
        }
        else {
            armLeft.setPosition(0.75);
            armRight.setPosition(0.25);
        }

        carouselRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    private void sleep(double millisec) {
        sleepTimer.reset();
        while(sleepTimer.milliseconds() < millisec) {}
    }
}
