package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class IntakeLift {
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;

    public CRServoImplEx intakeRotateServo;
    public ServoImplEx intakePosServo;

    public WebcamName webcam;

    static final double ArmSpeed = 1;

    // need to measure
    int leftHighJunctionPos = 1300;
    int leftMediumJunctionPos = 2600;
    int leftLowJunctionPos = 4751;
    int leftGroundJunctionPos = 4751;

    // need to measure
    int rightHighJunctionPos = 1300;
    int rightMediumJunctionPos = 2600;
    int rightLowJunctionPos = 4751;
    int rightGroundJunctionPos = 4751;

    LinearOpMode opMode;

    ElapsedTime sleepTimer;

    public IntakeLift(LinearOpMode op) {
        opMode = op;
    }

    public void intake() {
        intakeRotateServo.setPower(1);
    }

    public void outtake() {
        intakeRotateServo.setPower(-1);
    }

    public void goToJunctionPosition(int position) {
        if(position == 3) {
            leftArmMotor.setTargetPosition(leftHighJunctionPos);
            rightArmMotor.setTargetPosition(rightHighJunctionPos);
        }
        else if(position == 2) {
            leftArmMotor.setTargetPosition(leftMediumJunctionPos);
            rightArmMotor.setTargetPosition(rightMediumJunctionPos);
        }
        else if(position == 1) {
            leftArmMotor.setTargetPosition(leftLowJunctionPos);
            rightArmMotor.setTargetPosition(rightLowJunctionPos);
        }
        else {
            leftArmMotor.setTargetPosition(leftGroundJunctionPos);
            rightArmMotor.setTargetPosition(rightGroundJunctionPos);
        }

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(Math.abs(ArmSpeed));
        rightArmMotor.setPower(Math.abs(ArmSpeed));
    }

    public void initIntakeLift(HardwareMap hwMap) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        leftArmMotor = hwMap.get(DcMotorEx.class, "ArmMotorLeft");
        rightArmMotor = hwMap.get(DcMotorEx.class, "ArmMotorRight");

        intakeRotateServo = hwMap.get(CRServoImplEx.class, "intakeRotateServo");
        intakePosServo = hwMap.get(ServoImplEx.class, "intakePosServo");

        webcam = hwMap.get(WebcamName.class, "Webcam 1");

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setPower(0);
    }

    private void sleep(double millisec) {
        sleepTimer.reset();
        while(sleepTimer.milliseconds() < millisec) {}
    }
}
