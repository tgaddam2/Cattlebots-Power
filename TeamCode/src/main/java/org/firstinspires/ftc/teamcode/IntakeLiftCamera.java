package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class IntakeLiftCamera {
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;

    public DcMotorEx ExtendSpoolMotor;
    public DcMotorEx RetractSpoolMotor;

    public CRServoImplEx intakeRotateServo;
    public ServoImplEx intakePosServo;

    public WebcamName webcam;

    CameraBlueOrange camera;

    static final double ArmSpeed = 1;

    // need to measure [left, right]
    int [] highJunctionPos = {1300, 1300, 120, 120};
    int [] mediumJunctionPos = {2600, 2600, 120, 120};
    int [] lowJunctionPos = {4751, 4751, 120, 120};
    int [] groundJunctionPos = {4751, 4751, 120, 120};

    LinearOpMode opMode;

    ElapsedTime sleepTimer;

    public IntakeLiftCamera(LinearOpMode op) {
        opMode = op;
    }

    public void intake() {
        intakeRotateServo.setPower(1);
    }

    public void outtake() {
        intakeRotateServo.setPower(-1);
    }

    public void liftToPos(int [] position) {
        leftArmMotor.setTargetPosition(position[0]);
        rightArmMotor.setTargetPosition(position[1]);

        ExtendSpoolMotor.setTargetPosition(position[2]);
        RetractSpoolMotor.setTargetPosition(position[3]);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(Math.abs(ArmSpeed));
        rightArmMotor.setPower(Math.abs(ArmSpeed));
    }
    
    public void adjustLift(String direction) {
        leftArmMotor.setTargetPosition(0);
        rightArmMotor.setTargetPosition(1);

        ExtendSpoolMotor.setTargetPosition(2);
        RetractSpoolMotor.setTargetPosition(3);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(Math.abs(ArmSpeed));
        rightArmMotor.setPower(Math.abs(ArmSpeed));
    }

    public String getSignalPos() {
        String position = camera.getStringPosition();
        return position;
    }

    public void initIntakeLiftCamera(HardwareMap hwMap) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        leftArmMotor = hwMap.get(DcMotorEx.class, "LiftMotorLeft");
        rightArmMotor = hwMap.get(DcMotorEx.class, "LiftMotorRight");

        ExtendSpoolMotor = hwMap.get(DcMotorEx.class, "ExtendSpoolMotor");
        RetractSpoolMotor = hwMap.get(DcMotorEx.class, "RetractSpoolMotor");

        intakeRotateServo = hwMap.get(CRServoImplEx.class, "intakeRotateServo");
        intakePosServo = hwMap.get(ServoImplEx.class, "intakePosServo");

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);

        leftArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftArmMotor.setPower(0);
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
