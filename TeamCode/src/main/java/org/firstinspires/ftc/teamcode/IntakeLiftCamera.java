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

    public CRServoImplEx intakeRotateServo;
    public ServoImplEx intakePosServo;

    public WebcamName webcam;

    CameraBlueOrange camera;

    static final double ArmSpeed = 1;

    // need to measure [left, right]
    int [] highJunctionPos = {1300, 1300};
    int [] mediumJunctionPos = {2600, 2600};
    int [] lowJunctionPos = {4751, 4751};
    int [] groundJunctionPos = {4751, 4751};

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

    public void liftMove(int [] position) {
        leftArmMotor.setTargetPosition(position[0]);
        leftArmMotor.setTargetPosition(position[1]);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(Math.abs(ArmSpeed));
        rightArmMotor.setPower(Math.abs(ArmSpeed));
    }

    public String getSignalPos() {
        String position = camera.getStringPosition();
        return position;
    }

    public void initIntakeLift(HardwareMap hwMap) {
        sleepTimer = new ElapsedTime();

        // Define and Initialize Motors
        leftArmMotor = hwMap.get(DcMotorEx.class, "ArmMotorLeft");
        rightArmMotor = hwMap.get(DcMotorEx.class, "ArmMotorRight");

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
