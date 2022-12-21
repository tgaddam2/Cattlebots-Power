package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class IntakeLiftCamera {
    public DcMotorEx leftArmMotor;
    public DcMotorEx rightArmMotor;

    public DcMotorEx ExtendSpoolMotor;
    public DcMotorEx RetractSpoolMotor;

    public CRServoImplEx intakeRotateServo;
    public ServoImplEx intakePosServo;

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

    public void intake() {
        intakeRotateServo.setPower(-1);
    }

    public void outtake() {
        intakeRotateServo.setPower(1);
    }

    public void offtake() {
        intakeRotateServo.setPower(0);
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
        ExtendSpoolMotor.setTargetPosition(500);
        RetractSpoolMotor.setTargetPosition(500);

        leftArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(Math.abs(ArmSpeed));
        rightArmMotor.setPower(Math.abs(ArmSpeed));
    }

    public void setAngle(int newAngle) {

    }

    public void adjustAngle(String direction) {

    }

    public void updateExtendSpool(int targetHeight) {
        int targetExtendStringValue = (int) (537.7 * (targetHeight) / 107.0);

//        int extendSpoolDifference = ((targetExtendStringValue - ExtendSpoolMotor.getCurrentPosition()));
//
//        if (Math.abs(extendSpoolDifference) <= 50) {
//            ExtendSpoolMotor.setPower(0);
//        } else {
//            ExtendSpoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if (extendSpoolDifference > 0) {
//                ExtendSpoolMotor.setPower(Math.abs(SpoolSpeed));
//            } else {
//                ExtendSpoolMotor.setPower(-0.3 * Math.abs(SpoolSpeed));
//            }
//        }

        ExtendSpoolMotor.setTargetPosition(targetExtendStringValue);
        ExtendSpoolMotor.setPower(0.5);
        ExtendSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateRetractSpool(int targetHeight) {
        int targetRetractStringValue = (int) (537.7 * (targetHeight) / 107.0);

//        int retractSpoolDifference = ((targetRetractStringValue - RetractSpoolMotor.getCurrentPosition()));
//
//
//        if (Math.abs(retractSpoolDifference) <= 50) {
//            RetractSpoolMotor.setPower(0);
//        } else {
//            RetractSpoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            if (retractSpoolDifference > 0) {
//                RetractSpoolMotor.setPower(0.3 * Math.abs(SpoolSpeed));
//            } else {
//                RetractSpoolMotor.setPower(-1 * Math.abs(SpoolSpeed));
//            }
//        }

        RetractSpoolMotor.setTargetPosition(targetRetractStringValue);
        RetractSpoolMotor.setPower(0.5);
        RetractSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void updateLiftArm(int targetHeight) {
////        int AngleValue = (int) (targetAngle * 537.7 / 360);
//        if (!ExtendSpoolMotor.isBusy()) {
//            ExtendSpoolMotor.setPower(0);
//        }
//        if (!RetractSpoolMotor.isBusy()) {
//            RetractSpoolMotor.setPower(0);
//        }
//
//        int targetExtendStringValue = (int) (537.7 * (calcDistExtend(50, targetHeight) - calcDistExtend(50, 0)) / 107.0);
//        int targetRetractStringValue = (int) (537.7 * (calcDistRetract(50, targetHeight) - calcDistRetract(50, 0)) / 107.0);
////
//        if (Math.abs(ExtendSpoolMotor.getCurrentPosition() - targetExtendStringValue) <= 10) {
//            ExtendSpoolMotor.setTargetPosition(ExtendSpoolMotor.getCurrentPosition());
//            RetractSpoolMotor.setTargetPosition(ExtendSpoolMotor.getCurrentPosition());
//            RetractSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            ExtendSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else if (ExtendSpoolMotor.getCurrentPosition() > targetExtendStringValue) {
//            RetractSpoolMotor.setTargetPosition(targetRetractStringValue);
//            RetractSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            RetractSpoolMotor.setPower(1);
//        } else {
//            ExtendSpoolMotor.setTargetPosition(targetExtendStringValue);
//            ExtendSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            ExtendSpoolMotor.setPower(1);
//        }
//
////            }
        int targetExtendStringValue = (int) (537.7 * (targetHeight) / 107.0);
        int targetRetractStringValue = (int) (537.7 * (targetHeight) / 107.0);

        int extendSpoolDifference = ((targetExtendStringValue - ExtendSpoolMotor.getCurrentPosition()));
        int retractSpoolDifference = ((targetRetractStringValue - RetractSpoolMotor.getCurrentPosition()));
//        int averageSpoolDifference = (extendSpoolDifference + retractSpoolDifference)/2;

        if (Math.abs(extendSpoolDifference) <= 50) {
//            ExtendSpoolMotor.setTargetPosition(ExtendSpoolMotor.getCurrentPosition());
//            ExtendSpoolMotor.setPower(1);
//            ExtendSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ExtendSpoolMotor.setPower(0);
        } else {
            ExtendSpoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (extendSpoolDifference > 0) {
                ExtendSpoolMotor.setPower(Math.abs(SpoolSpeed));
            } else {
                ExtendSpoolMotor.setPower(-0.3 * Math.abs(SpoolSpeed));
            }
        }

        if (Math.abs(retractSpoolDifference) <= 50) {
//            RetractSpoolMotor.setTargetPosition(RetractSpoolMotor.getCurrentPosition());
//            RetractSpoolMotor.setPower(1);
//            RetractSpoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RetractSpoolMotor.setPower(0);
        } else {
            RetractSpoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (retractSpoolDifference > 0) {
                RetractSpoolMotor.setPower(0.3 * Math.abs(SpoolSpeed));
            } else {
                RetractSpoolMotor.setPower(-1 * Math.abs(SpoolSpeed));
            }
        }


//        if (Math.abs(AngleValue - leftArmMotor.getCurrentPosition()) >= 10) {
//            int ArmMotorSign = 1;
//
//            if (AngleValue - leftArmMotor.getCurrentPosition() < 0) {
//                ArmMotorSign = -1;
//
//            if (Math.abs(averageSpoolDifference) <= 5) {
//                leftArmMotor.setPower(ArmMotorSign * ArmSpeed);
//                rightArmMotor.setPower(ArmMotorSign * ArmSpeed);
//            } else {
//                int MotorSpeedRatio = ((Math.abs(AngleValue - leftArmMotor.getCurrentPosition())))/averageSpoolDifference;
//
//                leftArmMotor.setPower(ArmMotorSign * Math.abs(MotorSpeedRatio * SpoolSpeed));
//                rightArmMotor.setPower(ArmMotorSign * Math.abs(MotorSpeedRatio * SpoolSpeed));
//            }
//        } else {
//            leftArmMotor.setPower(0);
//            rightArmMotor.setPower(0);
////        }
    }

    public int calcDistExtend(int angle, int h) {
        return h - (int) Math.sqrt(6044 - 6192 * Math.cos(angle * 3.1416/180 + 0.519));
    }

    public int calcDistRetract(int angle, int h) {
        return (int) Math.sqrt(9360 + Math.pow((375 + h), 2) - 192 * Math.sqrt(400 + Math.pow((375 + h), 2)) * Math.cos(angle * 3.1416/180 + 0.0533));
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

        intakeRotateServo = hwMap.get(CRServoImplEx.class, "IntakeRotateServo");
        intakePosServo = hwMap.get(ServoImplEx.class, "IntakePosServo");

        leftArmMotor.setDirection(DcMotor.Direction.FORWARD);
        ExtendSpoolMotor.setDirection(DcMotor.Direction.REVERSE);
        RetractSpoolMotor.setDirection(DcMotor.Direction.REVERSE);

        ExtendSpoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RetractSpoolMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ExtendSpoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RetractSpoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
