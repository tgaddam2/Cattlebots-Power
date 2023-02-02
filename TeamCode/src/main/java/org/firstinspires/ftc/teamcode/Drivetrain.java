package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drivetrain {
    public DcMotor FLMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BRMotor = null;

    static private final double     COUNTS_PER_MOTOR_REV    = 384.5 ;
    static private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static private final double     WHEEL_DIAMETER_INCHES   = 96/25.4 ;     // For figuring circumference
    static private final double     WHEEL_COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static private final int Reverse = 1;
    static private final int Forward = -1;

    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;
    IMU.Parameters parameters;
    YawPitchRollAngles angles;

    LinearOpMode opMode;

    public Drivetrain(LinearOpMode op) {
        opMode = op;
    }

    public void drive(double speed, double distance) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = FLMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTarget = BLMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTarget = FRMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTarget = BRMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);

            FLMotor.setTargetPosition(newFrontLeftTarget);
            BLMotor.setTargetPosition(newBackLeftTarget);
            FRMotor.setTargetPosition(newFrontRightTarget);
            BRMotor.setTargetPosition(newBackRightTarget);

            FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FLMotor.setPower(Math.abs(speed));
            BLMotor.setPower(Math.abs(speed));
            FRMotor.setPower(Math.abs(speed));
            BRMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() && (FLMotor.isBusy() && FRMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to FL %7d :FR %7d :BL %7d :BR %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2",  "Running at FL %7f :FR %7f :BL %7f :BR %7f",
                        FLMotor.getPower(),
                        FRMotor.getPower(),
                        BLMotor.getPower(),
                        BRMotor.getPower());
                opMode.telemetry.update();
            }

            // Stop all motion;
            FLMotor.setPower(0);
            BLMotor.setPower(0);
            FRMotor.setPower(0);
            BRMotor.setPower(0);
        }
    }

    public void pidDrive(double distance) {
        int FLTarget;
        int BLTarget;
        int FRTarget;
        int BRTarget;

        initEncoders();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            FLTarget = FLMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            BLTarget = BLMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            FRTarget = FRMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            BRTarget = BRMotor.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);

            FRMotor.setTargetPosition(FRTarget);
            FLMotor.setTargetPosition(FLTarget);
            BRMotor.setTargetPosition(BRTarget);
            BLMotor.setTargetPosition(BLTarget);

            double kP = 0.002;
            double kI = 0.002;
            double kD = 0.002;

            double FRError = FRTarget - FRMotor.getCurrentPosition();
            double FLError = FLTarget - FLMotor.getCurrentPosition();
            double BRError = BRTarget - BRMotor.getCurrentPosition();
            double BLError = BLTarget - BLMotor.getCurrentPosition();

            double FRIntegralSum = 0;
            double FLIntegralSum = 0;
            double BRIntegralSum = 0;
            double BLIntegralSum = 0;

            double FRDerivative = 0;
            double FLDerivative = 0;
            double BRDerivative = 0;
            double BLDerivative = 0;

            double FRLastError = 0;
            double FLLastError = 0;
            double BRLastError = 0;
            double BLLastError = 0;

            runtime.reset();

            while (Math.abs(FRError) > 15 && Math.abs(FLError) > 15) {
                FRError = FRTarget - FRMotor.getCurrentPosition();
                FLError = FLTarget - FLMotor.getCurrentPosition();
                BRError = BRTarget - BRMotor.getCurrentPosition();
                BLError = BLTarget - BLMotor.getCurrentPosition();

                FRIntegralSum = FRError * runtime.seconds();
                FLIntegralSum = FLError * runtime.seconds();
                BRIntegralSum = BRError * runtime.seconds();
                BLIntegralSum = BLError * runtime.seconds();

                FRDerivative = (FRError - FRLastError) / runtime.seconds();
                FLDerivative = (FLError - FLLastError) / runtime.seconds();
                BRDerivative = (BRError - BRLastError) / runtime.seconds();
                BLDerivative = (BLError - BLLastError) / runtime.seconds();

                FRLastError = FRError;
                FLLastError = FLError;
                BRLastError = BRError;
                BLLastError = BLError;

                runtime.reset();

//                FRMotor.setPower((kP * FRError) + (kD * FRDerivative) + (kI * FRIntegralSum));
//                FLMotor.setPower((kP * FLError) + (kD * FLDerivative) + (kI * FLIntegralSum));
//                BRMotor.setPower((kP * BRError) + (kD * BRDerivative) + (kI * BRIntegralSum));
//                BLMotor.setPower((kP * BLError) + (kD * BLDerivative) + (kI * BLIntegralSum));

                FRMotor.setPower((kP * FRError));
                FLMotor.setPower((kP * FRError));
                BRMotor.setPower((kP * FRError));
                BLMotor.setPower((kP * FRError));
            }

            // Stop all motion;
            stopMotors();
        }
    }

    public void strafe(String direction, double speed, double distance) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        direction = direction.toLowerCase();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = FLMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTarget = BLMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTarget = FRMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTarget = BRMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);

            if (direction.equals("right")) {
                newFrontLeftTarget *= Reverse;
                newBackLeftTarget *= Forward;
                newFrontRightTarget *= Forward;
                newBackRightTarget *= Reverse;
            } else if (direction.equals("left")) {
                newFrontLeftTarget *= Forward;
                newBackLeftTarget *= Reverse;
                newFrontRightTarget *= Reverse;
                newBackRightTarget *= Forward;
            }

            FLMotor.setTargetPosition(newFrontLeftTarget);
            BLMotor.setTargetPosition(newBackLeftTarget);
            FRMotor.setTargetPosition(newFrontRightTarget);
            BRMotor.setTargetPosition(newBackRightTarget);

            FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FLMotor.setPower(Math.abs(speed));
            BLMotor.setPower(Math.abs(speed));
            FRMotor.setPower(Math.abs(speed));
            BRMotor.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() && (FLMotor.isBusy() && FRMotor.isBusy())) {
                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to FL %7d :FR %7d :BL %7d :BR %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2",  "Running at FL %7f :FR %7f :BL %7f :BR %7f",
                        FLMotor.getPower(),
                        FRMotor.getPower(),
                        BLMotor.getPower(),
                        BRMotor.getPower());
                opMode.telemetry.update();
            }

            // Stop all motion;
            FLMotor.setPower(0);
            BLMotor.setPower(0);
            FRMotor.setPower(0);
            BRMotor.setPower(0);
        }
    }

    public void strafe2(String direction, double speed, double distance) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        direction = direction.toLowerCase();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = FLMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTarget = BLMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTarget = FRMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTarget = BRMotor.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);

            if (direction.equals("right")) {
                newFrontLeftTarget *= Reverse;
                newBackLeftTarget *= Forward;
                newFrontRightTarget *= Forward;
                newBackRightTarget *= Reverse;
            } else if (direction.equals("left")) {
                newFrontLeftTarget *= Forward;
                newBackLeftTarget *= Reverse;
                newFrontRightTarget *= Reverse;
                newBackRightTarget *= Forward;
            }

            FLMotor.setTargetPosition(newFrontLeftTarget);
            BLMotor.setTargetPosition(newBackLeftTarget);
            FRMotor.setTargetPosition(newFrontRightTarget);
            BRMotor.setTargetPosition(newBackRightTarget);

            FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLMotor.setPower(Math.abs(speed));
            BLMotor.setPower(Math.abs(speed));
            FRMotor.setPower(Math.abs(speed));
            BRMotor.setPower(Math.abs(speed));
        }
    }

    void turn(double power, double angle, String direction) {
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();
        wait(50);
        angles = imu.getRobotYawPitchRollAngles();

        if (direction.equals("left")) {
            power *= -1;
        }

        FLMotor.setPower(power);
        BLMotor.setPower(power);
        FRMotor.setPower(-power);
        BRMotor.setPower(-power);

        while (opMode.opModeIsActive() && Math.abs(angles.getYaw(AngleUnit.DEGREES)) < angle) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();
        }

        imu.resetYaw();

        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);

        initEncoders();
    }

    void turnToZero(double power) {
        String direction = "right";
        angles = imu.getRobotYawPitchRollAngles();

        if(angles.getYaw(AngleUnit.DEGREES) < 0) {
            direction = "left";
        }

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction.equals("left")) {
            power *= -1;
        }

        FLMotor.setPower(power);
        BLMotor.setPower(power);
        FRMotor.setPower(-power);
        BRMotor.setPower(-power);

        runtime.reset();

        while (opMode.opModeIsActive() && Math.abs(angles.getYaw(AngleUnit.DEGREES)) > 3 && runtime.milliseconds() < 2000) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
            angles = imu.getRobotYawPitchRollAngles();
        }

        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);

        initEncoders();
    }

    public void stopMotors() {
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);
    }

    public void initDrivetrain(HardwareMap hwMap) {
        FLMotor = hwMap.get(DcMotor.class, "FL");
        BLMotor = hwMap.get(DcMotor.class, "BL");
        FRMotor = hwMap.get(DcMotor.class, "FR");
        BRMotor = hwMap.get(DcMotor.class, "BR");

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);
    }

    public void initGyro(HardwareMap hwMap) {
        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        imu = hwMap.get(IMU.class, "imu");
        angles = imu.getRobotYawPitchRollAngles();
        imu.initialize(parameters);
        imu.resetYaw();
    }

    void initEncoders() {
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void wait(int milliseconds) {
        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.startTime();
        while(waitTimer.milliseconds() < milliseconds) {}
    }
}
