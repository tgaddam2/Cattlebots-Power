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
    public DcMotor frontLeftDrive  = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive  = null;

    static private final double     COUNTS_PER_MOTOR_REV    = 537.6 ;
    static private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static private final double     WHEEL_DIAMETER_INCHES   = 96/25.4 ;     // For figuring circumference
    static private final double     WHEEL_COUNTS_PER_INCH   = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static private final int Reverse = -1;
    static private final int Forward = 1;

    private ElapsedTime runtime = new ElapsedTime();

    IMU imu;
    IMU.Parameters parameters;
    YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

    LinearOpMode opMode;

    public Drivetrain(LinearOpMode op) {
        opMode = op;
    }

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int)(distance * WHEEL_COUNTS_PER_INCH);

            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to FL %7d :FR %7d :BL %7d :BR %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2",  "Running at FL %7f :FR %7f :BL %7f :BR %7f",
                        frontLeftDrive.getPower(),
                        frontRightDrive.getPower(),
                        backLeftDrive.getPower(),
                        backRightDrive.getPower());
                opMode.telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }

    public void encoderDriveStrafe(int direction, double speed, double distance, double timeoutS) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeftDrive.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackLeftTarget = backLeftDrive.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newFrontRightTarget = frontRightDrive.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);
            newBackRightTarget = backRightDrive.getCurrentPosition() + (int) (distance * WHEEL_COUNTS_PER_INCH);

            if (direction == 0) {
                newFrontLeftTarget *= Reverse;
                newBackLeftTarget *= Forward;
                newFrontRightTarget *= Forward;
                newBackRightTarget *= Reverse;
            } else if (direction == 1) {
                newFrontLeftTarget *= Forward;
                newBackLeftTarget *= Reverse;
                newFrontRightTarget *= Reverse;
                newBackRightTarget *= Forward;
            }

            frontLeftDrive.setTargetPosition(newFrontLeftTarget);
            backLeftDrive.setTargetPosition(newBackLeftTarget);
            frontRightDrive.setTargetPosition(newFrontRightTarget);
            backRightDrive.setTargetPosition(newBackRightTarget);

            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() && (runtime.seconds() < timeoutS) && (frontLeftDrive.isBusy() && frontRightDrive.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to FL %7d :FR %7d :BL %7d :BR %7d", newFrontLeftTarget,  newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2",  "Running at FL %7f :FR %7f :BL %7f :BR %7f",
                        frontLeftDrive.getPower(),
                        frontRightDrive.getPower(),
                        backLeftDrive.getPower(),
                        backRightDrive.getPower());
                opMode.telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backRightDrive.setPower(0);
        }
    }

    void gyroTurn(double power, double angle, String direction) {

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu.resetYaw();

        if (direction.equals("left")) {
            power *= -1;
        }

        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backRightDrive.setPower(-power);

        while (opMode.opModeIsActive() && Math.abs(angles.getYaw(AngleUnit.DEGREES)) < angle) {
            opMode.telemetry.addData("heading", angles.getYaw(AngleUnit.DEGREES));
            opMode.telemetry.update();
        }

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        initEncoders();
        imu.resetYaw();
    }

    public void initDrivetrain(HardwareMap hwMap) {
        frontLeftDrive = hwMap.get(DcMotor.class, "FL");
        backLeftDrive = hwMap.get(DcMotor.class, "BL");
        frontRightDrive = hwMap.get(DcMotor.class, "FR");
        backRightDrive = hwMap.get(DcMotor.class, "BR");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void initGyro(HardwareMap hwMap) {
        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    void initEncoders() {
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
