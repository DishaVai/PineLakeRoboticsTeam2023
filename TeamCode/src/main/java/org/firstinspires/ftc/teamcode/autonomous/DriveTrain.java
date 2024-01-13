package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


public class DriveTrain {
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0;
    double Kd = 0;

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;

    ElapsedTime timer = new ElapsedTime();
    // todo: write your code here

    public DriveTrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initialize(hardwareMap);
    }
    public void initialize(HardwareMap hardwareMap){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double PIDControl(double reference, double lastError, DcMotor motor) {
        double state = motor.getCurrentPosition();
        double error = reference - state;
        if(error < 100 && error > -100) {
            error = 0;
        }
        integralSum += error * timer.seconds();
        double derivative = (error-lastError) / timer.seconds();

        lastError = error;

        timer.reset();

        double out = (error*Kp) + (derivative * Kd) + (integralSum * Ki);
        return out;
    }

    public void strafeLeft(int reference, int variance) {
        strafe(reference, variance);
    }

    public void strafeRight(int reference, int variance) {
        reference = -reference;
        strafe(reference, variance);
    }

    public void strafe(int reference, int variance) {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        while(leftFrontDrive.getCurrentPosition() > reference - variance
                || leftFrontDrive.getCurrentPosition() < reference + variance
        ) {
            double power = PIDControl(reference, reference, leftFrontDrive);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            if(leftFrontDrive.getCurrentPosition() >= reference - (variance+1)
                    && leftFrontDrive.getCurrentPosition() <= reference + (variance+1)) {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                break;
            }
        }
    }

    public void moveForward(int reference, int variance) {
        reference = - reference;
        moveBot(reference, variance);
    }

    public void moveBackward(int reference, int variance) {
        moveBot(reference, variance);
    }

    private void moveBot(int reference, int variance) {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        while(leftFrontDrive.getCurrentPosition() > reference + variance
                || leftFrontDrive.getCurrentPosition() < reference - variance) {
            double power = PIDControl(reference, reference, leftFrontDrive);
            leftFrontDrive.setPower(power * 0.5);
            rightFrontDrive.setPower(-power * 0.5);
            leftBackDrive.setPower(power * 0.5);
            rightBackDrive.setPower(-power * 0.5);
        }
    }

    public void turnLeft(int reference, int variance) {
        turn(reference, variance);
    }

    public void turnRight(int reference, int variance) {
        reference = -reference;
        turn(reference, variance);
    }

    private void turn(int reference, int variance) {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        while(leftFrontDrive.getCurrentPosition() < reference - variance
                || leftFrontDrive.getCurrentPosition() > reference + variance) {
            double power = PIDControl(reference, reference, leftFrontDrive);
            leftFrontDrive.setPower(0.5 * power);
            leftBackDrive.setPower(0.5 * power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        }
    }

    public void moveRobotToAprilTag(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void move(double rangeError, double  headingError, double  yawError) {
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        this.telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        if(rangeError > 4) {
            moveRobotToAprilTag(drive, strafe, turn);
            telemetry.addData("rangeError ", rangeError);
        }else{
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }
        this.telemetry.addData("distance left ", rangeError);
        this.telemetry.update();
    }

}