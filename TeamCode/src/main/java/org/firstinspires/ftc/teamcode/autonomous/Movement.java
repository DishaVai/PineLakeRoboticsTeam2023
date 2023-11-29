package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement extends LinearOpMode{
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;
    DcMotor leftBackDrive;
    DcMotor linearSlide;
    CRServo tray;

    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0;
    double Kd = 0;
    // double Kp = 0.05;
    // double Ki = 0.0150;
    // double Kd = 0.000001;
    ElapsedTime timer = new ElapsedTime();

    public void initialize(HardwareMap hardwareMap){
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        tray = hardwareMap.crservo.get("outputservo");
        linearSlide = hardwareMap.get(DcMotor.class, "outputslide");

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    // tuned power
    public void strafe(int reference, int variance) {
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
    //manual power version
    public void strafe(int reference, int variance, double power) {
        while(leftFrontDrive.getCurrentPosition() > reference - variance
                || leftFrontDrive.getCurrentPosition() < reference + variance) {
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
        }
    }

    public void moveForwardToThree(int reference, int variance) {
        while(leftFrontDrive.getCurrentPosition() > reference + variance
                || leftFrontDrive.getCurrentPosition() < reference - variance) {
            double power = PIDControl(reference, reference, leftFrontDrive);
            leftFrontDrive.setPower(power * 0.5);
            rightFrontDrive.setPower(-power * 0.5);
            leftBackDrive.setPower(power * 0.5);
            rightBackDrive.setPower(-power * 0.5);
        }
    }
    public void left(int reference, int variance) {
        while(leftFrontDrive.getCurrentPosition() < reference - variance
                || leftFrontDrive.getCurrentPosition() > reference + variance) {
            double power = PIDControl(reference, reference, leftFrontDrive);
            leftFrontDrive.setPower(0.5 * power);
            leftBackDrive.setPower(0.5 * power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        }
    }
    public void scoreOnBoard(int reference, int variance, Telemetry telemetry) {

        while(linearSlide.getCurrentPosition() < reference - variance
                || linearSlide.getCurrentPosition() > reference + variance) {
            double power = PIDControl (reference, reference, linearSlide);
            linearSlide.setPower(power);
            telemetry.addData("position: ", linearSlide.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("finished looping", "a");
        telemetry.update();
        linearSlide.setPower(0);
        tray.setPower(-0.5);
        sleep(500);
        tray.setPower(0);
    }
    public void resetPower() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        linearSlide.setPower(0);
    }
    public void runOpMode() {
    }

}