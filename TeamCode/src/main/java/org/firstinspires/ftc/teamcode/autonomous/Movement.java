package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement{
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;
    DcMotor leftBackDrive;

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

    public void strafeLeft(int desiredLoc, int variance) {
        while(leftFrontDrive.getCurrentPosition() > desiredLoc - variance
                || leftFrontDrive.getCurrentPosition() < desiredLoc + variance
        ) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
        }
    }
    public double PIDControl(double reference, double lastError, DcMotor motor) {
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

    public void moveForwardToThree(int desiredLoc, int variance) {
        while(leftFrontDrive.getCurrentPosition() > desiredLoc + variance
                || leftFrontDrive.getCurrentPosition() < desiredLoc - variance) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(power * 0.5);
            rightFrontDrive.setPower(-power * 0.5);
            leftBackDrive.setPower(power * 0.5);
            rightBackDrive.setPower(-power * 0.5);
        }
    }
    public void left(int desiredLoc, int variance) {
        while(leftFrontDrive.getCurrentPosition() < desiredLoc - variance
                || leftFrontDrive.getCurrentPosition() > desiredLoc + variance) {
            double power = PIDControl(desiredLoc, desiredLoc, leftFrontDrive);
            leftFrontDrive.setPower(0.5 * power);
            leftBackDrive.setPower(0.5 * power);
            rightBackDrive.setPower(power);
            rightFrontDrive.setPower(power);
        }
    }
    public void resetPower() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}