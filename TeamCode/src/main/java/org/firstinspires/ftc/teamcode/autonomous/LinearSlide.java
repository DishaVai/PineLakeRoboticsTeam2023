package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;


public class LinearSlide {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private DcMotor linearSlide;

    double integralSum = 0;
    double Kp = 0.1;
    double Ki = 0;
    double Kd = 0;
    ElapsedTime timer = new ElapsedTime();

    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){
        linearSlide = hardwareMap.get(DcMotor.class, "outputslide");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
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

    private void slide(int reference, int variance, Telemetry telemetry) {
        while(linearSlide.getCurrentPosition() < reference - variance
                || linearSlide.getCurrentPosition() > reference + variance) {
            double power = PIDControl (reference, reference, linearSlide);
            linearSlide.setPower(power);
            telemetry.addData("position: ", linearSlide.getCurrentPosition());
            telemetry.update();
        }
        linearSlide.setPower(0);
    }

    public void extendSlide(int reference, int variance) {
        reference = -reference;
        slide(reference, variance, telemetry);
    }

    public void retractSlide(int reference, int variance) {
        slide(reference, variance, telemetry);
    }
}