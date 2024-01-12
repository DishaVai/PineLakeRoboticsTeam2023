package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Tray extends LinearOpMode{
    private CRServo tray;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Tray(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        tray = hardwareMap.crservo.get("outputservo");
    }

    private void rotateTray(int time, boolean up) {
        if(up) {
            tray.setPower(-0.25);
        }else {
            tray.setPower(0.25);
        }
        sleep(time);
        tray.setPower(0);
    }

    public void rotateUp(int time) {
        rotateTray(time, true);
    }

    public void rotateDown(int time) {
        rotateTray(time, false);
    }

    public void runOpMode(){};
}