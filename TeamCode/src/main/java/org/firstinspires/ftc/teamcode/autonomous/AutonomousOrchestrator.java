package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.autonomous.*;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name="auto")
public class AutonomousOrchestrator extends LinearOpMode{

    private VisualControl visualControl;
    private LinearSlide linearSlide;
    private Tray tray;

    public void orchestrate() {
        tray.rotateUp(1500);
        sleep(2000);
        linearSlide.extendSlide(2500, 100);
        sleep(2000);
        linearSlide.retractSlide(2500, 100);
        sleep(2000);
        tray.rotateDown(1500);
        // visualControl.setManualExposure(6, 250);
        // AprilTagDetection aprilTag = visualControl.detectAprilTag();
        // if(aprilTag != null) {
        //     telemetry.addData(aprilTag.id + "", " hi");
        //     telemetry.update();
        // }else{
        //     telemetry.addData("not ", "found");
        //     telemetry.update();
        // }


    }

    public void runOpMode() {
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        visualControl = new VisualControl(hardwareMap, telemetry);
        tray = new Tray(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()) {
            orchestrate();
        }
    }
}