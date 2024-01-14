
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


@Autonomous(name="autoRed")
public class AutoOrchestrator2 extends LinearOpMode{

    private VisualControl visualControl;
    private LinearSlide linearSlide;
    private Tray tray;
    private DriveTrain drivetrain;
    boolean finished = false;
    double DESIRED_DISTANCE = 9.3;
    boolean reachedDistance = false;
    boolean firstTurn = false;
    double rangeError;
    double headingError;
    double yawError;

    public void orchestrate() {
        visualControl.setManualExposure(6, 250);
        while(opModeIsActive()) {
            if(!firstTurn) {
                drivetrain.moveForward(600, 100);
                drivetrain.turnRight(1600, 50);
                firstTurn = true;
            }
            AprilTagDetection aprilTag = visualControl.detectAprilTag();

            if(aprilTag != null && !reachedDistance) {
                if(aprilTag.id == 4) {
                    rangeError = (aprilTag.ftcPose.range - DESIRED_DISTANCE);
                    headingError = aprilTag.ftcPose.bearing;
                    yawError = aprilTag.ftcPose.yaw;
                    if(rangeError >= 3) {
                        drivetrain.move(rangeError, headingError,yawError);
                    }
                    if(rangeError <= 4) {
                        reachedDistance = true;
                    }
                    //sleep(10000);
                }
                // drivetrain.moveForward(500, 50);
            }else if(aprilTag != null && aprilTag.id == 4){
                reachedDistance = true;
                tray.rotateUp(1000);
                sleep(1000);
                linearSlide.extendSlide(2450, 100);
                tray.rotateDown(1000);
                sleep(700);
                linearSlide.extendSlide(2600, 100);
                //drivetrain.moveForward(-100, 70);
                tray.rotateDown(1000);
                sleep(1000);
                tray.rotateUp(1000);
                sleep(1000);
                linearSlide.retractSlide(-100, 100);
                break;
            }
        }
    }
    public void runOpMode() {
        visualControl = new VisualControl(hardwareMap, telemetry, 4);
        linearSlide = new LinearSlide(hardwareMap, telemetry);
        tray = new Tray(hardwareMap, telemetry);
        drivetrain = new DriveTrain(hardwareMap, telemetry);
        waitForStart();
        orchestrate();
    }
}
