package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.teamcode.autonomous.Movement;

@Autonomous(name = "red-close-score")
public class RedCloseScoring extends LinearOpMode{
    private CRServo tray;
    public void runOpMode() {
        Movement move = new Movement();
        move.initialize(hardwareMap);
        tray = hardwareMap.crservo.get("outputservo");


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            move.strafe(-1500, 50);
            move.moveForwardToThree(-2200, 50);
            move.left(-3500, 50);
            //move.moveForwardToThree(-3800, 50);
            tray.setPower(0.25);
            sleep(500);
            tray.setPower(0);
            move.scoreOnBoard(-1700, 50, telemetry);
            move.retract(-1700, 50, telemetry);
        }
    }
}