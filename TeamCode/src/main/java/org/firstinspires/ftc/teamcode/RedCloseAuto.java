package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.Movement;


@Autonomous(name = "red-close-auto")

public class RedCloseAuto extends LinearOpMode{
    public void runOpMode() {
        Movement move = new Movement();
        move.initialize(hardwareMap);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            move.strafe(-2600, 25);
        }
    }
}