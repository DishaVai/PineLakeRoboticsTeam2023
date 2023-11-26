package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.autonomous.Movement;


@Autonomous(name = "red-close-delay")

public class RedCloseDelay extends LinearOpMode{
    public void runOpMode() {
        Movement move = new Movement();
        move.initialize(hardwareMap);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            sleep(5000);
            move.strafe(-2600, 25);
            //move.strafeLeft(2000,-25);
        }
    }
}