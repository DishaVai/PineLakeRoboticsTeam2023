package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.Movement;

@Autonomous(name = "blue-far-auto")
public class BlueFarAuto extends LinearOpMode{
    public void runOpMode() {
        Movement move = new Movement();
        move.initialize(hardwareMap);


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();


        if (opModeIsActive()) {
            move.moveForwardToThree(-1900, 100);
            move.resetPower();
            move.strafe(3550, 25);
            //As the robot goes through the gate, any linear slides will smash into the
            //gate, most likely breaking it. Using a lower power will prevent this.
            //move.strafeLeft(2000,-25);
        }
    }
}