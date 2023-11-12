package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Airplane")
public class Airplane extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private CRServo airplane = null;
    //private DcMotor linearSlide1;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        airplane = hardwareMap.crservo.get("airplane");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.dpad_up) {
                airplane.setPower(-1);
            } else {
                airplane.setPower(0);
            }
        }
    }
    /*public void moveToPosition(double reference) {
        while (linearSlide.getCurrentPosition() != reference) {
            double power = PIDControl(reference, linearSlide.getCurrentPosition());
            linearSlide.setPower(power);
        }
    }*/
//}
}