package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "intake")
public class intaketest extends LinearOpMode {

    CRServo servo;
    public void runOpMode() {
        servo = hardwareMap.get(CRServo.class, "S1");

    waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                servo.setPower(-1);
            }
            if (gamepad1.a) {
                servo.setPower(1);
            }
            if (gamepad1.x){
                servo.setPower(0);

            }
        }
    }
}
