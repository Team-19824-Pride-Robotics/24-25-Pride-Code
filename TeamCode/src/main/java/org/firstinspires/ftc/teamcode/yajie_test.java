package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="yajie_test")
public class yajie_test extends OpMode {

    //pid
    private PIDController controller;
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 0;
    public static double target2 = 110;
    public static double target1 = 110;

    boolean liftControl = false;


    DcMotorEx FR;
    DcMotorEx FL;
    DcMotorEx BR;
    DcMotorEx BL;
    double rotate;

    //lift
    DcMotorEx lift1;
    DcMotorEx lift2;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//pid
        //controller = new PIDController(p, i, d);

        FR = hardwareMap.get(DcMotorEx.class, "FR");
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR.setDirection(DcMotorEx.Direction.REVERSE);
        FR.setDirection(DcMotorEx.Direction.REVERSE);

        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);


    }

    @Override
    public void loop() {


        double d_power = .8 - .4 * gamepad1.left_trigger + (.5 * gamepad1.right_trigger);
        double drive = gamepad1.left_stick_y;
        double rotate_stick = -gamepad1.right_stick_x;
        double rotate_button = 0;

        rotate = rotate_stick + .5 * rotate_button;

        BL.setPower(drive + rotate);
        FL.setPower(drive + rotate);
        BR.setPower(drive - rotate);
        FR.setPower(drive - rotate);

        if (gamepad1.dpad_up) {
            BL.setPower(-d_power);
            FL.setPower(-d_power);
            BR.setPower(-d_power);
            FR.setPower(-d_power);
        } else if (gamepad1.dpad_down) {
            BL.setPower(d_power);
            FL.setPower(d_power);
            BR.setPower(d_power);
            FR.setPower(d_power);
        } else if (gamepad1.dpad_left) {
            BL.setPower(-d_power);
            FL.setPower(d_power);
            BR.setPower(d_power);
            FR.setPower(-d_power);
        } else if (gamepad1.dpad_right) {
            BL.setPower(d_power);
            FL.setPower(-d_power);
            BR.setPower(-d_power);
            FR.setPower(d_power);
        }

        //winch
        if (gamepad1.x) {
            target = 0;
        }
        if (gamepad1.y) {
            target = target1;
        }
        if (gamepad1.a) {
            target = target2;
        }


        controller.setPID(p, i, d);
        int liftPos1 = lift1.getCurrentPosition();
        int liftPos2 = lift2.getCurrentPosition();
        double pid = controller.calculate(liftPos1, target);
        double pid2 = controller.calculate(liftPos2, target);
        double ff = 0;

       double lPower1 = pid + ff;
     double lPower2 = pid2 + ff;

     lift1.setPower(lPower1);
        lift2.setPower(lPower2);


        telemetry.addData("target", target);
        telemetry.addData("Run time", getRuntime());
        telemetry.addData("1", "test");
        telemetry.addData("pos1", lift1.getCurrentPosition());
        telemetry.addData("power1", lift1.getPower());
        telemetry.addData("pos2", lift2.getCurrentPosition());
        telemetry.addData("power2", lift2.getPower());
        telemetry.update();
    }
}

