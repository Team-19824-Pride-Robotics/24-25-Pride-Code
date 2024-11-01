/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Pride robot 2 player")
@Config
public class jacksonTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //lift pid
    public static double p = 0.005, i = 0, d = 0;
    public static double f = 0;
    public static double target = 0;
    public static double target2 = 110;
    public static double target1 = 110;

    private PIDController controller;

    wrist Wrist;
    elbow Elbow;
    //Slide heights
    public static int saHeight1 = 1300;
    public static int spHeight1 = 0;
    public static int saHeight2 = 3000;
    public static int spHeight2 = 1000;


    public static int baseHeight = 0;

    public static double HPos = 0.12;

    public static double HPos2 = 0.4;

    public static double Bpos = 0.08;

    public static double Bpos2 = 0.47;

    public static double Epos1 = 0.67;
    public static double Epos2 = 0.3;
    public static double Epos3 = 0.07;

    public static double Cpos = 1;

    public static double Cpos2 = 0.7;

    public static double Wpos1 = 0.7;

    public static double Wpos2 = 0.7;
    public static double Wpos3 = 0.7;

    DcMotorEx lift1;
    DcMotorEx lift2;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        controller = new PIDController(p, i, d);
       //Drive base config
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRight = hardwareMap.dcMotor.get("backRightMotor");
        //Mechanism config
       Servo claw = hardwareMap.servo.get("claw");
       Servo backWrist = hardwareMap.servo.get("backWrist");
       Servo frontWrist = hardwareMap.servo.get("frontWrist");
       Servo elbow = hardwareMap.servo.get("armElbow");
       Servo horizontalSlides1 = hardwareMap.servo.get("horizontalSlides1");
        Servo horizontalSlides2 = hardwareMap.servo.get("horizontalSlides2");
       CRServo intake = hardwareMap.get(CRServo.class, "intake");
       Servo intakeBucket = hardwareMap.servo.get("intakeBucket");
       //lift
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setDirection(DcMotorEx.Direction.REVERSE);

        //Drivebase reversal
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//Various drive code variables
        int spStage = -1;
        boolean atOrigin = true;
        int saStage = -1;
        boolean intakeIsOut = false;
        boolean slideAdjusted = false;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


            //elbow.setPosition(Epos1);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Pid stuff lol
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
            //Claw controls

            //close claw
            if (gamepad1.left_bumper){
                if(!intakeIsOut) {
                    claw.setPosition(Cpos2);
                    elbow.setPosition(Epos1);
                } else {
                    intake.setPower(1);
                }

            }
            ///open claw
            if (gamepad1.right_bumper){
                if(!intakeIsOut) {
                claw.setPosition(Cpos);
                elbow.setPosition(Epos3);
                } else{
                    intake.setPower(-1);
                    intakeBucket.setPosition(Bpos2);
                }
            }
            //These controls are to control the outtake so that it can score a sample


            // MESSAGE IF YAJIE IS LOOKING AT CODE:
            //Don't mess with this without talking to me first!


            //go up a stage if able
            if (gamepad1.y && saStage>=0 && saStage < 3){
                saStage = saStage+1;
                spStage = -1;
                atOrigin=false;

            }
            //go down a stage if able
            if (gamepad1.a && saStage>0 && saStage <= 3){
                saStage = saStage - 1;

            } //go to stage 1 from origin
            if (gamepad1.b && atOrigin){
                saStage = 0;
                spStage = 0;
                atOrigin=false;
            }
            if(gamepad1.x) {
                if ((saStage == 0 || spStage == 0)) {
                    //bring claw to origin
                    atOrigin = true;
                    elbow.setPosition(Epos1);
                    slideAdjusted=false;
                }
            }
            if (saStage == 0) {
                //go to stage 0
                elbow.setPosition(Epos3);
                target=0;


            }
            if (saStage == 1) {
                elbow.setPosition(Epos2);
                target=saHeight1;

                //go to stage 1
            }
            if (saStage == 2) {
                elbow.setPosition(Epos2);
                target=saHeight2;

                //go to stage 2
            }




            //same deal as the previous code, but this is for the specimen positions instead
            //of the sample positions
            if (gamepad1.dpad_up && spStage>=0 && spStage < 3){
                spStage = spStage+1;
                saStage = -1;
                atOrigin=false;
            }
            if (gamepad1.dpad_down && spStage>0 && spStage <= 3){
                spStage = spStage - 1;

            }

            if (spStage == 0) {
                //do nothing cuz sp and sa stage 0 are the same
                slideAdjusted=false;
            }
            if (spStage == 1) {
                elbow.setPosition(Epos3);
                target=spHeight1;
                slideAdjusted=false;

                //go to stage 1
            }
            if (spStage == 2) {
                elbow.setPosition(Epos3);
                target=spHeight2;
                slideAdjusted=false;

                //go to stage 2

            }

            // Bring claw back to origin

//cool

            if(gamepad1.dpad_left){
                if (!atOrigin && saStage!=0 && spStage !=0 && !slideAdjusted) {
                //Bring slides down a little to hang a sample
                    slideAdjusted=true;

                } else if (!atOrigin && saStage!=0 && spStage !=0 && slideAdjusted) {
                    //Bring slides up a little bit to reset if we miss the sample
                    slideAdjusted=false;
                }

            }
            if(gamepad1.right_trigger>0.5){
                if(!intakeIsOut) {
                    //send out intake and raise bucket
                    horizontalSlides1.setPosition(HPos2);
                    horizontalSlides2.setPosition(HPos2);
                    intake.setPower(1);
                    intakeIsOut = true;
                }
            }
            if(gamepad1.left_trigger>0.5){
                horizontalSlides1.setPosition(HPos);
                horizontalSlides2.setPosition(HPos);
                intakeBucket.setPosition(Bpos);
                intakeIsOut=false;
            }
            if(!intakeIsOut) {
                intake.setPower(0);
            }
            //slides

            //Drive code
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



//pid


            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


//             Send calculated power to wheels
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

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
}
