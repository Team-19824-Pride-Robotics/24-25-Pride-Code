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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Jackson test")
@Config
public class jacksonTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    wrist Wrist;
    elbow Elbow;
    //Slide heights
    public static int saHeight1 = 0;
    public static int spHeight1 = 0;
    public static int saHeight2 = 0;
    public static int spHeight2 = 0;

    public static int saHeight3 = 0;
    public static int baseHeight = 0;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
       Servo horizontalSlides1 = hardwareMap.servo.get("horizontalSlide1");
        Servo horizontalSlides2 = hardwareMap.servo.get("horizontalSlide2");
       Servo intake = hardwareMap.servo.get("intake");
       Servo intakeBucket = hardwareMap.servo.get("intakeBucket");
       DcMotorEx verticalDownSlides = hardwareMap.get(DcMotorEx.class, "verticalDownSlides");
       DcMotorEx verticalUpSlides = hardwareMap.get(DcMotorEx.class, "verticalUpSlides");

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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Claw controls

            //close claw
            if (gamepad1.left_bumper){
                backWrist.setPosition(0.35);
                frontWrist.setPosition(0.35);
                //elbow.setPosition(0.5);
            }
            //open claw
            if (gamepad1.right_bumper){
                backWrist.setPosition(0.05);
                frontWrist.setPosition(0.05);
                //elbow.setPosition(0.1);

            }
            //These controls are to control the outtake so that it can score a specimen

            //I think it would be best to control it in "stages" (Ex. Stage 0 is all the way down,
            //Stage 1 is to score in 1st bucket, Stage 2 is to score in 2nd bucket) and when we are
            //in the intake or moving it to sample positions, we'll set the score to a value like -1
            //that this code will just ignore

            // MESSAGE IF YAJIE IS LOOKING AT CODE:
            //Don't mess with this without talking to me first!

            //go up a stage if able
            if (gamepad1.y && spStage>=0 && spStage < 3){
                spStage = spStage+1;
                saStage = -1;
                atOrigin=false;
            }
            //go down a stage if able
            if (gamepad1.a && spStage>0 && spStage <= 3){
                spStage = spStage - 1;

            } //go to stage 1
            if (gamepad1.b && atOrigin){
                spStage = 0;
                saStage = 0;
                atOrigin=false;
            }
            if (spStage == 0) {
                //go to stage 0
                elbow.setPosition(0.8);
                backWrist.setPosition(0.35);
                frontWrist.setPosition(0.35);
                verticalUpSlides.setTargetPosition(saHeight1);
                verticalDownSlides.setTargetPosition(-saHeight1);

            }
            if (spStage == 1) {
                elbow.setPosition(0.6);
                backWrist.setPosition(0.35);
                frontWrist.setPosition(0.35);
                verticalUpSlides.setTargetPosition(spHeight1);
                verticalDownSlides.setTargetPosition(-spHeight1);
                //go to stage 1
            }
            if (spStage == 2) {
                elbow.setPosition(0.6);
                backWrist.setPosition(0.35);
                frontWrist.setPosition(0.35);
                verticalUpSlides.setTargetPosition(spHeight2);
                verticalDownSlides.setTargetPosition(-spHeight2);
                //go to stage 2
            }




            //same deal as the previous code, but this is for the sample positions instead
            //of the specimen positions
            if (gamepad1.dpad_up && saStage>=0 && saStage < 3){
                saStage = saStage+1;
                spStage = -1;
                atOrigin=false;
            }
            if (gamepad1.dpad_down && saStage>0 && saStage <= 3){
                saStage = saStage - 1;

            }
            if (gamepad1.dpad_right && atOrigin){
                saStage = 0;
                spStage = 0;
                atOrigin=false;
            }
            if (saStage == 0) {
                //do nothing cuz sp and sa stage 0 are the same
                slideAdjusted=false;
            }
            if (saStage == 1) {
                elbow.setPosition(0.8);
                backWrist.setPosition(0.35);
                frontWrist.setPosition(0.35);
                slideAdjusted=false;
                verticalUpSlides.setTargetPosition(saHeight2);
                verticalDownSlides.setTargetPosition(-saHeight2);
                //go to stage 1
            }
            if (saStage == 2) {
                elbow.setPosition(0.8);
                backWrist.setPosition(0.35);
                frontWrist.setPosition(0.35);
                slideAdjusted=false;
                verticalUpSlides.setTargetPosition(saHeight3);
                verticalDownSlides.setTargetPosition(-saHeight3);
                //go to stage 2

            }

            // Bring claw back to origin
            if(gamepad1.x) {
                if ((saStage == 0 || spStage == 0) && !atOrigin) {
                    //bring claw to origin
                    atOrigin = true;
                    elbow.setPosition(0.05);
                    slideAdjusted=false;
                }
            }


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
                } else {
                    //turn on intake spinner and lower bucket
                }
            }
            if(gamepad1.left_trigger>0.5){
                //return intake
            }
            //Drive code
            double y = gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad2.right_stick_x;



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//             Send calculated power to wheels
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);


        }
    }
}
