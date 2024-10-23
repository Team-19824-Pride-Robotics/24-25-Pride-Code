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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;





/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Jackson test")
//@Disabled
public class jacksonTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    wrist Wrist;
    elbow Elbow;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

       //Drive base config
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        //Mechanism config
       Servo claw = hardwareMap.servo.get("claw");
       Servo backWrist = hardwareMap.servo.get("backWrist");
       Servo frontWrist = hardwareMap.servo.get("frontWrist");
       Servo elbow = hardwareMap.servo.get("armElbow");
       Servo horizontalSlides = hardwareMap.servo.get("horizontalSlide");
       DcMotorEx verticalDownSlides = hardwareMap.get(DcMotorEx.class, "verticalDownSlides");
       DcMotorEx verticalUpSlides = hardwareMap.get(DcMotorEx.class, "verticalUpSlides");
        //Drivebase reversal
//        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //Various drive code variables
        int spStage = -1;
        boolean atOrigin = true;
        int saStage = -1;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Claw controls

            //close claw
            if (gamepad1.left_bumper){
                backWrist.setPosition(0.99);
                frontWrist.setPosition(0.99)
                ;
                elbow.setPosition(0.5);
            }
            //open claw
            if (gamepad1.right_bumper){
                backWrist.setPosition(-0.99);
                frontWrist.setPosition(-0.99);
                elbow.setPosition(0.5);
            }
            //These controls are to control the outtake so that it can score a specimen

            //I think it would be best to control it in "stages" (Ex. Stage 0 is all the way down,
            //Stage 1 is to score in 1st bucket, Stage 2 is to score in 2nd bucket) and when we are
            //in the intake or moving it to sample positions, we'll set the score to a value like -1
            //that this code will just ignore

            // MESSAGE IF YAJIE IS LOOKING AT CODE:
            //Don't delete this without talking to me first!

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
            }
            if (spStage == 1) {
                elbow.setPosition(0.6);
                //go to stage 1
            }
            if (spStage == 2) {
                elbow.setPosition(0.6);
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
            }
            if (saStage == 1) {
                elbow.setPosition(0.8);
                //go to stage 1
            }
            if (saStage == 2) {
                elbow.setPosition(0.8);
                //go to stage 2
            }

            // Bring claw back to origin
            if(gamepad1.x) {
                if ((saStage == 0 || spStage == 0) && !atOrigin) {
                    //bring claw to origin
                    atOrigin = true;
                    elbow.setPosition(0.05);
                }
            }


            if(gamepad1.dpad_left){
                if (!atOrigin && saStage!=0 && spStage !=0) {
                //Bring slides down a little to hang a sample

                }

            }
            //Drive code
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;




//             Send calculated power to wheels
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);


        }
    }
}
