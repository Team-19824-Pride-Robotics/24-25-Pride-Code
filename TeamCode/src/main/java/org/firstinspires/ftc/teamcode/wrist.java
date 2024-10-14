package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class wrist {
    private Servo backWrist;
    private Servo frontWrist;

    public wrist(HardwareMap hardwareMap) {
        frontWrist = hardwareMap.get(Servo.class, "claw");
        backWrist = hardwareMap.get(Servo.class, "claw");
    }

    public class turnWristVertical implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            frontWrist.setPosition(-0.5);
            backWrist.setPosition(0.5);
            return false;
        }
    }
}
