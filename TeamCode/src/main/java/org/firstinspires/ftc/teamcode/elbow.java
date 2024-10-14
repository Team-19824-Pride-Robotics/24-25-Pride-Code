package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class elbow {
    private Servo armElbow;
    public elbow(HardwareMap hardwareMap) {
        armElbow = hardwareMap.get(Servo.class, "armElbow");
    }
    public class armUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armElbow.setPosition(0.5);
            return false;
        }
    }
    public class armIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            armElbow.setPosition(0);
            return false;
        }
    }
}
