package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wrist {

    Servo servoWrist;

    public static double collectSample = 0.9;

    public static double collectSpecimen = 0.1;
    @SuppressWarnings("unused")
    public static double closedWrist = 0.1;

    public static double scoreSpecimen = 0.1;

    public static double prepScoreSpecimen = 1;

    public static double scoreSample = 1;


    public Wrist(HardwareMap hardwareMap){
        servoWrist = hardwareMap.get(Servo.class, "wrist");
    }

    @SuppressWarnings("unused")
    public double getPos() { return servoWrist.getPosition(); }

    public void moveToPose(double pose){
        servoWrist.setPosition(pose);
    }

}
