package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class VirtualFourBar {

    Servo servoBar;

    public static double closedPose = 0.9;

    public static double specPick = 0.1;
    
    public static double chamber = 1;

    public static double sampleIntake = 1;


    public VirtualFourBar(HardwareMap hardwareMap){
        servoBar = hardwareMap.get(Servo.class, "bar");
    }

    public double getPos() { return servoBar.getPosition(); }

    public void moveToPose(double pose){
        servoBar.setPosition(pose);
    }

}
