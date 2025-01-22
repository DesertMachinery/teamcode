package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    Servo servoClaw;

    public static double closedPose = 1;

    public static double openPose = 0.33;

    public Claw(HardwareMap hardwareMap){
        servoClaw = hardwareMap.get(Servo.class, "Claw");
    }

    public double getPos() { return servoClaw.getPosition(); }

    public void closeClaw(){
        servoClaw.setPosition(closedPose);
    }

    public void openClaw(){
        servoClaw.setPosition(openPose);
    }

    public double getClawPose(){
        return servoClaw.getPosition();
    }

}
