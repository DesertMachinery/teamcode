package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Config
public class Claw {

    Servo servoClaw;

    public static double closedPose = 0.75;

    public static double openPose = 0.4;

    public Claw(HardwareMap hardwareMap){
        servoClaw = hardwareMap.get(Servo.class, "Claw");
    }

    @SuppressWarnings("unused")
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
