package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    Servo servoClaw;

    static double closedPose = 0;

    static double openPose = 1;

    public Claw(HardwareMap hardwareMap){
        servoClaw = hardwareMap.get(Servo.class, "Claw");
    }

    public void closeClaw(){
        servoClaw.setPosition(closedPose);
    }

    public void openClaw(){
        servoClaw.setPosition(openPose);
    }

}
