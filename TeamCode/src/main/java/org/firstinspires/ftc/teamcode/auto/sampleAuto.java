package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
@SuppressWarnings("unused")
public class sampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);


        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();


        waitForStart();

        elapsedTime.reset();

        while(elapsedTime.seconds() < 0.5) {

            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            opModeIsActive();
        }

        elapsedTime.reset();
        while (elapsedTime.seconds()<0.5){
            mechanum.drive(0, 0.3, 0);
            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            opModeIsActive();
        }

        mechanum.stop();

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            claw.openClaw();
            opModeIsActive();
        }

        opModeIsActive();

        claw.openClaw();

        elapsedTime.reset();
        while (elapsedTime.seconds()<0.5) {
            mechanum.drive(0, -0.3, 0);
            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            opModeIsActive();
        }
        mechanum.stop();

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.2){
            arm.moveToPose(Arm.ClosedArm);
            elbow.moveToPose(Elbow.closedElbow);
            opModeIsActive();
        }

    }

}


