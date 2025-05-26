package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
@SuppressWarnings("unused")
public class specimenAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Subsystems
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);

        // Time running
        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();


        waitForStart();

        elapsedTime.reset();

        // Prepare arms
        while(elapsedTime.seconds() < 0.5) {

            arm.moveToPose(Arm.prepSpec);
            elbow.moveToPose(Elbow.prepSpec);
            opModeIsActive();
        }

        elapsedTime.reset();
        // Go to bar
        while (elapsedTime.seconds()<4){
            mechanum.drive(0, 0.3, 0);
            arm.moveToPose(Arm.prepSpec);
            elbow.moveToPose(Elbow.prepSpec);
            opModeIsActive();
        }

        mechanum.stop();

        elapsedTime.reset();

        //Score sample
        while (elapsedTime.seconds() < 0.5){
            arm.moveToPose(Arm.scoreSpec);
            elbow.moveToPose(Elbow.scoreSpecimen);
            opModeIsActive();
        }

        opModeIsActive();

        claw.openClaw();


        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.2){
            arm.moveToPose(Arm.ClosedArm);
            elbow.moveToPose(Elbow.closedElbow);
            opModeIsActive();
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            opModeIsActive();
            mechanum.drive(0.6,-0.4,0);
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 2){
            opModeIsActive();
            mechanum.drive(0.6,-0.4,0);
            arm.moveToPose(0.1);
            elbow.moveToPose(0.1);
        }

        while(opModeIsActive()) {
            arm.moveToPose(0);
            elbow.moveToPose(0);
            mechanum.drive(0.6,-0.4,0);
        }
    }

}


