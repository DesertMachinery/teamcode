package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
@SuppressWarnings("unused")
public class specimenAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);


        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();


        waitForStart();

        elapsedTime.reset();

        while(elapsedTime.seconds() < 2) {

            arm.moveToPose(Arm.prepSpec);
            elbow.moveToPose(Elbow.scoreSpecimen);
            wrist.moveToPose(Wrist.prepScoreSpecimen);
            opModeIsActive();
        }

        elapsedTime.reset();
        while (elapsedTime.seconds()<1){
            mechanum.drive(0, 0.3, 0);
            arm.moveToPose(Arm.prepSpec);
            elbow.moveToPose(Elbow.scoreSpecimen);
            wrist.moveToPose(Wrist.prepScoreSpecimen);
            opModeIsActive();
        }

        mechanum.stop();

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            arm.moveToPose(Arm.scoreSpec);
            elbow.moveToPose(Elbow.scoreSpecimen);
            wrist.moveToPose(Wrist.scoreSpecimen);
            opModeIsActive();
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.4){
            mechanum.drive(0, -0.2, 0);
            opModeIsActive();
        }

        opModeIsActive();

        claw.openClaw();

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.4){
            opModeIsActive();

            arm.moveToPose(Arm.collectSpecimen);
            elbow.moveToPose(Elbow.collectSpecimen);
            wrist.moveToPose(Wrist.collectSpecimen);
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.2){
            arm.moveToPose(Arm.ClosedArm);
            elbow.moveToPose(Elbow.closedElbow);
            wrist.moveToPose(Wrist.closedWrist);
            opModeIsActive();
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            opModeIsActive();
            mechanum.drive(0.6,-0.1,0);
        }

        mechanum.stop();

        while(opModeIsActive()) {
            arm.moveToPose(Arm.ClosedArm);
            elbow.moveToPose(Elbow.closedElbow);
            wrist.moveToPose(Wrist.closedWrist);
            mechanum.drive(0.6,-0.1,0);
        }
    }

}


