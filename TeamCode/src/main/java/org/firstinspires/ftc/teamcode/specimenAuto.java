package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
public class specimenAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);
        VirtualFourBar virtualFourBar = new VirtualFourBar(hardwareMap);

        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();


        waitForStart();

        elapsedTime.reset();

        while(elapsedTime.seconds() < 2) {
            virtualFourBar.moveToPose(VirtualFourBar.chamber);
            arm.moveToPose(Arm.prepSpec);
        }

        elapsedTime.reset();
        while (elapsedTime.seconds()<1){
            mechanum.drive(0, 0.3, 0);
            arm.moveToPose(Arm.prepSpec);
        }

        mechanum.stop();

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            arm.moveToPose(Arm.scoreSpec);
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.2){
            mechanum.drive(0, -0.2, 0);
        }


        claw.openClaw();

        elapsedTime.reset();
        while (elapsedTime.seconds() < 2){
            virtualFourBar.moveToPose(VirtualFourBar.closedPose);
            arm.moveToPose(Arm.collectSpecimen);
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.2){
            arm.moveToPose(Arm.ClosedArm);
        }

        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.5){
            mechanum.drive(0.6,-0.1,0);
        }

        mechanum.stop();

        while(opModeIsActive()) {
            arm.moveToPose(Arm.ClosedArm);
        }
    }

}


