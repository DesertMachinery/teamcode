package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
public class parkAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);

        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();

        waitForStart();

        elapsedTime.reset();

        while(opModeIsActive()) {
            if(elapsedTime.seconds() < 3){
                mechanum.drive(0.5,-0.1,0);
            }

//            arm.moveToPose(Arm.ClosedArm);
//            elevator.moveToPose(Elevator.ClosedElevator);
        }
    }

}


