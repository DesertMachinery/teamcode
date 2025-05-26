package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
public class parkAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Subsystems
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);

        // Time running
        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();

        waitForStart();

        // Reset the time
        elapsedTime.reset();

        while(opModeIsActive()) {

            // Go to parking station
            if(elapsedTime.seconds() < 3){
                mechanum.drive(0.5,-0.1,0);
            }
        }
    }

}


