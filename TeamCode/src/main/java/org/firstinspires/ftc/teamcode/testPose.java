package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.VirtualFourBar;

@TeleOp
public class testPose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        VirtualFourBar virtualFourBar = new VirtualFourBar(hardwareMap);
        //Claw claw = new Claw(hardwareMap);
        //Elevator elevator = new Elevator(hardwareMap);

        boolean clawToggle = false;

        waitForStart();

        while(opModeIsActive()) {
        virtualFourBar.moveToPose(0.5);
            arm.moveToPose(0.92);
            telemetry.addData("arm pose", arm.getArmPose());

            telemetry.update();

        }
    }

}


