package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
public class Automomouse extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        double armState = 0;

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("arm pose", arm.getArmPose());
            telemetry.update();
elevator.moveToPose(0.1);



            if(gamepad1.x){
                armState = 0.96;
            }

            if(gamepad1.left_bumper){
                armState = 0.7;
            }

            if (gamepad1.right_bumper){
                claw.openClaw();
            }

            arm.moveToPose(armState);
        } //9609
    }

}


