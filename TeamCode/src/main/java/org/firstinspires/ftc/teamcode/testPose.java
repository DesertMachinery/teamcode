package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;

//@TeleOp
public class testPose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);

        //Claw claw = new Claw(hardwareMap);
        //Elevator elevator = new Elevator(hardwareMap);

        boolean clawToggle = false;

        waitForStart();

        while(opModeIsActive()) {

            arm.moveToPose(0.92);
            telemetry.addData("arm pose", arm.getArmPose());
            telemetry.addData("elbow pose", elbow.getElbowPose());

            telemetry.update();

        }
    }

}


