package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;

@TeleOp
@Config
public class test extends LinearOpMode {

    public static double armPose = 0.7;
    public static double elbowPose = 0.5;
    public static double wristPose = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        //Claw claw = new Claw(hardwareMap);
        //Elevator elevator = new Elevator(hardwareMap);

        boolean clawToggle = false;

        waitForStart();

        while (opModeIsActive()) {
//        virtualFourBar.moveToPose(0.5);
            if (elbowPose != 0)
                elbow.moveToPose(elbowPose);
            if (armPose != 0)
                arm.moveToPose(armPose);

            wrist.moveToPose(wristPose);


            telemetry.addData("arm pose", arm.getArmPose());
            telemetry.addData("elbow pose", elbow.getElbowPose());


            //
            telemetry.update();

        }
    }

}


