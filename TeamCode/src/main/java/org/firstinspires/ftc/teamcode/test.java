package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;

//@TeleOp
@Config
public class test extends LinearOpMode {

    public static double armPose = 0.0;
    public static double elbowPose = 0.0;
    public static boolean clawToggle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);

        Claw claw = new Claw(hardwareMap);
        //Elevator elevator = new Elevator(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
//        virtualFourBar.moveToPose(0.5);
            if (elbowPose != 0)
                elbow.moveToPose(elbowPose);
            if (armPose != 0)
                arm.moveToPose(armPose);

            if(clawToggle){
                claw.openClaw();
            }else {
                claw.closeClaw();
            }




            FtcDashboard.getInstance().getTelemetry().addData("arm pose", arm.getArmPose());
            FtcDashboard.getInstance().getTelemetry().addData("elbow pose", elbow.getElbowPose());


            //
            FtcDashboard.getInstance().getTelemetry().update();

        }
    }

}


