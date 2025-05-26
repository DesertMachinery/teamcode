package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawRotator;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;

//@TeleOp
@Config
@TeleOp
public class test extends LinearOpMode {

    public static double rotatorPose = 0;
    public static double elbowPose = 0;
    public static double armPose = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        ClawRotator clawRotator = new ClawRotator(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            if(rotatorPose!=0){
                clawRotator.moveToPose(rotatorPose);
            }
            else {
                clawRotator.setPower(0);
            }

            if(elbowPose!=0){
                elbow.moveToPose(elbowPose);
            }
            else {
                elbow.setPower(0);
            }

            if(armPose!=0){
                arm.moveToPose(armPose);
            }
            else {
                arm.setPower(0);
            }

            FtcDashboard.getInstance().getTelemetry().addData("rotator pose", clawRotator.getPose());
            FtcDashboard.getInstance().getTelemetry().addData("elbow pose", elbow.getElbowPose());
            FtcDashboard.getInstance().getTelemetry().addData("arm pose", arm.getArmPose());

            FtcDashboard.getInstance().getTelemetry().update();

        }
    }

}


