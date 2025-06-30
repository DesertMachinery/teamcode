package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
@SuppressWarnings("unused")
public class sampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Subsystems
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        ClawRotator clawRotator = new ClawRotator(hardwareMap);
        // Time running
        ElapsedTime elapsedTime = new ElapsedTime();

        claw.closeClaw();

        waitForStart();

        // Reset the time
        elapsedTime.reset();

        // Prepare the arm
        while(elapsedTime.seconds() < 0.5) {

            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            clawRotator.moveToPose(ClawRotator.rotatorLowBasket);
            opModeIsActive();

            Action tea =
                    new MecanumDrive(hardwareMap, new Pose2d(0.0,0.0,0.0 ))
                            .actionBuilder(new Pose2d(0,0,0))
                            .lineToX(0).build();
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            tea.preview(telemetryPacket.fieldOverlay());
            tea.run(telemetryPacket);
            telemetryPacket.fieldOverlay();
        }

        elapsedTime.reset();

        // Go to basket
        while (elapsedTime.seconds()<0.5){
            mechanum.drive(0, 0.3, 0);
            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            clawRotator.moveToPose(ClawRotator.rotatorLowBasket);
            opModeIsActive();
        }

        mechanum.stop();


        elapsedTime.reset();

        // Score sample
        while (elapsedTime.seconds() < 0.5){
            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            clawRotator.moveToPose(ClawRotator.rotatorLowBasket);
            claw.openClaw();
            opModeIsActive();
        }

        opModeIsActive();

        claw.openClaw();

        elapsedTime.reset();

        // Pickup sample
        while (elapsedTime.seconds()<0.5) {
            mechanum.drive(0, -0.3, 0);
            arm.moveToPose(Arm.armLowBasket);
            elbow.moveToPose(Elbow.elbowLowBasket);
            clawRotator.moveToPose(ClawRotator.rotatorLowBasket);
            opModeIsActive();
        }
        mechanum.stop();

        // Score sample
        elapsedTime.reset();
        while (elapsedTime.seconds() < 0.2){
            arm.moveToPose(Arm.ClosedArm);
            elbow.moveToPose(Elbow.closedElbow);
            clawRotator.moveToPose(ClawRotator.closedRotator);
            opModeIsActive();
        }

    }

}


