package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
@SuppressWarnings("unused")
public class sampleTeleop extends LinearOpMode {

    public static double closeClawWaitTime = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // Subsystems
        Mechanum mechanum = new Mechanum(hardwareMap, 90);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);

        // Start positions
        double armState = 0;
        double elbowState = 0;
        double wristState = 0;

        // Toggles
        boolean clawToggle = true;
        boolean collectToggle = true;
        boolean prepCollectToggle = false;

        boolean prepareCollectDebounce = true;
        boolean goDown = false;

        // Timers
        ElapsedTime collectTimer = new ElapsedTime();
        ElapsedTime goDownTimer = new ElapsedTime();



        // Telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        claw.closeClaw();

        // Init mode
        waitForStart();

        // Start
        while (opModeIsActive()) {

            // Rotation reset
            if (gamepad1.options) {
                mechanum.resetRotation();
            }

            // Prepare collect sample
            if (gamepad1.dpad_up && armState!= Arm.prepCollectSample){
                armState = Arm.goDownPose;
                elbowState = Elbow.collectSample;
                prepCollectToggle = true;
            }

            if (prepCollectToggle && armState == Arm.goDownPose && arm.getArmPose() >= Arm.goDownPose - 0.1 && arm.getArmVel() < 1){
                armState = Arm.prepCollectSample;
                prepCollectToggle = false;
            }

            // Collect sample
            if (gamepad1.dpad_down) {
                if(collectToggle){
                    collectToggle = false;

                    armState = Arm.collectSample;
                    elbowState = Elbow.collectSample;
                    claw.openClaw();

                }
            } else {
                if(!collectToggle){
                    claw.closeClaw();
                    collectTimer.reset();
                }
                collectToggle = true;
                if (collectTimer.seconds() > closeClawWaitTime && armState == Arm.collectSample){
                    armState = Arm.postCollectSample;
                }
            }

            // Open/close claw
            if (gamepad1.circle) {
                if (clawToggle) {
                    clawToggle = false;

                    if (claw.getClawPose() == Claw.closedPose) {
                        claw.openClaw();
                    } else {
                        claw.closeClaw();
                    }
                }
            } else {
                clawToggle = true;
            }

            // Emergency reset
            if(gamepad1.square){
                armState = -100;
                elbowState = -100;
            }

            // Score sample om low basket
            if (gamepad1.cross) {
                armState = Arm.armLowBasket;
                elbowState = Elbow.elbowLowBasket;
            }

            // Prepare to score specimen
            if (gamepad1.left_bumper) {
                armState = Arm.prepSpec;
                elbowState = Elbow.prepSpec;
            }

            // Score specimen
            if (gamepad1.left_trigger > 0.05) {
                armState = Arm.scoreSpec;
                elbowState = Elbow.scoreSpecimen;
            }

            // Collect specimen
            if(gamepad1.right_bumper){
                armState = Arm.collectSpecimen;
                elbowState = Elbow.collectSpecimen;
            }

            // Slow driving
            if (gamepad1.right_trigger > 0.05) {
                mechanum.drive(
                        Math.pow(gamepad1.left_stick_x, 3) * 0.25,
                        Math.pow(-gamepad1.left_stick_y, 3) * 0.25,
                        Math.pow(gamepad1.right_stick_x, 3) * 0.15);
            }
            else {
                mechanum.drive(
                        Math.pow(gamepad1.left_stick_x, 3),
                        Math.pow(-gamepad1.left_stick_y, 3),
                        Math.pow(gamepad1.right_stick_x, 3));
            }

            arm.moveToPose(armState);
            elbow.moveToPose(elbowState);

            // Telemetry Data
            FtcDashboard.getInstance().getTelemetry().addData("arm state", armState);
            FtcDashboard.getInstance().getTelemetry().addData("elbow state", elbowState);
            FtcDashboard.getInstance().getTelemetry().addData("arm pose", arm.getArmPose());
            FtcDashboard.getInstance().getTelemetry().addData("elbow pose", elbow.getElbowPose());

            FtcDashboard.getInstance().getTelemetry().update();

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}