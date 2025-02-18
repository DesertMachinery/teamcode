package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.Timer;

@TeleOp
public class CompTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Mechanum mechanum = new Mechanum(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        VirtualFourBar virtualFourBar = new VirtualFourBar(hardwareMap);

        double armState = 0;

        boolean clawToggle = true;
        boolean collectToggle = true;

        ElapsedTime collectTimer = new ElapsedTime();
        ElapsedTime goDownTimer = new ElapsedTime();

        boolean prepereCollectDebounce = true;
        boolean goDown = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.options) {
                mechanum.resetRotation();
            }

            // prepare collect
            if (gamepad1.dpad_up) {
                claw.openClaw();
                virtualFourBar.moveToPose(VirtualFourBar.sampleIntake);

                if(prepereCollectDebounce){
                    prepereCollectDebounce = false;
                    goDown = true;

                    goDownTimer.reset();
                }
            }
            else {
                prepereCollectDebounce = true;
            }

            if(goDownTimer.seconds() > 0.5 && goDown){
                armState = Arm.goDownPose;
                goDown = false;
            }

            if (armState == Arm.goDownPose && arm.getArmPose() > Arm.goDownPose ){
                armState = Arm.prepCollectPose;
            }

            //collect
            if (gamepad1.dpad_down) {
                if(collectToggle){
                    collectToggle = false;

                    armState = Arm.collectPose;
                    claw.openClaw();
                    virtualFourBar.moveToPose(VirtualFourBar.sampleIntake);
                }
            } else {
                if(!collectToggle){
                    claw.closeClaw();
                    collectTimer.reset();

                    while (collectTimer.seconds() < 1){

                    }
                    armState = Arm.postCollectPose;
                }
                collectToggle = true;
            }

            //open/close claw
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

            if (gamepad1.left_bumper) {
                armState = Arm.prepSpec;
                virtualFourBar.moveToPose(VirtualFourBar.chamber);
            }

            if (gamepad1.left_trigger > 0.05) {
                armState = Arm.scoreSpec;
                virtualFourBar.moveToPose(VirtualFourBar.chamber);
            }

            if(gamepad1.right_bumper){
                armState = Arm.collectSpecimen;
                virtualFourBar.moveToPose(VirtualFourBar.specPick);
            }

            if (gamepad1.right_trigger > 0.05) {
                mechanum.drive(
                        Math.pow(gamepad1.left_stick_x, 3) * 0.25,
                        Math.pow(-gamepad1.left_stick_y, 3) * 0.25,
                        Math.pow(gamepad1.right_stick_x, 3) * 0.25);
            }
            else {
                mechanum.drive(
                        Math.pow(gamepad1.left_stick_x, 3),
                        Math.pow(-gamepad1.left_stick_y, 3),
                        Math.pow(gamepad1.right_stick_x, 3));
            }


            arm.moveToPose(armState);


            telemetry.addData("arm pose", arm.getArmPose());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}