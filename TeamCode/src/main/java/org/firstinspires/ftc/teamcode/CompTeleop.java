package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
@SuppressWarnings("unused")
public class CompTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Mechanum mechanum = new Mechanum(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);

        double armState = 0;
        double elbowState = 0;
        double wristState = 0;

        boolean clawToggle = true;
        boolean collectToggle = true;

        ElapsedTime collectTimer = new ElapsedTime();
        ElapsedTime goDownTimer = new ElapsedTime();

        boolean prepareCollectDebounce = true;
        boolean goDown = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        claw.closeClaw();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.options) {
                mechanum.resetRotation();
            }

            // prepare collect
            if (gamepad1.dpad_up) {
                arm.moveToPose(Arm.prepCollectPose);
                elbow.moveToPose(Elbow.collectSample);
                claw.openClaw();

                if(prepareCollectDebounce){
                    prepareCollectDebounce = false;
                    goDown = true;

                    goDownTimer.reset();
                }
            }
            else {
                prepareCollectDebounce = true;
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
                    elbowState = Elbow.collectSample;
                    claw.openClaw();

                }
            } else {
                if(!collectToggle){
                    claw.closeClaw();
                    collectTimer.reset();

                    while (collectTimer.seconds() < 1){
                        opModeIsActive();
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

            if (gamepad1.x) {
                armState = Arm.armLowBasket;
                elbowState = Elbow.elbowLowBasket;
            }

            if (gamepad1.left_bumper) {
                armState = Arm.prepSpec;
                elbowState = Elbow.scoreSpecimen;
            }

            if (gamepad1.left_trigger > 0.05) {
                armState = Arm.scoreSpec;
                elbowState = Elbow.scoreSpecimen;
            }

            if(gamepad1.right_bumper){
                armState = Arm.collectSpecimen;
                elbowState = Elbow.collectSpecimen;
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
            elbow.moveToPose(elbowState);

            telemetry.addData("arm pose", arm.getArmPose());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}