package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
public class CompTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Mechanum mechanum = new Mechanum(hardwareMap);
        Elevator elevator = new Elevator(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        double armState = 0;
        double elevatorState = 0;

        boolean clawToggle = true;
        boolean collectToggle = true;

        ElapsedTime collectTimer = new ElapsedTime();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.options) {
                mechanum.resetRotation();
            }

            // prepare collect
            if (gamepad1.right_bumper) {
                claw.openClaw();
                elevatorState = Elevator.prepCollectPose;
                armState = Arm.prepCollectPose;
            }

            //collect
            if (gamepad1.left_bumper) {
                if(collectToggle){
                    collectToggle = false;

                    elevatorState = Elevator.collectPose;
                    armState = Arm.collectPose;
                    claw.openClaw();
                }
            }
            else {
                if(!collectToggle){
                    claw.closeClaw();
                    collectTimer.reset();

                    while (collectTimer.seconds() < 0.3){

                    }
                    collectTimer.reset();

                    elevatorState = Elevator.prepCollectPose;
                    armState = Arm.prepCollectPose;
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

            //hold
            if (gamepad1.cross) {
                armState = Arm.ClosedArm;
                elevatorState = Elevator.ClosedElevator;
            }

            //upper basket
            if (gamepad1.left_trigger > 0.05) {
                armState = Arm.highBasket;
                elevatorState = Elevator.highBasket;
                claw.closeClaw();
            }


            elevator.moveToPose(elevatorState);
            arm.moveToPose(armState);

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


            telemetry.addData("arm pose", arm.getArmPose());
            telemetry.addData("elevator pose", elevator.getElevatorPose());

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}