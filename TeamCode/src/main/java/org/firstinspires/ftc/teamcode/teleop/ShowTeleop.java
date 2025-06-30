package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.ClawRotator;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.Mechanum;

@TeleOp
@SuppressWarnings("unused")
public class ShowTeleop extends LinearOpMode {

    public static double closeClawWaitTime = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // Subsystems
        Mechanum mechanum = new Mechanum(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Elbow elbow = new Elbow(hardwareMap);
        ClawRotator clawRotator = new ClawRotator(hardwareMap);

        // Start positions
        double armState = 0;
        double elbowState = 0;
        double rotatorState = 0;

        boolean clawToggle = false;

        claw.closeClaw();

        // Init mode
        waitForStart();

        // Start
        while (opModeIsActive()) {

            // Rotation reset
            if (gamepad1.options) {
                mechanum.resetRotation();
            }

            if (gamepad1.dpad_up) {
                armState = Arm.prepCollectSample;
                elbowState = Elbow.collectSample;
                rotatorState = ClawRotator.collectSample;
            }

            if (gamepad1.dpad_down) {
                armState = Arm.collectSample;
                elbowState = Elbow.collectSample;
                rotatorState = ClawRotator.collectSample;
            }

            if (gamepad1.cross) {
                armState = Arm.armLowBasket;
                elbowState = Elbow.elbowLowBasket;
                rotatorState = ClawRotator.rotatorLowBasket;
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
            clawRotator.moveToPose(rotatorState);

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
