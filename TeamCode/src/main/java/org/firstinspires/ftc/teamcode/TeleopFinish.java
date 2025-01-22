package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
public class TeleopFinish extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        boolean armToggle = false;

        Elevator elevator = new Elevator(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Mechanum mechanum = new Mechanum(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Mechanum drive setup
            mechanum.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Reset yaw rotation
            if (gamepad1.options) {
                mechanum.resetRotation();
            }

            // Elevator up and down
            if (gamepad1.dpad_up) {
                elevator.setPower(0.3);
            } else if (gamepad1.dpad_down) {
                elevator.setPower(-0.3);
            }

            // Set arm pos
            if(gamepad1.right_bumper){

            }

            // Claw toggle
            if (gamepad1.a) {
                if (claw.getPos() == 1) {
                    claw.closeClaw();
                } else {
                    claw.openClaw();
                }
            }
        }
    }
}