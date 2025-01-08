package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.*;

@TeleOp
public class TestTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Elevator elevator = new Elevator(hardwareMap);

        Arm arm = new Arm(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

//            elevator.moveToPose(Elevator.highBasket);
            arm.moveToPose(Arm.highBasket);

            FtcDashboard.getInstance().getTelemetry().addData("elevator pose", elevator.getElevatorPose());
            FtcDashboard.getInstance().getTelemetry().update();

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}