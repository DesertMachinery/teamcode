package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;

public class ExampleAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPose);

        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        boolean finishedPath = false;

        Action driveToScore1 = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, 10, 0), Math.toRadians(90))
                .build();
        finishedPath = true;
        while (finishedPath && arm.getArmPose() > 0.3) {
            opModeIsActive();
            arm.moveToPose(0.35);

            TelemetryPacket telemetryPacket = new TelemetryPacket();
            driveToScore1.preview(telemetryPacket.fieldOverlay());
            finishedPath = driveToScore1.run(telemetryPacket);
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        }

        Action driveToPickup = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, 10, 0), Math.toRadians(90)
                        , new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30, 30))
                .build();
        finishedPath = true;
        while (finishedPath && arm.getArmPose() > 0.3) {
            opModeIsActive();
            arm.moveToPose(0.35);

            TelemetryPacket telemetryPacket = new TelemetryPacket();
            driveToPickup.preview(telemetryPacket.fieldOverlay());
            finishedPath = !driveToPickup.run(telemetryPacket);
            FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket);
        }
    }
}
