package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous
public class Automomouse extends LinearOpMode {

    private DcMotor LeftMotor;
    private DcMotor RightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        RightMotor = hardwareMap.get(DcMotor.class, "Right");
        LeftMotor = hardwareMap.get(DcMotor.class, "Left");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while(opModeIsActive()) {
            double leftEncoder = LeftMotor.getCurrentPosition();
            double rightEncoder = RightMotor.getCurrentPosition();

            MotorLeft(0.5, 30, 2000);

        }
    }
    public void MotorForward(double power, int pos, long time) {
        RightMotor.setTargetPosition(pos);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftMotor.setTargetPosition(pos);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setPower(power);
        LeftMotor.setPower(power);

        sleep(time);
    }
    public void MotorBackward(double speed, int pos, long time) {
        RightMotor.setTargetPosition(pos);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftMotor.setTargetPosition(pos);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setPower(-speed);
        LeftMotor.setPower(-speed);

        sleep(time);
    }
    public void MotorRight(double speed, int pos, long time) {
        RightMotor.setTargetPosition(pos);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftMotor.setTargetPosition(pos);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setPower(speed);
        LeftMotor.setPower(-speed);

        sleep(time);
    }
    public void MotorLeft(double speed, int pos, long time) {
        RightMotor.setTargetPosition(pos);
        RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftMotor.setTargetPosition(pos);
        LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightMotor.setPower(-speed);
        LeftMotor.setPower(speed);

        sleep(time);
    }
    public void MotorStop() {
        RightMotor.setPower(0);
        LeftMotor.setPower(0);
    }

}


