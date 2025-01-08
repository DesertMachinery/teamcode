package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Mechanum {

    double botHeading;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    double sin = 0;
    double cos = 0;
    double max = 0;

    double frontLeftMotorPower = 0;
    double frontRightMotorPower = 0;
    double backLeftMotorPower = 0;
    double backRightMotorPower = 0;

    public Mechanum(HardwareMap hardwareMap) {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Will change to left side motors if needed after we get wheels

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match the robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void Forward(double power) {

        int angle = 0;
        double turn = 0;

        sin = Math.sin(angle - Math.PI / 4);
        cos = Math.cos(angle - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftMotorPower = power * cos/sin + turn;
        frontRightMotorPower = power * sin/cos - turn;
        backLeftMotorPower = power * sin/cos + turn;
        backRightMotorPower = power * cos/sin - turn;

        frontLeftMotor.setPower(frontLeftMotorPower);
        frontRightMotor.setPower(frontRightMotorPower);
        backLeftMotor.setPower(backLeftMotorPower);
        backRightMotor.setPower(backRightMotorPower);

    }

    public void Backwards(double power) {

        int angle = 0;
        double turn = 0;

        sin = Math.sin(angle - Math.PI / 4);
        cos = Math.cos(angle - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftMotorPower = power * cos/sin + turn;
        frontRightMotorPower = power * sin/cos - turn;
        backLeftMotorPower = power * sin/cos + turn;
        backRightMotorPower = power * cos/sin - turn;

        frontLeftMotor.setPower(-frontLeftMotorPower);
        frontRightMotor.setPower(-frontRightMotorPower);
        backLeftMotor.setPower(-backLeftMotorPower);
        backRightMotor.setPower(-backRightMotorPower);

    }

    public void Left(double power) {

        int angle = 0;
        double turn = 0;

        sin = Math.sin(angle - Math.PI / 4);
        cos = Math.cos(angle - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftMotorPower = power * cos/sin + turn;
        frontRightMotorPower = power * sin/cos - turn;
        backLeftMotorPower = power * sin/cos + turn;
        backRightMotorPower = power * cos/sin - turn;

        frontLeftMotor.setPower(frontLeftMotorPower);
        frontRightMotor.setPower(-frontRightMotorPower);
        backLeftMotor.setPower(backLeftMotorPower);
        backRightMotor.setPower(-backRightMotorPower);

    }

    public void Right(double power) {

        int angle = 0;
        double turn = 0;

        sin = Math.sin(angle - Math.PI / 4);
        cos = Math.cos(angle - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        frontLeftMotorPower = power * cos/sin + turn;
        frontRightMotorPower = power * sin/cos - turn;
        backLeftMotorPower = power * sin/cos + turn;
        backRightMotorPower = power * cos/sin - turn;

        frontLeftMotor.setPower(-frontLeftMotorPower);
        frontRightMotor.setPower(frontRightMotorPower);
        backLeftMotor.setPower(-backLeftMotorPower);
        backRightMotor.setPower(backRightMotorPower);

    }
    /*
    public void Forward() {
        double y = 1.0;
        double x = 0;
        double anotherX = 0;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(anotherX), 1);
        double frontLeftPower = (rotY + rotX + anotherX) / denominator;
        double backLeftPower = (rotY - rotX + anotherX) / denominator;
        double frontRightPower = (rotY - rotX - anotherX) / denominator;
        double backRightPower = (rotY + rotX - anotherX) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
     */
}
