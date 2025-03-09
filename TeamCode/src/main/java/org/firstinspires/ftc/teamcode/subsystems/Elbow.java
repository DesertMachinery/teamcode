package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Elbow {
    private final DcMotorEx elbowMotor;

    public static double collectSample = 0.95;
    public static double scoreSpecimen = 0.1;
    public static double collectSpecimen = 0.5505;
    public static double elbowLowBasket = 0.1;


    private final double maxPose = 921;

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;

    public Elbow(HardwareMap hardwareMap) {
        elbowMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Elbow");

        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @SuppressWarnings("unused")
    public void setPower(double power) {
        elbowMotor.setPower(power);
    }

    public void moveToPose(double pose) {

        pose *= maxPose;

        // obtain the encoder position
        double encoderPosition = elbowMotor.getCurrentPosition();

        // calculate the error
        double error = pose - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (0.015 * error) + (0.0003 * derivative) + ( 0 * integralSum);

        elbowMotor.setPower(out);

        lastError = error;
        timer.reset();
    }

    public double getElbowPose() {
        return elbowMotor.getCurrentPosition() / maxPose;
    }
}