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

    // Positions
    public static double collectSample =  0.6093333333333333;
    public static double collectSpecimen = 0.6093333333333333;
    public static double prepSpec = 0.3;
    public static double scoreSpecimen = 0.13;
    public static double elbowLowBasket = 0.624;
    public static double closedElbow = 0.1;
    private final double maxPose = 1500;
    private static double maxSpeed = 1;

    // PID Parameters
    public static double KP = 0.05;
    public static double KI = 0;
    public static double KD = 0.0002;
    double lastPose = 0;
    double lastError = 0;
    double integralSum = 0;
    double error = 0;
    double derivative = 0;

    // Timer
    ElapsedTime timer = new ElapsedTime();

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

        if (pose != lastPose) {
            lastPose = pose;
            integralSum = 0;
        }

        pose *= maxPose;

        // Obtain the encoder position
        double encoderPosition = elbowMotor.getCurrentPosition();

        // Calculate the error
        error = pose - encoderPosition;

        derivative = (error - lastError) / timer.seconds();

        // Sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (KP * error) + (KI * integralSum) + (KD * derivative);

        if(out > maxSpeed){
            out = maxSpeed;
        }

        if (out < -maxSpeed){
            out = -maxSpeed;
        }

        elbowMotor.setPower(out);

        lastError = error;
        timer.reset();
    }

    public double getElbowPose() {
        return elbowMotor.getCurrentPosition() / maxPose;
    }
}