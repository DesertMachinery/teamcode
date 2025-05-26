package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ClawRotator {
    private final DcMotorEx clawRotatorMotor;

    // Positions
    public static double collectSample = 0.72;
    public static double collectSpecimen = 0.0421;
    public static double prepSpec = 0.3;
    public static double scoreSpecimen = 0.13;
    public static double elbowLowBasket = 0.68;
    public static double closedElbow = 0.1;
    private final double maxPose = 1700;

    // PID Parameters
    public static double KP = 0.0573;
    public static double KI = 0;
    public static double KD = 0.00085;
    public static double maxSpeed = 1;
    public static double maxReverseSpeed = 1;
    double lastPose = 0;
    double lastError = 0;
    double integralSum = 0;
    double error = 0;
    double derivative = 0;

    // Timer
    ElapsedTime timer = new ElapsedTime();

    public ClawRotator(HardwareMap hardwareMap) {
        clawRotatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "ClawRotator");

        clawRotatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        clawRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @SuppressWarnings("unused")
    public void setPower(double power) {
        clawRotatorMotor.setPower(power);
    }

    public void moveToPose(double pose) {

        if (pose != lastPose) {
            lastPose = pose;
            integralSum = 0;
        }

        pose *= maxPose;

        // Obtain the encoder position
        double encoderPosition = clawRotatorMotor.getCurrentPosition();

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
        clawRotatorMotor.setPower(out);

        lastError = error;
        timer.reset();
    }

    public double getPose() {
        return clawRotatorMotor.getCurrentPosition() / maxPose;
    }
}