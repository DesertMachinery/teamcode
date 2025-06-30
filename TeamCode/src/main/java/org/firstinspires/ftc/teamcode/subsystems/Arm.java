package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class Arm {
    private final DcMotorEx armMotor;

    // Positions
    public static double prepCollectSample = 0.9140401146131805;
    public static double goDownPose = 0.75;
    public static double collectSample = 0.9512893982808023;
    public static double postCollectSample = 0.8;
    public static double ClosedArm = 0.1;
    public static double collectSpecimen = 0.49;
    public static double prepSpec = 0.19;
    public static double scoreSpec = 0.27;
    public static double armLowBasket = 0.4349570200573066;
    public static double maxPose = 1745;
    public static double maxSpeed = 1;

    // PID Parameters
    public static double KP = 0.01;
    public static double KI = 0;
    public static double KD = 0.0002;
    double lastPose = 0;
    double lastError = 0;
    double integralSum = 0;

    // Timer
    ElapsedTime timer = new ElapsedTime();

    public Arm(HardwareMap hardwareMap) {
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){
        armMotor.setPower(power);
    }

    public void moveToPose(double pose) {
        if(pose != lastPose){
            lastPose = pose;
            integralSum = 0;
        }

        pose *= maxPose;

        // Obtain the encoder position
        double encoderPosition = armMotor.getCurrentPosition();

        // Calculate the error
        double error = pose - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        // Sum of all error over time
        integralSum = integralSum + (error * timer.seconds());



        double out = (KP * error) + (KI * integralSum) + (KD * derivative);

        if(out > maxSpeed){
            out = maxSpeed;
        }

        if (out < -maxSpeed){
            out = -maxSpeed;
        }

        armMotor.setPower(out);

        lastError = error;
        timer.reset();
    }
    public double getArmPose() {
        return armMotor.getCurrentPosition() / maxPose;
    }

    public double getArmVel(){
        return armMotor.getVelocity()/1992.6*360;
    }
}
