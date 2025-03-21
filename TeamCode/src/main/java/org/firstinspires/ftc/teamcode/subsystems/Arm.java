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

    public static double collectPose = 0.86;
    public static double goDownPose = 0.8;
    public static double prepCollectPose = 0.833;
    public static double postCollectPose = 0.71;
    public static double ClosedArm = 0.1;
    public static double collectSpecimen = 0.605;
    public static double prepSpec = 0.5179;
    public static double scoreSpec = 0.6091;
    public static double armLowBasket = 0.1 ;

    public static double KP = 0.025;
    public static double KI = 0;
    public static double KD = 0.00065;




    private final double maxPose = 1000;

    ElapsedTime timer = new ElapsedTime();
    double lastPose = 0;
    double lastError = 0;
    double integralSum = 0;

    public Arm(HardwareMap hardwareMap) {
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @SuppressWarnings("unused")
    public void setPower(double power){
        armMotor.setPower(power);
    }

    public void moveToPose(double pose) {
        if(pose != lastPose){
            lastPose = pose;
            integralSum = 0;
        }

        pose *= maxPose;

        // obtain the encoder position
        double encoderPosition = armMotor.getCurrentPosition();

        // calculate the error
        double error = pose - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (KP * error) + (KI * integralSum) + (KD * derivative);

        armMotor.setPower(out);

        lastError = error;
        timer.reset();
    }
    public double getArmPose() {
        return armMotor.getCurrentPosition() / maxPose;
    }
}
