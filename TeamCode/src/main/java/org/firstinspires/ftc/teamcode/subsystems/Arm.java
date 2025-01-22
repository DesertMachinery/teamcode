package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Arm {

    private final DcMotorEx armMotor;

    public static double collectPose = 0.9783;
    public static double prepCollectPose = 0.9;
    public static double ClosedArm = 0.1;
    public static double highBasket = 0.4532;



    private final double maxPose = 921;

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;
    double integralSum = 0;

    public Arm(HardwareMap hardwareMap) {
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power){
        armMotor.setPower(power);
    }

    public void moveToPose(double pose) {

        pose *= maxPose;

        // obtain the encoder position
        double encoderPosition = armMotor.getCurrentPosition();

        // calculate the error
        double error = pose - encoderPosition;

        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (0.03 * error) + (0.0007 * derivative);

        armMotor.setPower(out);

        lastError = error;
        timer.reset();
    }
    public double getArmPose() {
        return armMotor.getCurrentPosition() / maxPose;
    }
}
