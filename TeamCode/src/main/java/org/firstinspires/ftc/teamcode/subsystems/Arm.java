package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {

    private final DcMotorEx armMotor;

    public static double ClosedArm = 0;

    public static double lowBasket = 0.5;

    public static double highBasket = 1;

    private final double maxPose = 2253.0;

    public Arm(HardwareMap hardwareMap) {
        armMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveToPose(double pose) {

        pose *= maxPose;

        // obtain the encoder position
        double encoderPosition = armMotor.getCurrentPosition();

        // calculate the error
        double error = pose - encoderPosition;

        double out = (0.004 * error);
        armMotor.setPower(out);
    }

    public double getArmPose() {
        return armMotor.getCurrentPosition() / maxPose;
    }
}
