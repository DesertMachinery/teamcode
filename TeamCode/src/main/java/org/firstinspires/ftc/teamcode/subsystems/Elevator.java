package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {

    private final DcMotorEx elevatorMotor;

    public static double ClosedElevator = 0;

    public static  double collectPose = 0;
    public static  double prepCollectPose = 0;

    public static double lowBasket = 0.5;

    public static double highBasket = 1.0424;

    private final double maxPose = 2253.0;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "Elevator");

        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void moveToPose(double pose) {

        pose *= maxPose;

        // obtain the encoder position
         double encoderPosition = elevatorMotor.getCurrentPosition();

        // calculate the error
        double error = pose - encoderPosition;

        double out = (0.004 * error);
        elevatorMotor.setPower(out);
    }

    public void setPower(double power){
        elevatorMotor.setPower(power);
    }

    public double getElevatorPose() {
        return elevatorMotor.getCurrentPosition() / maxPose;
    }
}
