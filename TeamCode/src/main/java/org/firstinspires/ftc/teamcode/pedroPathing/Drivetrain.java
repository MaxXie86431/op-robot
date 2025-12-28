package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    private final DcMotor FrontLeftMotorPower, FrontRightMotorPower, BackLeftMotorPower, BackRightMotorPower;

    public Drivetrain(HardwareMap hardwareMap) {


        //Initialize motors
        FrontLeftMotorPower = hardwareMap.get(DcMotorEx.class, "Top-Left-Motor");
        FrontRightMotorPower = hardwareMap.get(DcMotorEx.class, "Top-Right-Motor");
        BackLeftMotorPower = hardwareMap.get(DcMotorEx.class, "Bottom-Left-Motor");
        BackRightMotorPower = hardwareMap.get(DcMotorEx.class, "Bottom-Right-Motor");

        FrontRightMotorPower.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotorPower.setDirection(DcMotor.Direction.REVERSE);

        FrontLeftMotorPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotorPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotorPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotorPower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void drive(double leftX, double leftY, double rightX) {
        leftX = -leftX;
        leftY = leftY * 1.1;

        double frontLeftPower = leftX + leftY + rightX;
        double frontRightPower = leftX - leftY - rightX;
        double backLeftPower = leftX - leftY + rightX;
        double backRightPower = leftX + leftY - rightX;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        //get powers
        FrontLeftMotorPower.getPower();
        FrontRightMotorPower.getPower();
        BackLeftMotorPower.getPower();
        BackRightMotorPower.getPower();

        //set powers
        FrontLeftMotorPower.setPower(frontLeftPower);
        FrontRightMotorPower.setPower(frontRightPower);
        BackLeftMotorPower.setPower(backLeftPower);
        BackRightMotorPower.setPower(backRightPower);
    }
}
