package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    //drive motors
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor intakeLeft, intakeRight;
    public boolean activeOpmode;

    //jewel mechanism
    public Servo jewel, flipper;
    public ColorSensor color;
    public boolean jewelup = true;
    HardwareMap map;

    //Relic mech
    DcMotor relic;
    Servo arm, claw;

    public double oldLeftSpeed = 0, oldRightSpeed = 0, oldIntakeSpeedLeft = 0, oldIntakeSpeedRight = 0;

    public Robot(HardwareMap map) {
        this.map = map;
    }

    public void init(HardwareMap map) {
        RobotMap.INCREMENT = 0.1;
        //drivetrain
        leftFront = map.dcMotor.get("1");
        leftBack = map.dcMotor.get("2"); // changed originally rightFront
        rightFront = map.dcMotor.get("3");
        rightBack = map.dcMotor.get("4");

        jewel = map.servo.get("jewelS");
        flipper = map.servo.get("Flipper");
        color = map.colorSensor.get("color");
        intakeLeft = map.dcMotor.get("intakeLeft");
        intakeRight = map.dcMotor.get("intakeRight");

        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Driving Methods
    public void setDriveMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setTargetPos(int left, int right) {
        leftFront.setTargetPosition(left);
        rightFront.setTargetPosition(right);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behav) {
        leftFront.setZeroPowerBehavior(behav);
        leftBack.setZeroPowerBehavior(behav);
        rightFront.setZeroPowerBehavior(behav);
        rightBack.setZeroPowerBehavior(behav);
    }

    public void setDrivePower(double val) {
        setDrivePower(-val, -val);
    }

    public void setDrivePower(double left, double right) {
        left = rangeKeep(left, -1, 1);
        right = rangeKeep(right, -1, 1);

        leftBack.setPower(left);
        leftFront.setPower(left);
        rightBack.setPower(right);
        rightFront.setPower(right);
    }

    public double[] getPower() {
        return new double[]{leftBack.getPower(), leftFront.getPower(), rightBack.getPower(), rightFront.getPower()};
    }

    public double[] getPosition() {
        return new double[]{leftBack.getCurrentPosition(), leftFront.getCurrentPosition(), rightBack.getCurrentPosition(), rightFront.getCurrentPosition()};
    }

    public void smoothDrive(double left, double right) {
        oldLeftSpeed = RobotMap.smoothSpeed(left, oldLeftSpeed, RobotMap.INCREMENT);
        oldRightSpeed = RobotMap.smoothSpeed(right, oldRightSpeed, RobotMap.INCREMENT);

        left = oldLeftSpeed;
        right = oldRightSpeed;

        leftFront.setPower(left);
        rightFront.setPower(right);
        leftBack.setPower(left);
        rightBack.setPower(right);
    }

    public void smoothIntake(double speedLeft, double speedRight) {
        oldIntakeSpeedLeft = RobotMap.smoothSpeed(speedLeft, oldIntakeSpeedLeft, RobotMap.INCREMENT);
        speedLeft = oldIntakeSpeedLeft;
        oldIntakeSpeedRight = RobotMap.smoothSpeed(speedRight, oldIntakeSpeedRight, RobotMap.INCREMENT);
        speedRight = oldIntakeSpeedRight;

        this.intakeLeft.setPower(-speedLeft);
        this.intakeRight.setPower(speedRight);
    }

    public void stop() {
        setDrivePower(0);
    }

    public double rangeKeep(double x, double min, double max) {
        if (x < min) return min;
        else if (x > max) return max;
        return x;
    }

    //Jewel Mech
    public void toggleJewel() {
        if (jewel.getPosition() == 1) {
            jewel.setPosition(0);
        } else {
            jewel.setPosition(1);
        }

        jewelup = !jewelup;
    }

    public void toggleFlipper() {
        if (flipper.getPosition() == 1) {
            flipper.setPosition(0);
        } else {
            flipper.setPosition(1);
        }

        jewelup = !jewelup;
    }

}