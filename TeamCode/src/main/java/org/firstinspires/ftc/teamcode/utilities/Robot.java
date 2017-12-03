package org.firstinspires.ftc.teamcode.utilities;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Robot {
    //drive motors
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public boolean activeOpmode;

    //jewel mechanism
    public Servo jewel;
    public ColorSensor color;
    public boolean jewelup = true;

    public double oldLeftSpeed = 0, oldRightSpeed = 0;

    public Robot(HardwareMap map) {
        init(map);
    }

    public void init(HardwareMap map) {
        RobotMap.INCREMENT = 0.1;
        //drivetrain
        leftFront = map.dcMotor.get("2");
        rightFront = map.dcMotor.get("1");
        leftBack = map.dcMotor.get("4"); // changed originally rightFront
        rightBack = map.dcMotor.get("3");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behav) {
        leftFront.setZeroPowerBehavior(behav);
        leftBack.setZeroPowerBehavior(behav);
        rightFront.setZeroPowerBehavior(behav);
        rightBack.setZeroPowerBehavior(behav);
    }

    public void setDrivePower(float val) {
        setDrivePower(-val, -val);
    }

    public void setDrivePower(double left, double right) {
        left = rangeKeep(left, -1, 1);
        right = rangeKeep(right, -1, 1);

        leftBack.setPower(left);
        leftFront.setPower(left);
        rightBack.setPower(-right);
        rightFront.setPower(-right);


    }

    public double[] getPower() {
        return new double[]{leftBack.getPower(), leftFront.getPower(), rightBack.getPower(), rightFront.getPower()};
    }

    public void smoothDrive(double left, double right) {
        oldLeftSpeed = RobotMap.smoothSpeed(left, oldLeftSpeed, RobotMap.INCREMENT);
        oldRightSpeed = RobotMap.smoothSpeed(right, oldRightSpeed, RobotMap.INCREMENT);

        left = oldLeftSpeed;
        right = oldRightSpeed;

        setDrivePower(left, right);
    }

    public void stop() {
        setDrivePower(0);
    }


    double leftError;
    double rightError;
    double leftTarget;
    double rightTarget;
    double powerD_L, powerD_R;

    public void driveDistance(float dist, LinearOpMode mode){

        leftTarget = leftFront.getCurrentPosition()+ dist;
        rightTarget = rightFront.getCurrentPosition()+ dist;

        leftError = leftTarget - leftFront.getCurrentPosition();
        rightError = rightTarget - rightFront.getCurrentPosition();

        while(mode.opModeIsActive()){
            logDistance(mode.telemetry, dist);
            if((Math.abs(leftError) < RobotMap.DRIVE_TOLERANCE && Math.abs(rightError) < RobotMap.DRIVE_TOLERANCE)) {
                break;
            }

            leftError = leftTarget - (leftFront.getCurrentPosition());
            rightError = rightTarget - (rightFront.getCurrentPosition());
            powerD_L = RobotMap.P_CONSTANT_DRIVING*leftError;
            powerD_R = RobotMap.P_CONSTANT_DRIVING*rightError;

            setDrivePower(powerD_L, powerD_R);
        }
        logDistance(mode.telemetry, dist);

        this.stop();
    }

    private void logDistance(Telemetry telemetry, double dist){

        // 0
        Logging.log("left Position", leftFront.getCurrentPosition(), telemetry);
        // 0
        Logging.log("right Position", rightFront.getCurrentPosition(), telemetry);
        // 0.0,0.0,0.0,0.0
        Logging.log("Motor Power", Arrays.toString(getPower()), telemetry);
        // 0.0, 0.0
        Logging.log("Motor Power", powerD_L +" " + powerD_R, telemetry);
        // 8256
        Logging.log("target Position", dist, telemetry);
        // true
        Logging.log("Active opmode", activeOpmode, telemetry);
        // 8256
        Logging.log("left Error", leftError, telemetry);
        // 8256
        Logging.log("right Error", rightError, telemetry);
        // false
        Logging.log("Boolean Case", Math.abs(leftError) < RobotMap.DRIVE_TOLERANCE && Math.abs(rightError) < RobotMap.DRIVE_TOLERANCE, telemetry);
        telemetry.update();
    }


    private double rangeKeep(double x, double min, double max) {
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



}