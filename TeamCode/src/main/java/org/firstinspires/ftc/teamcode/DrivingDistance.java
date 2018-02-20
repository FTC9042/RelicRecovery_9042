package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by anikaitsingh on 2/5/18.
 */

@Autonomous(name = "Driving Distance")
public class DrivingDistance extends LinearOpMode{
    //drive motors
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    double errorLeft, errorRight;

    double positionLeftAverage, positionRightAverage;

    //PID Objects
    PID pDrivingLeft = new PID(RobotMap.P_CONSTANT_DRIVING);
    PID pDrivingRight = new PID(RobotMap.P_CONSTANT_DRIVING);


    public void initR(){
        //using hardware map to access motors
        HardwareMap map = this.hardwareMap;
        leftFront = map.dcMotor.get("1");
        leftBack = map.dcMotor.get("2"); // changed originally rightFront
        rightFront = map.dcMotor.get("3");
        rightBack = map.dcMotor.get("4");

        //setting motor behavior to brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //reseting encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //using power of motor but closed loop done without FTC Software
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //set Direction
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runOpMode(){
        initR();

        //setting target value
        double targetInches = 25;
        pDrivingLeft.setTarget(leftFront.getCurrentPosition() + targetInches * RobotMap.TICKS_PER_INCH);//25 inches
        pDrivingRight.setTarget(rightFront.getCurrentPosition() + targetInches * RobotMap.TICKS_PER_INCH);//25 inches

        errorLeft = targetInches* RobotMap.TICKS_PER_INCH - leftFront.getCurrentPosition();
        errorRight = targetInches* RobotMap.TICKS_PER_INCH - rightFront.getCurrentPosition();
        waitForStart();

        //Proportional Loop
        while(opModeIsActive() && errorLeft > RobotMap.DRIVE_TOLERANCE && errorRight > RobotMap.DRIVE_TOLERANCE){
            positionLeftAverage = leftFront.getCurrentPosition();
            positionRightAverage = rightFront.getCurrentPosition();

            errorLeft = targetInches* RobotMap.TICKS_PER_INCH - leftFront.getCurrentPosition();
            errorRight = targetInches* RobotMap.TICKS_PER_INCH - rightFront.getCurrentPosition();

            pDrivingLeft.err = errorLeft;
            pDrivingRight.err = errorRight;

            telemetry.addData("rightPosition", positionRightAverage);
            telemetry.addData("Target", targetInches * RobotMap.TICKS_PER_INCH);
            telemetry.addData("Front Left Motor Position", leftFront.getCurrentPosition());
            telemetry.addData("Front Right Motor Position", rightFront.getCurrentPosition());
            telemetry.addData("Back Left Motor Position", leftBack.getCurrentPosition());
            telemetry.addData("Back Right Motor Position", rightBack.getCurrentPosition());
            telemetry.update();

            double left = pDrivingLeft.getValue(positionLeftAverage);
            double right = pDrivingRight.getValue(positionRightAverage);

            leftFront.setPower(Range.clip(left, -1,1));
            rightFront.setPower(Range.clip(right, -1,1));
        }


        //after done displaying final value of motor
        while(opModeIsActive()){
            leftFront.setPower(0);
            rightFront.setPower(0);
            telemetry.addData("Front Left Motor Position", leftFront.getCurrentPosition());
            telemetry.addData("Front Right Motor Position", rightFront.getCurrentPosition());
            telemetry.addData("Back Left Motor Position", leftBack.getCurrentPosition());
            telemetry.addData("Back Right Motor Position", rightBack.getCurrentPosition());
            telemetry.update();
        }
    }

}
