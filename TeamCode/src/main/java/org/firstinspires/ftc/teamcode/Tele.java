package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by anikaitsingh on 1/13/18.
 */

@TeleOp(name = "teleop")
public class Tele extends OpMode{
    Robot robot;

    public void init() {
        robot = new Robot(this.hardwareMap);
        HardwareMap map = this.hardwareMap;
        robot.leftFront = map.dcMotor.get("1");
        robot.leftBack = map.dcMotor.get("2"); // changed originally rightFront
        robot.rightFront = map.dcMotor.get("3");
        robot.rightBack = map.dcMotor.get("4");

        robot.jewel = map.servo.get("jewelS");
        robot.flipper = map.servo.get("Flipper");
        robot.color = map.colorSensor.get("color");
        robot.intakeLeft = map.dcMotor.get("intakeLeft");
        robot.intakeRight = map.dcMotor.get("intakeRight");

        robot.rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.jewel.scaleRange(0,1);
        robot.flipper.scaleRange(0,1);
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(){
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.setDrivePower(gamepad1.left_stick_y, gamepad1.right_stick_y);

        robot.smoothIntake(gamepad2.left_stick_y, gamepad2.right_stick_y);

        if(gamepad1.right_trigger >0.5){

        }

        if (gamepad2.dpad_up) {
            robot.jewel.setPosition(0.9);
        }

        if (gamepad2.dpad_down) {
            robot.jewel.setPosition(0);
        }

        if (gamepad2.y) {
            robot.flipper.setPosition(0);
            robot.intakeLeft.setPower(-1);
            robot.intakeRight.setPower(1);
        }

        if(gamepad2.a){
            robot.flipper.setPosition(1);
        }
    }

}
