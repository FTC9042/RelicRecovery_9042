package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

        robot.relic = hardwareMap.dcMotor.get("relic");
        robot.arm = hardwareMap.servo.get("arm");
        robot.claw = hardwareMap.servo.get("claw");
    }

    public void loop(){
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5)
            robot.smoothDrive(0.3*gamepad1.left_stick_y, 0.3*gamepad2.right_stick_y);
        else if (gamepad1.left_trigger>0.5)
            robot.setDrivePower(0.3* gamepad1.left_stick_y, 0.3* gamepad1.right_stick_y);
        else if (gamepad1.right_trigger>0.5)
            robot.smoothDrive(gamepad1.left_stick_y, gamepad2.right_stick_y);
        else
            robot.setDrivePower(gamepad1.left_stick_y, gamepad1.right_stick_y);

        if(gamepad1.left_bumper){
            hardwareMap.servo.get("holder").setPosition(1);
        }

        if(gamepad1.right_bumper){
            hardwareMap.servo.get("holder").setPosition(0);
        }


        robot.smoothIntake(gamepad2.left_stick_y, gamepad2.right_stick_y);

        if (gamepad2.dpad_left) {
            robot.jewel.setPosition(0.9);
        }

        if (gamepad2.dpad_right) {
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

        if(gamepad2.right_bumper){
            robot.arm.setPosition(0.45);
        }

        if(gamepad2.left_bumper){
            robot.arm.setPosition(1);
        }

        if(gamepad2.dpad_down)
            robot.relic.setPower(-1);
        else if(gamepad2.dpad_up)
            robot.relic.setPower(1);
        else
            robot.relic.setPower(0);

        robot.claw.setPosition(gamepad2.right_trigger);

    }

}
