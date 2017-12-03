package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends OpMode {
    //gamepad 1 is driver
    Robot robot;
    boolean smooth = false;

    public void init() {
        robot = new Robot(this.hardwareMap);
    }


    public void loop() {

        if (gamepad1.a) {
            robot.setDrivePower(-.3f);
        } else if (gamepad1.y) {
            robot.setDrivePower(.3f);
        } else if (gamepad1.b) {
            robot.setDrivePower(-.3f, .3f);
        } else if (gamepad1.x) {
            robot.setDrivePower(.3f, -.3f);
        } else {
            if (smooth) robot.smoothDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
            else robot.setDrivePower(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }

        if (gamepad2.dpad_right) {
            smooth = !smooth;
        }

        telemetry.addData("Left Stick", gamepad1.left_stick_y);
        telemetry.addData("Right Stick", gamepad1.right_stick_y);

        telemetry.addData("Left Power Front", robot.leftFront.getPower());
        telemetry.addData("Left Power Back", robot.leftBack.getPower());
        telemetry.addData("Right Power Front", robot.rightFront.getPower());
        telemetry.addData("Right Power Back", robot.rightBack.getPower());

        telemetry.addData("Jewel Position", robot.jewel.getPosition());
        telemetry.addData("Smooth: ", smooth);
        telemetry.update();
    }

    public void stop() {
        robot.setDrivePower(0, 0);
    }

}